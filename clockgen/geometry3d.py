"""True-3D mesh builders for the table clock.

The watch (watchgen) lives happily in flat extrusions; a spherical
tourbillon does not.  Everything here is built additively from
parametric vertex grids -- surfaces of revolution, tubes swept along
3D paths, spherical shell panels -- so no mesh booleans are needed.
Overlapping sub-solids are fine: slicers (and the renderer) union them.
"""
import numpy as np
import trimesh


# ------------------------------------------------------------- transforms
def rotm(axis, deg, point=(0, 0, 0)):
    return trimesh.transformations.rotation_matrix(
        np.radians(deg), axis, point)


def transm(v):
    m = np.eye(4)
    m[:3, 3] = v
    return m


def frame(origin=(0, 0, 0), zdir=(0, 0, 1), xhint=(1, 0, 0)):
    """4x4 frame whose +Z is zdir, located at origin."""
    z = np.asarray(zdir, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(xhint, float)
    x = x - z * (x @ z)
    if np.linalg.norm(x) < 1e-8:
        x = np.array([0.0, 1.0, 0.0]) - z * z[1]
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = x, y, z, origin
    return m


def put(mesh, M):
    """Copy of mesh with transform applied."""
    m = mesh.copy()
    m.apply_transform(M)
    return m


def merge(*meshes):
    return trimesh.util.concatenate([m for m in meshes if m is not None])


# ------------------------------------------------------- grid -> trimesh
def _grid_mesh(P, close_u=False, close_v=False):
    """Quad grid P[nu, nv, 3] -> triangulated trimesh."""
    nu, nv = P.shape[:2]
    V = P.reshape(-1, 3)
    F = []
    ru = range(nu if close_u else nu - 1)
    rv = range(nv if close_v else nv - 1)
    for i in ru:
        i2 = (i + 1) % nu
        for j in rv:
            j2 = (j + 1) % nv
            a, b = i * nv + j, i * nv + j2
            c, d = i2 * nv + j2, i2 * nv + j
            F.append([a, b, c])
            F.append([a, c, d])
    m = trimesh.Trimesh(vertices=V, faces=np.array(F), process=False)
    m.update_faces(m.nondegenerate_faces())
    return m


def lathe(profile, n=48):
    """Revolve a closed (r, z) polyline fully around +Z.

    profile: list of (r, z); the loop is closed automatically.
    r=0 points are fine (they collapse to the axis).
    """
    pr = np.asarray(profile, float)
    if np.linalg.norm(pr[0] - pr[-1]) > 1e-9:
        pr = np.vstack([pr, pr[0]])
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    P = np.empty((n, len(pr), 3))
    P[..., 0] = np.cos(th)[:, None] * pr[None, :, 0]
    P[..., 1] = np.sin(th)[:, None] * pr[None, :, 0]
    P[..., 2] = pr[None, :, 1]
    return _grid_mesh(P, close_u=True, close_v=False)


def tube(path, r, n=8, closed=False, cap=True, r2=None):
    """Sweep a circle of radius r along path (Nx3); parallel-transport
    frames.  r may taper linearly to r2.  closed=True joins end to start
    (torus-like); cap=True closes the ends with fans."""
    p = np.asarray(path, float)
    N = len(p)
    # tangents
    t = np.gradient(p, axis=0)
    if closed:
        t[0] = t[-1] = (p[1] - p[-1])
    t /= np.linalg.norm(t, axis=1)[:, None]
    # parallel transport normal
    u = np.array([0.0, 0.0, 1.0])
    if abs(u @ t[0]) > 0.9:
        u = np.array([1.0, 0.0, 0.0])
    u = u - t[0] * (u @ t[0])
    u /= np.linalg.norm(u)
    U = np.empty_like(p)
    for i in range(N):
        if i:
            u = u - t[i] * (u @ t[i])
            u /= np.linalg.norm(u)
        U[i] = u
    V = np.cross(t, U)
    rr = np.linspace(r, r if r2 is None else r2, N)
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.cos(a)[None, :, None] * U[:, None, :] + \
        np.sin(a)[None, :, None] * V[:, None, :]
    P = p[:, None, :] + rr[:, None, None] * ring
    m = _grid_mesh(P, close_u=closed, close_v=True)
    if not closed and cap:
        caps = []
        for k, flip in ((0, True), (N - 1, False)):
            c = p[k]
            vs = np.vstack([P[k], c])
            fs = [[i, (i + 1) % n, n] for i in range(n)]
            cm = trimesh.Trimesh(vs, np.array(fs), process=False)
            if flip:
                cm.invert()
            caps.append(cm)
        m = merge(m, *caps)
    return m


def arc_path(R, a0_deg, a1_deg, n=48, c=(0, 0, 0)):
    """Arc in the XY plane around c, radius R."""
    a = np.radians(np.linspace(a0_deg, a1_deg, n))
    return np.column_stack([c[0] + R * np.cos(a),
                            c[1] + R * np.sin(a),
                            np.full(n, c[2])])


def ring_torus(R, r, n_main=48, n_tube=8):
    return tube(arc_path(R, 0, 360 * (1 - 1.0 / n_main), n_main), r,
                n=n_tube, closed=True)


def sphere_panel(R, t, lat0, lat1, lon0=0.0, lon1=360.0, n_lat=10,
                 n_lon=48):
    """Thick spherical shell panel between latitudes/longitudes (deg).
    Full longitude span -> closed band (no side walls needed)."""
    full = abs((lon1 - lon0) - 360.0) < 1e-6
    la = np.radians(np.linspace(lat0, lat1, n_lat))
    lo = np.radians(np.linspace(lon0, lon1, n_lon, endpoint=not full))

    def shell(rad):
        P = np.empty((len(lo), len(la), 3))
        P[..., 0] = rad * np.cos(la)[None, :] * np.cos(lo)[:, None]
        P[..., 1] = rad * np.cos(la)[None, :] * np.sin(lo)[:, None]
        P[..., 2] = rad * np.sin(la)[None, :]
        return P
    Po, Pi = shell(R), shell(R - t)
    mo = _grid_mesh(Po, close_u=full)
    mi = _grid_mesh(Pi[:, ::-1], close_u=full)
    walls = []
    # latitude edge walls
    for j, flip in ((0, False), (len(la) - 1, True)):
        W = np.stack([Po[:, j], Pi[:, j]], axis=2).transpose(0, 2, 1)
        w = _grid_mesh(W, close_u=full)
        if flip:
            w.invert()
        walls.append(w)
    if not full:
        for i, flip in ((0, True), (len(lo) - 1, False)):
            W = np.stack([Po[i], Pi[i]], axis=1)
            w = _grid_mesh(W)
            if flip:
                w.invert()
            walls.append(w)
    return merge(mo, mi, *walls)


def sphere_arc(R, lat0, lat1, lon_deg, r_tube, n=40, n_tube=8):
    """Meridian rib: tube along a constant-longitude arc of a sphere."""
    la = np.radians(np.linspace(lat0, lat1, n))
    lo = np.radians(lon_deg)
    path = np.column_stack([R * np.cos(la) * np.cos(lo),
                            R * np.cos(la) * np.sin(lo),
                            R * np.sin(la)])
    return tube(path, r_tube, n=n_tube)


# ----------------------------------------------------------- primitives
def cyl(r, h, z0=0.0, n=32, r2=None):
    """Cylinder (or cone if r2 given) along +Z from z0 to z0+h."""
    if r2 is None:
        r2 = r
    return lathe([(0, z0), (r, z0), (r2, z0 + h), (0, z0 + h)], n=n)


def ball(R, c=(0, 0, 0), sub=2):
    m = trimesh.creation.icosphere(subdivisions=sub, radius=R)
    m.apply_translation(c)
    return m


def box(ext, c=(0, 0, 0)):
    m = trimesh.creation.box(extents=ext)
    m.apply_translation(c)
    return m


def crown_teeth(R, n_teeth, w, h, depth, z0=0.0, tilt=0.0):
    """Ring of radial teeth (small boxes) standing on z0, tips outward.
    tilt (deg) leans the teeth radially -- bevel-gear look."""
    teeth = []
    for i in range(n_teeth):
        a = 360.0 * i / n_teeth
        t = box((depth, w, h), (0, 0, h / 2))
        if tilt:
            t.apply_transform(rotm([0, 1, 0], tilt))
        t.apply_translation([R, 0, z0])
        t.apply_transform(rotm([0, 0, 1], a))
        teeth.append(t)
    return merge(*teeth)


def extrude_flat(poly2d, h, z0=0.0):
    """shapely polygon -> solid (same as watchgen.geometry.extrude)."""
    meshes = []
    geoms = poly2d.geoms if hasattr(poly2d, "geoms") else [poly2d]
    for g in geoms:
        if g.is_empty or g.area < 1e-6:
            continue
        m = trimesh.creation.extrude_polygon(g, height=h)
        m.apply_translation([0, 0, z0])
        meshes.append(m)
    return merge(*meshes)
