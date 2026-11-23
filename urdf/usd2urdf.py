#!/usr/bin/env python3
"""Convert Omniverse / Isaac Sim camera USD assets to URDF + STL.

Design notes
------------
* Stage ``metersPerUnit`` is applied to every translation and vertex.
* Only *rigid* local transforms become URDF ``<origin>``.  URDF cannot express
  scale, so any non-unit scale is baked into the exported mesh vertices.
* A ``Mesh`` prim whose local transform is identity is merged into its parent
  Xform link (no extra link/joint), which keeps the tree readable.
* USD camera prims use the Omniverse/OpenGL convention (X right, Y up, Z back,
  looking down -Z).  ROS optical frames are (X right, Y down, Z forward), so a
  180 deg rotation about X is emitted as ``<name>_optical``.
* Intrinsics cannot live in URDF -> sidecar ``<model>_camera_info.yaml``.
* ``Render`` / ``Graph`` / ``Looks`` subtrees are ignored.

Usage
-----
    python3 usd2urdf.py "Camera Models/*.usd" -o out
    python3 usd2urdf.py "Camera Models/S56C.usd" -o out --mass 0.15 --no-collision
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, Gf
from scipy.spatial.transform import Rotation

SKIP_NAMES = {"Render", "Graph", "Looks"}
T_XFORM = UsdGeom.Tokens.Xform
T_MESH = UsdGeom.Tokens.Mesh
T_CAMERA = UsdGeom.Tokens.Camera
GEOM_TYPES = {T_XFORM, T_MESH, T_CAMERA}

IDENTITY_TOL = 1e-6
SCALE_TOL = 1e-6


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def sanitize(name: str) -> str:
    """URDF link names: keep [A-Za-z0-9_-], map everything else to '_'."""
    cleaned = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not cleaned:
        cleaned = "link"
    if cleaned[0].isdigit():
        cleaned = "_" + cleaned
    return cleaned


def decompose(matrix: Gf.Matrix4d) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (translation, rotation 3x3, scale) as a URDF-style column transform.

    ``Gf.Matrix4d`` uses the row-vector convention (``p_out = p_in * M``) with
    the translation in the **last row**.  URDF uses column vectors
    (``p_out = R @ p_in + t``), so the linear part must be transposed and the
    translation is simply ``M[3, :3]``.
    """
    m = np.array([[matrix[r][c] for c in range(4)] for r in range(4)], dtype=float)
    t = m[3, :3].copy()
    lin = m[:3, :3].T.copy()          # column-vector convention

    scale = np.linalg.norm(lin, axis=0)
    scale[scale < 1e-12] = 1.0
    rot = lin / scale

    ortho_err = float(np.max(np.abs(rot.T @ rot - np.eye(3))))
    if ortho_err > 1e-4:
        print(f"    ! shear in local transform (err={ortho_err:.2e})",
              file=sys.stderr)

    u, _, vt = np.linalg.svd(rot)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        rot = -rot
    return t, rot, scale


def matrix_to_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    """URDF uses fixed-axis XYZ, i.e. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    rpy = Rotation.from_matrix(rot).as_euler("xyz", degrees=False)
    return tuple(0.0 if abs(v) < 1e-12 else float(v) for v in rpy)


def fmt(v: float) -> str:
    if abs(v) < 5e-13:
        return "0"
    return f"{v:.9g}"


def triangulate(counts, indices):
    tris = []
    cursor = 0
    for n in counts:
        idx = indices[cursor:cursor + n]
        cursor += n
        for i in range(1, n - 1):
            tris.append((idx[0], idx[i], idx[i + 1]))
    return np.asarray(tris, dtype=np.int64)


# --------------------------------------------------------------------------- #
# data model
# --------------------------------------------------------------------------- #
@dataclass
class Link:
    name: str
    path: str
    visuals: list = field(default_factory=list)   # list of (mesh_rel_path,)
    is_camera: bool = False


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: tuple


# --------------------------------------------------------------------------- #
# conversion
# --------------------------------------------------------------------------- #
def convert(usd_path: Path, out_dir: Path, *, mass=None, collision=True,
            verbose=True, model_name=None, authored_optical=False) -> dict:
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"cannot open {usd_path}")

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    if up_axis != UsdGeom.Tokens.z:
        print(f"  ! upAxis={up_axis}, URDF expects Z-up; result may be rotated",
              file=sys.stderr)

    default_prim = stage.GetDefaultPrim()
    root_path = str(default_prim.GetPath()) if default_prim else "/"

    model_name = sanitize(model_name or usd_path.stem)
    excluded = set()
    if authored_optical:
        compute = stage.GetPrimAtPath('/Graph/ROS_TF/ComputePublisherTF')
        if compute:
            for path in compute.GetRelationship('inputs:targetPrims').GetTargets():
                frame = stage.GetPrimAtPath(path)
                if frame.GetCustomDataByKey('mountPoseStatus') == 'provisional_body_origin':
                    excluded.add(path)
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    links: dict[str, Link] = {}
    joints: list[Joint] = []
    used_names: set[str] = set()
    warnings: list[str] = []
    cameras: list[dict] = []
    stats = {"meshes": 0, "faces": 0, "skipped": 0}

    def unique(name: str) -> str:
        base = name
        i = 1
        while name in used_names:
            name = f"{base}_{i}"
            i += 1
        used_names.add(name)
        return name

    root_name = unique("base_link")
    links[root_name] = Link(name=root_name, path="/")

    def skipped(prim) -> bool:
        return any(p in SKIP_NAMES for p in prim.GetPath().pathString.split("/"))

    # ---- pass 1: collect geometry prims -------------------------------- #
    cache = UsdGeom.XformCache()

    def world_scale(prim) -> np.ndarray:
        """Accumulated scale from the stage root down to ``prim`` (inclusive).

        URDF links/joints cannot carry scale, so any scale inherited from a
        parent Xform must be baked into translations and mesh vertices.
        """
        _, _, sc = decompose(cache.GetLocalToWorldTransform(prim))
        return np.asarray(sc, dtype=float)

    published: dict[str, str] = {}   # prim path -> link name
    order: list = []

    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if skipped(prim) or prim.GetPath() in excluded:
            continue
        if prim.GetTypeName() not in GEOM_TYPES:
            continue
        xformable = UsdGeom.Xformable(prim)
        local = xformable.GetLocalTransformation()
        t, rot, scale = decompose(local)

        rigid_identity = (
            np.allclose(t, 0, atol=IDENTITY_TOL)
            and np.allclose(rot, np.eye(3), atol=1e-6)
            and np.allclose(scale, 1, atol=1e-6)
        )

        parent_prim = prim.GetParent()
        parent_path = str(parent_prim.GetPath())
        parent_link = published.get(parent_path)
        # every translation is expressed in the parent's (possibly scaled) frame
        parent_sc = world_scale(parent_prim)
        prim_sc = world_scale(prim)

        # merge identity-transform Mesh into parent link, keep the tree small
        if (prim.GetTypeName() == T_MESH and rigid_identity
                and parent_link is not None and not prim.IsInstanceProxy()):
            prim_name = unique(sanitize(prim.GetName()))
            published[str(prim.GetPath())] = parent_link
            order.append((prim, prim_name, parent_link, None, None, prim_sc))
            continue

        prim_name = unique(sanitize(prim.GetName()))
        parent_link = parent_link or root_name
        published[str(prim.GetPath())] = prim_name
        order.append((prim, prim_name, parent_link, t, rot, prim_sc, parent_sc))

    # ---- pass 2: emit links / joints / meshes -------------------------- #
    for entry in order:
        prim, name, parent_link = entry[0], entry[1], entry[2]
        t, rot = entry[3], entry[4]
        prim_sc = entry[5]
        parent_sc = entry[6] if len(entry) > 6 else np.ones(3)
        path = str(prim.GetPath())
        typ = prim.GetTypeName()

        if t is None:  # merged mesh prim
            link = links[parent_link]
        else:
            link = Link(name=name, path=path, is_camera=(typ == T_CAMERA))
            links[name] = link
            # translations are in the parent frame, which may be scaled
            xyz = np.asarray(t, dtype=float) * mpu * parent_sc
            rpy = matrix_to_rpy(rot)
            joints.append(Joint(
                name=f"{parent_link}_to_{name}",
                parent=parent_link,
                child=name,
                xyz=xyz,
                rpy=rpy,
            ))

        if typ == T_MESH:
            geom = UsdGeom.Mesh(prim)
            points = geom.GetPointsAttr().Get()
            counts = geom.GetFaceVertexCountsAttr().Get()
            indices = geom.GetFaceVertexIndicesAttr().Get()
            if not points or not counts:
                warnings.append(f"{path}: mesh has no geometry (unresolved reference?)")
                stats["skipped"] += 1
                continue

            pts = np.asarray([[float(v[0]), float(v[1]), float(v[2])] for v in points])
            pts = pts * mpu * prim_sc

            faces = triangulate(counts, indices)
            if geom.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded:
                faces = faces[:, ::-1]

            mesh_file = f"{name}.stl"
            write_stl(mesh_dir / mesh_file, pts, faces)
            link.visuals.append(f"meshes/{mesh_file}")
            stats["meshes"] += 1
            stats["faces"] += len(faces)

        elif typ == T_CAMERA:
            intr = None if prim.GetCustomDataByKey('calibrationStatus') == 'pending' else read_intrinsics(prim, mpu)
            if intr:
                intr["link"] = name
                cameras.append(intr)
                if verbose:
                    print(f"    camera {path}: {intr['width']}x{intr['height']} "
                          f"fx={intr['fx']:.2f} fy={intr['fy']:.2f} "
                          f"cx={intr['cx']:.2f} cy={intr['cy']:.2f} "
                          f"model={intr['distortion_model']}")
            else:
                warnings.append(f"{path}: camera prim without intrinsics")
            if authored_optical:
                continue
            # ROS optical frame for the camera link
            optical = unique(f"{name}_optical")
            links[optical] = Link(name=optical, path=path + "/optical")
            joints.append(Joint(
                name=f"{name}_optical_joint",
                parent=name,
                child=optical,
                xyz=np.zeros(3),
                rpy=(math.pi, 0.0, 0.0),
            ))

    # ---- write URDF ----------------------------------------------------- #
    up = []
    up.append('<?xml version="1.0"?>')
    up.append(f'<!-- Generated by usd2urdf.py from {usd_path.name} -->')
    up.append(f'<!-- metersPerUnit={mpu} upAxis={up_axis} -->')
    up.append(f'<robot name="{model_name}">')

    for link in links.values():
        up.append(f'  <link name="{link.name}">')
        for rel in link.visuals:
            up.append('    <visual>')
            up.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            up.append('      <geometry>')
            up.append(f'        <mesh filename="{rel}"/>')
            up.append('      </geometry>')
            up.append('    </visual>')
            if collision:
                up.append('    <collision>')
                up.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                up.append('      <geometry>')
                up.append(f'        <mesh filename="{rel}"/>')
                up.append('      </geometry>')
                up.append('    </collision>')
        if mass and link.visuals:
            inertia = box_inertia(link.name, mesh_dir, mass, link.visuals[0])
            up.append(f'    <inertial>')
            up.append(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
            up.append(f'      <mass value="{fmt(mass)}"/>')
            up.append(f'      <inertia ixx="{fmt(inertia[0])}" ixy="0" ixz="0" '
                      f'iyy="{fmt(inertia[1])}" iyz="0" izz="{fmt(inertia[2])}"/>')
            up.append('    </inertial>')
        up.append('  </link>')

    for j in joints:
        up.append(f'  <joint name="{j.name}" type="fixed">')
        up.append(f'    <parent link="{j.parent}"/>')
        up.append(f'    <child link="{j.child}"/>')
        up.append(f'    <origin xyz="{fmt(j.xyz[0])} {fmt(j.xyz[1])} {fmt(j.xyz[2])}" '
                  f'rpy="{fmt(j.rpy[0])} {fmt(j.rpy[1])} {fmt(j.rpy[2])}"/>')
        up.append('  </joint>')

    up.append('</robot>')
    up.append('')

    urdf_path = out_dir / f"{model_name}.urdf"
    urdf_path.write_text("\n".join(up))

    info_paths = []
    for intr in cameras:
        suffix = "" if len(cameras) == 1 else f"_{intr['link']}"
        info_paths.append(write_camera_info(
            out_dir / f"{model_name}{suffix}_camera_info.yaml",
            f"{model_name}{suffix}", intr))

    return {
        "model": model_name,
        "urdf": urdf_path,
        "links": len(links),
        "joints": len(joints),
        "meshes": stats["meshes"],
        "faces": stats["faces"],
        "skipped": stats["skipped"],
        "warnings": warnings,
        "intrinsics": info_paths,
        "cameras": cameras,
        "camera": cameras[0] if cameras else None,
    }


def write_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    import trimesh
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.export(str(path), file_type="stl")


def read_intrinsics(prim, mpu) -> dict:
    """Pull intrinsics from the omni lensdistortion attributes.

    Handles both ``opencvPinhole`` (rational_polynomial) and ``opencvFisheye``
    (equidistant) sub-namespaces; the model namespace is not always identical
    to what ``cameraProjectionType`` reports.
    """
    def get(name, default=None):
        return prim.GetAttribute(name).Get() if prim.HasAttribute(name) else default

    model_ns = get("omni:lensdistortion:model", "")
    candidates = []
    if model_ns:
        candidates.append(f"omni:lensdistortion:{model_ns}:")
    for fallback in ("opencvPinhole", "opencvFisheye"):
        prefix = f"omni:lensdistortion:{fallback}:"
        if prefix not in candidates:
            candidates.append(prefix)

    prefix = next((p for p in candidates if get(p + "fx") is not None), None)
    if prefix is None:
        return {}

    is_fisheye = "fisheye" in prefix.lower() or "equidistant" in str(
        get("cameraProjectionType", "")).lower()

    size = get(prefix + "imageSize")
    data = {
        "distortion_model": "equidistant" if is_fisheye else "rational_polynomial",
        "fx": get(prefix + "fx"),
        "fy": get(prefix + "fy"),
        "cx": get(prefix + "cx"),
        "cy": get(prefix + "cy"),
        "width": int(size[0]) if size else None,
        "height": int(size[1]) if size else None,
        "projection": get("cameraProjectionType", "unknown"),
        "focal_length_mm": (get("focalLength") or 0.0) * mpu * 1000.0,
        "hfov_deg": None,
        "vfov_deg": None,
    }

    if is_fisheye:
        # fisheye only carries k1..k4, no tangential terms
        data.update({
            "k1": get(prefix + "k1", 0.0) or 0.0,
            "k2": get(prefix + "k2", 0.0) or 0.0,
            "k3": get(prefix + "k3", 0.0) or 0.0,
            "k4": get(prefix + "k4", 0.0) or 0.0,
            "k5": 0.0, "k6": 0.0, "p1": 0.0, "p2": 0.0,
        })
    else:
        data.update({
            "k1": get(prefix + "k1", 0.0) or 0.0,
            "k2": get(prefix + "k2", 0.0) or 0.0,
            "k3": get(prefix + "k3", 0.0) or 0.0,
            "k4": get(prefix + "k4", 0.0) or 0.0,
            "k5": get(prefix + "k5", 0.0) or 0.0,
            "k6": get(prefix + "k6", 0.0) or 0.0,
            "p1": get(prefix + "p1", 0.0) or 0.0,
            "p2": get(prefix + "p2", 0.0) or 0.0,
        })

    ha, va, fl = (get("horizontalAperture"), get("verticalAperture"),
                  get("focalLength"))
    if ha and fl:
        data["hfov_deg"] = math.degrees(2 * math.atan(ha / 2 / fl))
    if va and fl:
        data["vfov_deg"] = math.degrees(2 * math.atan(va / 2 / fl))
    return data


def write_camera_info(path: Path, name: str, intr: dict) -> Path:
    lines = [
        f"# Generated by usd2urdf.py from the {name} camera prim in the USD asset",
        f"# ROS camera_info equivalent (intrinsics cannot be stored in URDF)",
        f"image_width: {intr['width']}",
        f"image_height: {intr['height']}",
        f"camera_name: {name}",
        f"camera_matrix:",
        f"  rows: 3",
        f"  cols: 3",
        f"  data: [{intr['fx']:.6f}, 0.0, {intr['cx']:.6f},",
        f"         0.0, {intr['fy']:.6f}, {intr['cy']:.6f},",
        f"         0.0, 0.0, 1.0]",
        f"distortion_model: {intr['distortion_model']}",
    ]
    if intr["distortion_model"] == "equidistant":
        lines += [
            "distortion_coefficients:",
            "  rows: 1",
            "  cols: 4",
            f"  data: [{intr['k1']:.9g}, {intr['k2']:.9g}, "
            f"{intr['k3']:.9g}, {intr['k4']:.9g}]",
        ]
    else:
        lines += [
            "distortion_coefficients:",
            "  rows: 1",
            "  cols: 8",
            f"  data: [{intr['k1']:.9g}, {intr['k2']:.9g}, {intr['p1']:.9g}, "
            f"{intr['p2']:.9g}, {intr['k3']:.9g}, {intr['k4']:.9g}, "
            f"{intr['k5']:.9g}, {intr['k6']:.9g}]",
        ]
    lines += [
        f"# projection type: {intr['projection']}",
        f"# focal length: {intr['focal_length_mm']:.3f} mm",
        f"# hfov: {intr['hfov_deg']:.3f} deg, vfov: {intr['vfov_deg']:.3f} deg",
    ]
    path.write_text("\n".join(lines) + "\n")
    return path


def box_inertia(link_name: str, mesh_dir: Path, mass: float, rel: str):
    import trimesh
    mesh = trimesh.load(str(mesh_dir.parent / rel), force="mesh")
    ext = mesh.extents
    x, y, z = float(ext[0]), float(ext[1]), float(ext[2])
    return (
        mass / 12.0 * (y * y + z * z),
        mass / 12.0 * (x * x + z * z),
        mass / 12.0 * (x * x + y * y),
    )


# --------------------------------------------------------------------------- #
# cli
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("patterns", nargs="+", help="USD/USDA/USDZ files or globs")
    ap.add_argument("-o", "--out", default="out", help="output directory (default: out)")
    ap.add_argument("--mass", type=float, default=None,
                    help="add <inertial> with this total mass (kg)")
    ap.add_argument("--no-collision", action="store_true", help="omit <collision>")
    args = ap.parse_args()

    files: list[Path] = []
    for pattern in args.patterns:
        found = sorted(glob.glob(pattern))
        if not found and Path(pattern).exists():
            found = [pattern]
        files.extend(Path(f) for f in found)
    files = [f for f in files if f.suffix.lower() in {".usd", ".usda", ".usdc", ".usdz"}]

    if not files:
        print("no USD files matched", file=sys.stderr)
        return 2

    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)

    results = []
    for usd in files:
        print(f"[+] {usd.name}")
        target = out_root / sanitize(usd.stem)
        target.mkdir(parents=True, exist_ok=True)
        try:
            res = convert(usd, target, mass=args.mass,
                          collision=not args.no_collision)
        except Exception as exc:  # noqa: BLE001
            print(f"    FAILED: {exc}", file=sys.stderr)
            results.append({"model": usd.stem, "error": str(exc)})
            continue
        print(f"    -> {res['urdf'].relative_to(out_root)}  "
              f"links={res['links']} joints={res['joints']} "
              f"meshes={res['meshes']} faces={res['faces']}"
              + (f" skipped={res['skipped']}" if res["skipped"] else ""))
        for w in res["warnings"]:
            print(f"    ! {w}")
        results.append(res)

    print("\n=== summary ===")
    ok = [r for r in results if "error" not in r]
    print(f"converted {len(ok)}/{len(results)} files")
    for r in ok:
        cams = r.get("cameras") or []
        extra = ""
        if cams:
            extra = "  cam " + ", ".join(
                f"{c['width']}x{c['height']} fx={c['fx']:.2f}" for c in cams)
        print(f"  {r['model']:42s} links={r['links']:3d} joints={r['joints']:3d}"
              f" meshes={r['meshes']:3d}{extra}")
    return 0 if len(ok) == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
