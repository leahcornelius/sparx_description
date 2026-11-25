#!/usr/bin/env python3
"""Generate fixed-joint Xacro from Fusion occurrence transforms + reviewed mapping."""
import argparse
import json
import math
from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit("PyYAML is required (install python3-yaml in ROS 2)") from exc


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inverse_rigid(t):
    out = [[t[j][i] if i < 3 and j < 3 else 0.0 for j in range(4)] for i in range(4)]
    for i in range(3):
        out[i][3] = -sum(out[i][k] * t[k][3] for k in range(3))
    out[3][3] = 1.0
    return out


def fusion_to_metres(transform_cm):
    result = [row[:] for row in transform_cm]
    for i in range(3):
        result[i][3] *= 0.01
    return result


def matrix_to_rpy(t):
    pitch = math.asin(max(-1.0, min(1.0, -t[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(t[2][1], t[2][2])
        yaw = math.atan2(t[1][0], t[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-t[0][1], t[1][1])
    return [roll, pitch, yaw]


def fmt(values):
    return " ".join("{:.9g}".format(v) for v in values)


def locate(manifest, path):
    for entry in manifest["occurrences"].values():
        if entry["path"] == path:
            return fusion_to_metres(entry["root_transform_cm"])
    raise ValueError("Occurrence path absent from manifest: " + path)


def datum_transform(manifest, name):
    try:
        xyz_cm = manifest["ros_datums"][name]["origin_root_cm"]
        rotation = manifest["base_axes_rotation_root"]
    except KeyError as exc:
        raise ValueError("CAD datum or chassis axes absent from manifest: " + name) from exc
    if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
        raise ValueError("base_axes_rotation_root must be a 3x3 matrix")
    transform = [list(row) + [0.0] for row in rotation]
    transform.append([0.0, 0.0, 0.0, 1.0])
    for i in range(3):
        transform[i][3] = float(xyz_cm[i]) * 0.01
    return transform


def frame_transform(manifest, mapping, occurrence_key, parent_transform):
    ref = mapping["occurrences"][occurrence_key]
    transform = locate(manifest, ref["path"])
    offset = ref.get("measurement_origin_offset_m") or [0.0, 0.0, 0.0]
    axes = ref.get("measurement_axes_rpy_rad") or [0.0, 0.0, 0.0]
    local = rpy_matrix(axes)
    for i in range(3):
        local[i][3] = offset[i]
    # Appending the rotations after the local origin translation preserves the
    # extracted sensor measurement-origin position.
    cad_mount = rpy_matrix(ref.get("cad_mounting_correction_rpy_rad") or [0.0, 0.0, 0.0])
    measurement = rpy_matrix(ref.get("measurement_frame_correction_rpy_rad") or [0.0, 0.0, 0.0])
    local = multiply(local, multiply(cad_mount, measurement))
    return multiply(inverse_rigid(parent_transform), multiply(transform, local))


def main():
    here = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=Path, default=here / "config/cad_mapping.yaml")
    parser.add_argument("--manifest", type=Path, default=here / "cad_export/cad_geometry.json")
    parser.add_argument("--output", type=Path, default=here / "urdf/generated_geometry.xacro")
    parser.add_argument("--allow-unvalidated", action="store_true",
                        help="generate candidate transforms while emitting prominent warnings")
    args = parser.parse_args()

    mapping = yaml.safe_load(args.mapping.read_text(encoding="utf-8"))
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1:
        raise SystemExit("Unsupported CAD manifest schema")
    if manifest.get("design_root") != mapping["fusion"]["expected_design_name"]:
        raise SystemExit("Manifest root does not match configured Fusion design")

    base_datum_name = mapping["fusion"]["root_to_ros"]["datum_name"]
    footprint_datum_name = mapping["frames"]["base_footprint"]["datum_name"]
    cad_base = datum_transform(manifest, base_datum_name)
    cad_footprint = datum_transform(manifest, footprint_datum_name)
    footprint_to_base = multiply(inverse_rigid(cad_footprint), cad_base)
    head = frame_transform(manifest, mapping, "head", cad_base)
    lidar = frame_transform(manifest, mapping, "lidar_model", multiply(cad_base, head))
    camera = frame_transform(manifest, mapping, "camera_model", multiply(cad_base, head))

    invalid = []
    if not mapping["fusion"]["root_to_ros"].get("validated", False):
        invalid.append("CAD chassis axes/origin → base_link are provisional")
    for key in ("camera_model", "lidar_model"):
        if not mapping["occurrences"][key].get("origin_validated", False):
            invalid.append(key + " occurrence origin is a provisional sensor origin")
    if invalid and not args.allow_unvalidated:
        raise SystemExit("Refusing unvalidated geometry. Resolve: " + "; ".join(invalid)
                         + ". Use --allow-unvalidated for a candidate only.")

    joints = [
        ("base_footprint_to_base_link", "base_footprint", "base_link", footprint_to_base, "pending chassis-to-ground datum"),
        ("base_link_to_head_link", "base_link", "head_link", head, "Fusion head occurrence origin"),
        ("head_link_to_laser_frame", "head_link", "laser_frame", lidar, "CAD occurrence origin proxy; verify C1 scan centre and axes"),
        ("head_link_to_camera_link", "head_link", "camera_link", camera, "CAD occurrence origin proxy; verify Orbbec mount/driver root"),
    ]
    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sparx_generated">',
        '  <!-- Generated from cad_geometry.json. Candidate only until mapping validation is true. -->',
    ]
    for name, parent, child, transform, note in joints:
        lines += [
            "  <!-- {} -->".format(note),
            '  <joint name="{}" type="fixed">'.format(name),
            '    <parent link="{}"/><child link="{}"/>'.format(parent, child),
            '    <origin xyz="{}" rpy="{}"/>'.format(fmt([transform[i][3] for i in range(3)]), fmt(matrix_to_rpy(transform))),
            "  </joint>",
        ]
    lines.append("</robot>")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("Generated:", args.output)
    for warning in invalid:
        print("WARNING:", warning, file=sys.stderr)
    print("WARNING: camera mount joint must not duplicate a driver-published parent edge", file=sys.stderr)


if __name__ == "__main__":
    main()
