"""Run inside Fusion 360's Python environment with Complete Assembly active.

Read-only with respect to the CAD design. Writes a small, deterministic JSON
manifest of configured occurrence transforms to the project output directory.
"""
import adsk.core
import adsk.fusion
import json
import os


OUTPUT_PATH = r"C:\Users\leahc\Documents\Sparx\code\sparx_description\cad_export\cad_geometry.json"
EXPECTED_DESIGN = "Complete Assembly"
TARGET_PATHS = {
    "base_reference": "/Body Frame:1",
    "chassis_axes_reference": "/Left Side:1",
    "head": "/Face:1/Head:1",
    "camera_assembly": "/Face:1/Head:1/Electronics:1/Gemini 355 asm:1",
    "camera_model": "/Face:1/Head:1/Electronics:1/Gemini 355 asm:1/Gemini 335:1",
    "lidar_model": "/Face:1/Head:1/Electronics:1/Lidar ASm:1/RPLIDAR C1:1",
}


def _matrix_rows(matrix):
    return [[matrix.getCell(r, c) for c in range(4)] for r in range(4)]


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        raise RuntimeError("No active Fusion design")
    if design.rootComponent.name != EXPECTED_DESIGN:
        raise RuntimeError(
            "Expected active root component {!r}, found {!r}".format(
                EXPECTED_DESIGN, design.rootComponent.name
            )
        )

    found = {}
    found_matrices = {}
    found_components = {}
    occurrence_count = 0

    def walk(component, path, parent_transform):
        nonlocal occurrence_count
        for occurrence in component.occurrences:
            occurrence_count += 1
            full_path = path + "/" + occurrence.name
            transform = parent_transform.copy()
            transform.transformBy(occurrence.transform2)
            if full_path in TARGET_PATHS.values():
                found[full_path] = {
                    "path": full_path,
                    "occurrence_name": occurrence.name,
                    "component_name": occurrence.component.name,
                    "root_transform_cm": _matrix_rows(transform),
                }
                found_matrices[full_path] = transform.copy()
                found_components[full_path] = occurrence.component
            walk(occurrence.component, full_path, transform)

    walk(design.rootComponent, "", adsk.core.Matrix3D.create())
    missing = [path for path in TARGET_PATHS.values() if path not in found]
    if missing:
        raise RuntimeError("Configured CAD occurrence path(s) not found: " + ", ".join(missing))

    root_datums = {}
    for key, feature_name in (
        ("base_link", "ROS_base_link_origin"),
        ("base_footprint", "ROS_base_footprint_origin"),
    ):
        point = design.rootComponent.constructionPoints.itemByName(feature_name)
        if not point:
            raise RuntimeError("Required ROS datum point is missing: " + feature_name)
        geometry = point.geometry
        root_datums[key] = {
            "feature_name": feature_name,
            "origin_root_cm": [geometry.x, geometry.y, geometry.z],
            "axes_in_root": "Fusion root X/Y/Z; configured as ROS forward/left/up",
        }

    camera_assembly = found_components[TARGET_PATHS["camera_assembly"]]
    mount = camera_assembly.constructionPoints.itemByName("Mounting point datum point")
    if not mount:
        raise RuntimeError("Required camera mounting datum point is missing")
    mount_point = mount.geometry.copy()
    mount_point.transformBy(found_matrices[TARGET_PATHS["camera_assembly"]])
    camera_mount_datum = {
        "feature_name": mount.name,
        "parent_occurrence": TARGET_PATHS["camera_assembly"],
        "origin_root_cm": [mount_point.x, mount_point.y, mount_point.z],
    }

    manifest = {
        "schema_version": 1,
        "design_root": design.rootComponent.name,
        "length_unit": design.unitsManager.defaultLengthUnits,
        "fusion_internal_length_unit": "cm",
        "occurrence_count": occurrence_count,
        "occurrences": {
            key: found[path] for key, path in TARGET_PATHS.items()
        },
        "ros_datums": root_datums,
        "base_axes_rotation_root": [row[:3] for row in found[TARGET_PATHS["chassis_axes_reference"]]["root_transform_cm"][:3]],
        "base_axes_reference": TARGET_PATHS["chassis_axes_reference"],
        "camera_mount_datum": camera_mount_datum,
        "transform_convention": (
            "4x4 column-vector transform; translation in Fusion internal cm; "
            "nested occurrence transforms composed from root to leaf"
        ),
        "origin_note": (
            "Transforms locate CAD occurrence origins. They do not establish "
            "sensor measurement origins unless the mapping supplies a verified offset."
        ),
    }
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w", encoding="utf-8") as output:
        json.dump(manifest, output, indent=2, sort_keys=True)
        output.write("\n")
    print("Wrote CAD geometry manifest: " + OUTPUT_PATH)
    print("Occurrences inspected: {}".format(occurrence_count))
