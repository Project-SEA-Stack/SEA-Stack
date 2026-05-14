
# 5sa model YAML WRITER 

import math
import yaml
from pathlib import Path

# ----------------------------
# Inputs
# ----------------------------
SEGMENT_LENGTH = 36.0
GAP = 0.0
N_SEGMENTS = 5
DIAMETER = 4.0
spacing = SEGMENT_LENGTH + GAP
TSDA_radius = 1.5

RHO = 1025.0
G = 9.81

TOTAL_MASS = 1.35e6 

COG_Z_OFFSET = -0.2
DRAFT = 0.259199          # center of object location (positive = below waterline)
OUTPUT_FILE = Path(__file__).resolve().parent / "spreading" / "5sa_spreading_auto.model.yaml"

#TSDA coefficients
STIFFNESS = 100000
DAMPING = 2000000
FREE_LENGTH = 2.0

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def inertia_cylinder(m, r, L):
    Ixx = 0.5 * m * r**2
    Iyy = (1/12) * m * (3*r**2 + L**2)
    return [Ixx, Iyy, Iyy]


def joint_location(i):
    """i = 0 means joint between body1-body2"""
    x = SEGMENT_LENGTH + i * spacing
    return [x, 0.0, -DRAFT]


def ram_points(i, y_offset, z_offset):
    """Generate TSDA endpoints for a joint"""
    x = SEGMENT_LENGTH + i * spacing
    return (
        [x - 1.0, y_offset, z_offset],
        [x + 1.0, y_offset, z_offset],
    )


def build_yaml():
    r = DIAMETER / 2.0
    mass_per_segment = TOTAL_MASS / N_SEGMENTS

    bodies = []
    joints = []
    tsdas = []

    spacing = SEGMENT_LENGTH + GAP

    for i in range(N_SEGMENTS):
        cx = SEGMENT_LENGTH / 2 + i * spacing

        body = {
        "name": f"body{i+1}",
        "location": [cx, 0.0, -DRAFT],
        "mass": mass_per_segment,
        "fixed": False,
        "inertia": {
            "moments": inertia_cylinder(mass_per_segment, r, SEGMENT_LENGTH),
            "products": [0, 0, 0],
        },
        "com": {
            "location": [0, 0, COG_Z_OFFSET],
            "orientation": [0, 0, 0],
        },
        "visualization": {
            "model_file": f"../assets/geometry/segment_{i+1}.obj",
            "color": [0.15, 0.35 + 0.05*i, 0.7],
        }}      

        bodies.append(body)              

    ground_body = {
    "name": "ground",
    "location": [90.0, 0.0, -10.0],
    "mass": 1,
    "fixed": True,
    "inertia": {
        "moments": [1, 1, 1],
        "products": [0, 0, 0],
    },
    "com": {
        "location": [0, 0, 0],
        "orientation": [0, 0, 0],
    },}

    bodies.append(ground_body)

    for i in range(N_SEGMENTS - 1):
        xj = SEGMENT_LENGTH * (i + 1) + GAP * i

        joints.append({
            "name": f"joint_{i+1}{i+2}",
            "type": "UNIVERSAL",
            "body1": f"body{i+1}",
            "body2": f"body{i+2}",
            "location": [xj, 0.0, -DRAFT],
            "axis1": [0, 1, 0],
            "axis2": [0, 0, 1],
        })
        
        for name, y, z in [
            ("top", 0.0, TSDA_radius),
            ("bottom", 0.0, -TSDA_radius),
            ("port", TSDA_radius, 0.0),
            ("stbd", -TSDA_radius, 0.0),
        ]:
            p1 = [xj - 1.0, y, -DRAFT + z]
            p2 = [xj + 1.0, y, -DRAFT + z]

            tsdas.append({
                "name": f"ram_{i+1}{i+2}_{name}",
                "type": "TSDA",
                "body1": f"body{i+1}",
                "body2": f"body{i+2}",
                "point1": p1,
                "point2": p2,
                "spring_coefficient": STIFFNESS,
                "damping_coefficient": DAMPING,
                "free_length": FREE_LENGTH,
                "visualization": {
                "type": "SPRING",
                "radius": 0.15,
                "resolution": 65,
                "turns": 10,
            },
            })

    model_name = OUTPUT_FILE.name.replace(".yaml", "").replace(".", "_")
    
    yaml_data = {
        "chrono-version": 10.0,
        "model": {
            "name": model_name,
            "angle_degrees": False,
            "data_path": {"type": "RELATIVE", "root": "."},
            "bodies": bodies,
            "joints": joints,
            "tsdas": tsdas,
        }
    }

    return yaml_data


def write_yaml(data, path):
    with open(path, "w") as f:
        yaml.safe_dump(
        data,
        f,
        sort_keys=False,
        default_flow_style=None)


if __name__ == "__main__":
    data = build_yaml()

    write_yaml(data, OUTPUT_FILE)

    print(f"YAML written to: {OUTPUT_FILE}")

