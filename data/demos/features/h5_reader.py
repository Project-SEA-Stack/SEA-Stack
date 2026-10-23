
import h5py
import matplotlib.pyplot as plt
import argparse
import numpy as np
from pathlib import Path
import re


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("path", nargs="?", default=None,
                        help="Define path to HDF5 file")

    parser.add_argument("--bodies", nargs="*", type=int,
                        help="Body number(s) (default: all)")

    parser.add_argument("--field", nargs="+",
                    default=["position"],
                    choices=["position", "velocity", "acceleration", "energy", "power", "joint_force", "joint_torque", "tsda_force"],
                    help="Data to plot (can pass multiple)")

    parser.add_argument("--dof", nargs="+",
                        default= ["heave"],
                        choices=["surge", "sway", "heave"],
                        help="Degree of freedom(s)")
    
    parser.add_argument("--joints", nargs="*", type=str,
                        help="Joint number(s) ie. 12 23 34 etc. (default: all)")

    parser.add_argument("--tsdas", nargs="*", type=str,
                    help="TSDA names (default: all)")
    
    parser.add_argument("--list", action="store_true",
                    help="List available joint and TSDA force outputs")
    
    parser.add_argument(
        "--mooring", nargs="+",
        help="Mooring mode: <field> [index]")
    
    parser.add_argument(
        "--mooring_path", type=str, default=None,
        help="Path to mooring folder (default: same location as outputs)")

    return parser.parse_args()

DOF_MAP = {
    "surge": 0,
    "sway": 1,
    "heave": 2
}

JOINT_MAP = {
    "joint_force": "reaction1_force",
    "joint_torque": "reaction1_torque"
}

BODY_FIELDS = {"position", "velocity", "acceleration"}
TSDA_FIELDS = {"energy", "power", "tsda_force"}
JOINT_FIELDS = {"joint_force", "joint_torque"}

TSDA_MAP = {
    "tsda_force": "reaction_force_body1"
}

FIELD_META = {
    "position": {
        "title": "Body Position",
        "ylabel": "Position [m]"
    },
    "velocity": {
        "title": "Body Velocity",
        "ylabel": "Velocity [m/s]"
    },
    "acceleration": {
        "title": "Body Acceleration",
        "ylabel": "Acceleration [m/s²]"
    },
    "energy": {
        "title": "Cumulative Energy",
        "ylabel": "Energy [J]"
    },
    "power": {
        "title": "Instantaneous Power",
        "ylabel": "Power [W]"
    },
    "joint_force": {
        "title": "Reaction Force",
        "ylabel": "Force [N]"
    },
    "reactTorque": {
        "title": "Reaction Torque",
        "ylabel": "Torque [N m]"
    }
}

FIELD_META["tsda_force"] = {
    "title": "TSDA Reaction Force (Body 1)",
    "ylabel": "Force [N]"
}

def load_data(path, bodies, fields, joints, tsdas):
    import numpy as np

    data = {}

    with h5py.File(path, "r") as f:

        time = f["results/time/time"][:]

        if any(f in {"position", "velocity", "acceleration"} for f in fields):

            all_bodies = [
                b for b in f["results/model/bodies"].keys()
                if b != "ground"
            ]

            selected_bodies = (
                all_bodies if bodies is None else [f"body{b}" for b in bodies]
            )

            for body in selected_bodies:
                data[body] = {}

                for field in fields:
                    if field in {"position", "velocity", "acceleration"}:
                        data[body][field] = f[
                            f"results/model/bodies/{body}/{field}"
                        ][:]

        if any(f in {"joint_force", "joint_torque"} for f in fields):

            all_joints = list(f["results/model/joints"].keys())
            selected_joints = all_joints if joints is None else joints

            for joint in selected_joints:
                key = f"joint_{joint}" if not joint.startswith("joint_") else joint

                data[key] = {}   

                for field in fields:
                    if field in JOINT_MAP:
                        path = f"results/model/joints/{joint}/{JOINT_MAP[field]}"

                        if path in f:   
                            data[key][field] = f[path][:]


        if any(f in TSDA_FIELDS for f in fields):

            tsda_group = f["results/model/tsdas"]
            all_tsdas = [k for k in tsda_group.keys() if k != "names"]

            use_total = tsdas is not None and "all" in tsdas

            if use_total:
                total_energy = None
                total_power = None

                for name in all_tsdas:
                    tsda = tsda_group[name]

                    if "energy" in fields and "absorbed_energy" in tsda:
                        e = tsda["absorbed_energy"][:]
                        total_energy = e if total_energy is None else total_energy + e

                    if "power" in fields and "absorbed_power" in tsda:
                        p = tsda["absorbed_power"][:]
                        total_power = p if total_power is None else total_power + p

                data["TSDA_total"] = {}

                if total_energy is not None:
                    data["TSDA_total"]["energy"] = total_energy

                if total_power is not None:
                    data["TSDA_total"]["power"] = total_power

            else:
                selected_tsdas = all_tsdas if tsdas is None else tsdas

                for name in selected_tsdas:

                    if name not in tsda_group:
                        continue

                    tsda = tsda_group[name]
                    key = f"tsda_{name}"

                    data[key] = {}

                    for field in fields:

                        if field == "energy":
                            data[key][field] = tsda["absorbed_energy"][:]

                        elif field == "power":
                            data[key][field] = tsda["absorbed_power"][:]

                        elif field in TSDA_MAP:
                            h5name = TSDA_MAP[field]

                            if h5name in tsda:
                                data[key][field] = tsda[h5name][:]

    return time, data

def plot_data(time, data, fields, dofs=None, joints=None):

    if isinstance(dofs, str):
        dofs = [dofs]

    for field in fields:
        if field in BODY_FIELDS:

            if not dofs:
                raise ValueError(f"{field} requires at least one DOF")

            for dof in dofs:
                plt.figure()

                meta = FIELD_META.get(field, {})
                title = f"{meta.get('title', field)} ({dof})"
                ylabel = meta.get("ylabel", field)

                for key, group in data.items():

                    if field not in group:
                        continue

                    y = group[field]
                    idx = DOF_MAP[dof]
                    y = y[:, idx]

                    plt.plot(time, y, label=key)

                plt.title(title)
                plt.xlabel("Time [s]")
                plt.ylabel(ylabel)
                plt.legend()
                plt.grid(True)

        elif field in JOINT_FIELDS or field in TSDA_FIELDS:
            plt.figure()

            meta = FIELD_META.get(field, {})
            title = meta.get("title", field)
            ylabel = meta.get("ylabel", field)
            # label = key.replace("tsda_", "").replace("joint_", "")
            
            for key, group in data.items():

                if field not in group:
                    continue

                label = key.replace("tsda_", "").replace("joint_", "")

                y = group[field]

                if len(y.shape) == 2:
                    y = y[:, 2]  # default to heave (z)
                
                plt.plot(time, y, label=label)

            plt.title(title)
            plt.xlabel("Time [s]")
            plt.ylabel(ylabel)
            plt.legend()
            plt.grid(True)
    plt.show()

    # input("Press Enter OR close all plots to continue...\n")
    # plt.close("all")


def list_outputs(path):
    with h5py.File(path, "r") as f:

        print("\n--- JOINT OUTPUTS ---")
        joints = f["results/model/joints"]

        for joint_name in joints.keys():
            joint = joints[joint_name]

            if "reaction1_force" in joint:
                print(f"{joint_name} → joint_force")

            if "reaction1_torque" in joint:
                print(f"{joint_name} → joint_torque")

        print("\n--- TSDA OUTPUTS ---")
        tsdas = f["results/model/tsdas"]

        for name in tsdas.keys():
            if name == "names":
                continue

            tsda = tsdas[name]

            if "reaction_force_body1" in tsda:
                print(f"{name} → tsda_force")

        print()




def load_mooring(folder, field, index=None):
    """
    folder: mooring directory
    lines: list[int] or None or ["all"]
    bodies: list[int] or None
    fields: ["tension", "force", "moment"]
    """

    files = list(Path(folder).glob("*.out"))

    line_files = sorted(Path(folder).glob("*Line*.out"))
    body_files = sorted(Path(folder).glob("*Body*.out"))

    # ---------------- LINE TENSION ----------------
    if field == "tension":
        data= {}
        for f in line_files:

            match = re.search(r"_Line(\d+)", f.stem)

            if not match:
                continue

            line_id = int(match.group(1))

            if index is not None and line_id != index:
                continue

            with open(f) as file:
                header = file.readline().split()

            raw = np.loadtxt(f, skiprows=1)

            time = raw[:, 0]

            tension_cols = [i for i, h in enumerate(header) if "Te" in h]

            if not tension_cols:
                continue

            max_tension = raw[:, tension_cols].max(axis=1)

            yield line_id, time, max_tension
            

    elif field in ["force", "moment"]:

        for f in body_files:

            match = re.search(r"Body(\d+)", f.stem)
            body_id = int(match.group(1)) if match else f.stem

            if index is not None and body_id != index:
                continue

            raw = np.loadtxt(f, skiprows=1)
            time = raw[:, 0]

            if field == "force":
                data = raw[:, 1:4]
            else:
                data = raw[:, 4:7]

            yield body_id, time, data, field
    return data



def plot_mooring(results, field):

    if field in ["force", "moment"]:
        labels_map = {"force": ["Fx", "Fy", "Fz"], "moment": ["Mx", "My", "Mz"]}
        comp_labels = labels_map[field]
        ylabel = {"force": "Force [N]", "moment": "Moment [N·m]"}[field]
        
        # Plot each body on a separate figure with all components
        for result in results:
            id_, t, y, f = result
            if y.ndim == 2:
                plt.figure()
                plt.plot(t, y[:, 0], label=comp_labels[0])
                plt.plot(t, y[:, 1], label=comp_labels[1])
                plt.plot(t, y[:, 2], label=comp_labels[2])
                plt.title(f"Mooring {field} (Body {id_})")
                plt.xlabel("Time [s]")
                plt.ylabel(ylabel)
                plt.legend()
                plt.grid(True)
                plt.show()
    else:
        # Original tension plotting
        plt.figure()
        for result in results:
            id_, t, y = result
            plt.plot(t, y, label=f"Line {id_}")
        plt.title(f"Mooring {field}")
        plt.xlabel("Time [s]")
        ylabel = "Tension [N]"
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)
        plt.show()


def main():
    args = parse_args()

    if args.list:
        if args.path is None:
            raise ValueError("You must provide a path to an HDF5 file to list outputs.")
        list_outputs(args.path)
        return

    run_h5 = any([
        args.field != ["position"],
        args.bodies is not None,
        args.joints is not None,
        args.tsdas is not None
    ])

    run_mooring = args.mooring is not None

    # ---------------- HDF5 ----------------
    if run_h5:
        time, data = load_data(
            args.path,
            args.bodies,
            args.field,
            args.joints,
            args.tsdas
        )

        plot_data(time, data, args.field, args.dof, args.joints)

        if "TSDA_total" in data:
            if "energy" in data["TSDA_total"]:
                total_energy = data["TSDA_total"]["energy"][-1]
                print(f"\nTotal absorbed energy (all TSDAs): {total_energy:.3f} J")

            if "power" in data["TSDA_total"]:
                power = data["TSDA_total"]["power"]
                print(f"Average power (all TSDAs): {power.mean():.3f} W")
                print(f"Peak power (all TSDAs): {power.max():.3f} W")

    # ---------------- MOORING ----------------
    if run_mooring:
        field = args.mooring[0]
        index = args.mooring[1] if len(args.mooring) > 1 else None

        if index is not None:
            index = int(index)

        if args.mooring_path:
            folder = Path(args.mooring_path).resolve()
        else:
            if args.path:
                run_dir = Path(args.path).resolve().parents[1]
            else:
                run_dir = Path.cwd().resolve()
            folder = run_dir / "mooring"

        mooring_data = list(load_mooring(folder, field=field, index=index))
        plot_mooring(mooring_data, field)

    # ---------------- DEFAULT ----------------
    if not run_h5 and not run_mooring:
        time, data = load_data(
            args.path,
            bodies=None,
            fields=["position"],
            joints=None,
            tsdas=None
        )

        plot_data(time, data, ["position"], ["heave"], None)


if __name__ == "__main__":
    main()

