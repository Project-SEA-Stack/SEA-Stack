
# H5 output reader

A flexible Python tool for quick visualizing HDF5 SEA-Stack simulation outputs.

It supports:
- Body motion (position, velocity, acceleration)
- Joint reactions (force & torque)
- Translational spring-damper actuators (TSDAs) outputs (energy, power, reaction forces)

With capabilities such as
- Multiple devices plotting
- Various naming conventions
- Filtering via CLI
- File introspection (`--list`)

---------

## Basic command structure:

run h5 output reader python file in terminal
- located in data/demos/run_seastack/features
  - either cd data/demos/run_seastack/features or direct to data/demos/run_seastack/features/h5outputsReader.py

```bash
python h5outputsReader.py <path_to_h5_output_file/results.x.h5> --field <field> [options] --bodies <body> [options] --dof <dof> [options]  --joints <joints> [options] --tsda <tsda> [options] --list --mooring <field> [index] --mooring_path <path>
```

## Features

### `--field`
Select what data to plot.

**Body fields:**
- position
- velocity
- acceleration  

**Joint fields:**
- joint_force
- joint_torque  

**TSDA fields:**
- energy
- power
- tsda_force  

---

### `--bodies`
Select body numbers to plot on same figure (default = all)

**Used with:** body fields  

**Example:**
```bash
--bodies 1 2 4
```

---

### `--dof`
Select degrees of freedom to plot on separate figures (default = heave)

Options:
- surge
- sway
- heave  
- roll
- pitch
- yaw

**Used with:** body fields  (not acceleration)

**Example:**
```bash
--dof heave
```

or for multiple plots:
```bash
--dof surge sway heave
```

---

### `--joints`
Select joint names to plot

**Used with:** joint fields  

**Tip:** use `--list` to see available joint names  

**Example:**
```bash
--joints float_plate_joint
```

can plot multiple joint parameters on same plot:
```bash
--joints joint_12 joint_34
```

---

### `--tsdas`
Select TSDA names to plot

**Used with:** TSDA fields  

**Example:**
```bash
--tsdas ram_23_stbd ram_45_stbd
```

**Special option:**
```bash
--tsdas all
```

- Plots the sum of all TSDAs (for energy and power fields specification)
- Will prints:
  - total energy
  - average power
  - peak power  

---

### `--list`
List all available joint and TSDA names in the file for force and torque data

**Example:**
```bash
--list
```

---

### `--mooring`
Load and visualize mooring output data from .out files
Supports:
- force Fx, Fy, Fz [on body]
- moment Mx, My, Mz [on body]
- tension [on line]

With the same `--mooring` command and [integer] for respective field

Mooring directory can be specified with `--mooring_path`

**Example:**
```bash
--mooring tension 1 3 5 --mooring_path <path_to/mooring>
```
--mooring <field> [index] --mooring_path <path>


---

## Full example usage

### Plot heave position (all bodies) of 5sa demo - bodies not specified

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5 --field position --dof heave
```
<img width="572" height="456" alt="image" src="https://github.com/user-attachments/assets/ee8b9a63-f13a-43c3-975d-c065858f9389" />

---

### Plot sway velocity (of selected bodies) of 5sa model
(note: order bodies are listed can change plotting order!)

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5 --field velocity --dof sway --bodies 3 5 
```
<img width="570" height="445" alt="image" src="https://github.com/user-attachments/assets/b2b534de-e555-4c8d-bc37-362e7c4e1c60" />

---

### Plot total TSDA power of RM3

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5  --field power
```
<img width="630" height="442" alt="image" src="https://github.com/user-attachments/assets/a2eaa409-3586-41a5-b81f-955f2293e2b3" />

---

### List available joint force outputs for the 5sa
```bash
python h5outputsReader.py 5sa/regular_waves/outputs/results.regular.h5  --list
```

<img width="252" height="522" alt="image" src="https://github.com/user-attachments/assets/87dd47c3-8137-4b86-bb24-5afb171ecc31" />

---

### Plot joint torque for RM3

```bash
python h5outputsReader.py ../rm3/regular_waves/outputs/results.regular.h5  --field joint_torque --joints float_plate_joint
```
<img width="576" height="441" alt="image" src="https://github.com/user-attachments/assets/7f5bce72-d031-4455-bee2-21938041a05e" />

---

### Plot selected TSDA forces (between body 1 and 2) of 5sa (tsda names from list function output)

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5 --field tsda_force --tsdas ram_12_top ram_12_bottom ram_12_stbd ram_12_port
```

<img width="580" height="450" alt="image" src="https://github.com/user-attachments/assets/fdae318d-a2b5-4c4c-aa1e-5241d1cc52cb" />

---

### Plot selected TSDA energy of 5sa (tsda names from list function output)

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5  --field energy --tsdas ram_23_stbd ram_45_stbd
```
<img width="544" height="446" alt="image" src="https://github.com/user-attachments/assets/fe8867e5-04ba-4741-ab7c-b06f86fd7fb4" />

---

### Plot total TSDA power of 5sa

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5  --field energy --tsdas all
```

<img width="588" height="439" alt="image" src="https://github.com/user-attachments/assets/44591a55-c582-444f-8489-832ed803cc99" />

<img width="341" height="37" alt="image" src="https://github.com/user-attachments/assets/6fde763d-1551-4517-8a30-3fb9c486f3f8" />

---

### Plot moment on body 2 of 5sa

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5  --mooring moment 2
```
<img width="556" height="449" alt="image" src="https://github.com/user-attachments/assets/6d2f0dd3-4477-4486-bebe-b05b3c872e8d" />

---

### Plot tension on lines of RM3 specifying path

```bash
python h5outputsReader.py ../5sa/regular_waves/outputs/results.regular.h5  --mooring tension --mooring_path C:\Users\ariley\source\repos\sea-stack\build\data\demos\run_seastack\rm3\assets\mooring
```
<img width="713" height="443" alt="image" src="https://github.com/user-attachments/assets/4580dd69-9077-4d01-9a02-c07048c3b857" />

---
