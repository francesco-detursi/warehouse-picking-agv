# Warehouse Picking AGV

Single-robot warehouse collection simulation developed in MATLAB. A mission starts at the waiting station, visits selected loading docks, unloads at the dispatch station and returns to wait. The application combines interactive task selection, route planning, motion execution and simulated pedestrian traffic.

**[Read the detailed technical report (PDF)](docs/Warehouse_AGV_Detailed_Report.pdf)**

## Overview

- Select racks directly on the map and drag entries to set their visit order.
- Follow the selected order or minimise the computed route-distance cost by enumerating rack permutations.
- Plan footprint-aware routes with A* search, line-of-sight simplification and checked Bézier transitions.
- Execute loading and unloading as timed service stops with station alignment.
- Simulate continuous pedestrian traffic with a selectable count and random seed; slow, wait or replan when appropriate.
- Save mission trajectories, events and metrics for subsequent analysis.

## Reported results

For the same four-rack request without pedestrians, optimising the visit order reduced travelled distance from 91.21 m to 69.86 m (23.41%). Mission duration fell from 254.82 s to 236.83 s (7.06%), including 44 s of service time in both cases. All nine missions documented in the report completed their requested services and return; six included pedestrian traffic.

These results describe the evaluated configurations, not a general performance guarantee. The report explains the scenarios, metric definitions and limitations. Historical result folders are not included in this repository; new runs generate their own outputs.

## Requirements

- MATLAB R2025b, the version used for development and evaluation.
- A desktop MATLAB session for the graphical interface.

No external datasets, ROS installation or hardware are needed. The supplied application uses MATLAB functions; no additional toolbox dependency has been identified in the source. Compatibility with older releases has not been established.

## Quick start

1. Select **Code → Download ZIP** on GitHub and extract the archive.
2. Open MATLAB and set **Current Folder** to the extracted folder containing `startWarehouse.m`.
3. In the Command Window, run:

```matlab
startWarehouse
```

The entry point adds `src` to the MATLAB path, checks the warehouse layout and opens the mission interface. Preserve all folder names, including the `+` prefixes used by MATLAB packages.

### Run a mission

1. Choose **No pedestrians** for an initial run, or **Random pedestrians** and set **People** and **Seed**.
2. Click the racks to serve. Drag entries in the selected-rack list to change their order; use **Clear selected** or **Clear all** to remove them.
3. Choose **Follow selected order** or **Optimise visit order**.
4. Select **Plan mission** to inspect the route and planned distance.
5. Select **Run mission** to calculate the mission and open its replay. Close the replay before starting another run.

Simulation is computed before replay opens. Calculation time depends on the mission and computer; replay speed does not alter the computed results. **Save plan** exports the plan without executing it.

### Reproduce the four-rack comparison

Select **No pedestrians** and the racks in this order: **R02, R05, R04, R03**. Run once with **Follow selected order**, then plan and run the same selected racks with **Optimise visit order**. Compare `TravelledDistance_m` and `Duration_s` in the two `metrics.csv` files. Keep the map and motion settings unchanged. The full evaluation conditions are documented in the report.

## Repository contents

```text
warehouse-picking-agv/
├── README.md
├── .gitignore
├── startWarehouse.m
├── src/
│   └── +warehouse/
│       ├── +config/       Robot and motion settings
│       ├── +maps/         Warehouse geometry and stations
│       ├── +mission/      Requests, selection and plan saving
│       ├── +planning/     Routes, visit order and replanning
│       ├── +execution/    Motion execution and result saving
│       ├── +geometry/     Footprints and clearance checks
│       ├── +safety/       Simulated pedestrians and supervision
│       ├── +validation/   Layout checks used at startup
│       └── +view/         Interface, replay and figure exports
└── docs/
    └── Warehouse_AGV_Detailed_Report.pdf
```

The `+safety` package name refers to simulation supervision, not certified safety functionality.

## Generated outputs

Runs are saved automatically under `results/mission_<timestamp>/`, including:

- `mission_run.mat`: structured simulation data.
- `trajectory.csv`: sampled robot trajectory.
- `events.csv`: mission and service events.
- `metrics.csv`: mission-level performance metrics.
- Additional pedestrian, supervisor and replanning logs where applicable.

Saved plans and exported figures also use the local `results` directory. This directory is excluded from version control by `.gitignore` to keep the repository focused on the application and report.

## Scope

The model assumes a known map, ideal localisation and perception, kinematic robot motion and cooperative pedestrians. It is an offline simulation with stepwise feedback, not a hardware controller or safety-certified system. Visit-order selection is exact for the stored route-cost matrix at the current small scale; it does not establish a globally shortest continuous trajectory or minimum mission time.

## Author

Francesco De Tursi — MSc student in Mechanical Engineering, University of Bologna.

[LinkedIn](https://www.linkedin.com/in/francescodetursi/) · [GitHub](https://github.com/francesco-detursi) · [Email](mailto:ffrancesco.detursi@gmail.com)
