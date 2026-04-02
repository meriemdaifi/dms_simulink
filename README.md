# DMS ADAS Simulink Project

A complete **Driver Monitoring System (DMS)** for Advanced Driver Assistance Systems (ADAS), implemented as a MATLAB script that programmatically generates and runs a full Simulink model.

## Overview

This project builds an end-to-end DMS Simulink model entirely from MATLAB code — no manual Simulink editor work required. The system monitors the driver's state through simulated sensors and provides graduated alerts and vehicle interventions when impairment is detected.

## Architecture

```
┌─────────────────┐    ┌──────────────────────────┐    ┌─────────────────────┐    ┌──────────────────┐
│  Sensor Inputs   │───▶│  Driver State Estimation  │───▶│  Alert & Intervention│───▶│ Dashboard & Log  │
│                  │    │                            │    │                      │    │                  │
│ • PERCLOS        │    │ • Drowsiness Estimator     │    │ • Alert Level Gen    │    │ • Scopes         │
│ • Blink Duration │    │   (weighted fusion)        │    │   (5 levels: 0–4)   │    │ • To Workspace   │
│ • Yawn Flag      │    │ • Distraction Detector     │    │ • Visual/Audio/      │    │ • Time-series    │
│ • Gaze Angle     │    │   (gaze + head logic)      │    │   Haptic alerts     │    │   logging        │
│ • Head Yaw/Pitch │    │ • Fatigue Estimator        │    │ • LKA Controller     │    │                  │
│ • Vehicle Speed  │    │   (steering + lane + speed) │    │ • Speed Controller   │    │                  │
│ • Steering Torque│    │ • Risk Fusion (max)        │    │ • Emergency Stop     │    │                  │
│ • Lateral Dev.   │    │                            │    │                      │    │                  │
└─────────────────┘    └──────────────────────────┘    └─────────────────────┘    └──────────────────┘
```

### Subsystems

| Subsystem | Description |
|-----------|-------------|
| **Sensor Inputs** | 9 simulated sensor channels (sine-wave generators with bias, noise, and saturation) |
| **Driver State Estimation** | Three MATLAB Function blocks computing drowsiness, distraction, and fatigue scores (0–1), fused into an overall risk score |
| **Alert & Intervention** | Converts risk into 5 alert levels with visual/audio/haptic outputs; includes lane-keeping assist and speed reduction controllers |
| **Dashboard & Logging** | Scope displays and To-Workspace blocks for real-time monitoring and post-simulation analysis |

### Detection Algorithms

- **Drowsiness**: Weighted fusion of PERCLOS (40%), blink duration (25%), head nod pitch (20%), and yawn duration (15%)
- **Distraction**: Gaze-off-road angle (60%) + head turn yaw (40%), normalised against configurable thresholds
- **Fatigue**: Steering variability (40%) + lateral deviation (40%), scaled by speed factor

### Alert Levels

| Level | Name | Trigger (Risk) | Response |
|-------|------|----------------|----------|
| 0 | None | < 0.3 | No action |
| 1 | Low | ≥ 0.3 | Visual indicator |
| 2 | Medium | ≥ 0.5 | Audio + visual; gentle speed reduction |
| 3 | High | ≥ 0.7 | All modalities; moderate speed reduction |
| 4 | Critical | ≥ 0.9 | Emergency stop initiated |

## File Structure

```
dms_simulink/
├── dms_config.m          # Configuration parameters and thresholds
├── dms_adas_project.m    # Main script: builds, wires, and runs the Simulink model
├── dms_test.m            # Validation and test suite (7 tests)
└── README.md             # This file
```

## Quick Start

### Prerequisites

- MATLAB R2020b or later
- Simulink
- Stateflow (for MATLAB Function blocks)

### Run the Project

```matlab
% 1. Load configuration into workspace
dms_config

% 2. Build the model, simulate, and visualise results
dms_adas_project
```

The script will:
1. Create a new Simulink model `DMS_ADAS_Model.slx`
2. Build all four subsystems with internal blocks and wiring
3. Connect subsystems at the top level
4. Configure MATLAB Function blocks with detection/control algorithms
5. Run a 300-second simulation
6. Generate three analysis figures (driver state, alerts, intervention)
7. Print summary statistics

### Run Tests

```matlab
dms_test
```

Runs 7 automated tests covering model structure, parameter sanity, simulation smoke test, signal range validation, threshold boundaries, and intervention response.

## Configuration

All tunable parameters are centralised in `dms_config.m`:

| Category | Key Parameters |
|----------|---------------|
| **Simulation** | Stop time (300 s), sample time (0.02 s / 50 Hz) |
| **Sensors** | Camera noise, gaze sample rate, head-pose noise |
| **Drowsiness** | PERCLOS threshold (0.4), blink duration threshold (0.5 s), fusion weights |
| **Distraction** | Gaze-off angle threshold (30°), head turn threshold (25°) |
| **Fatigue** | Steering entropy threshold (0.7), lane deviation threshold (0.3 m) |
| **Alerts** | 5 levels, escalation delays, cooldown time (10 s) |
| **Intervention** | LKA gain (0.5), max deceleration (2 m/s²), emergency decel (5 m/s²) |

## Generated Outputs

After simulation, the following variables are available in the MATLAB workspace:

| Variable | Description |
|----------|-------------|
| `log_DrowsinessScore` | Time-series of drowsiness score (0–1) |
| `log_DistractionScore` | Time-series of distraction score (0–1) |
| `log_FatigueScore` | Time-series of fatigue score (0–1) |
| `log_OverallRisk` | Time-series of overall risk (max of 3 scores) |
| `log_AlertLevel` | Time-series of alert level (0–4) |
| `log_VehicleSpeed` | Time-series of vehicle speed (km/h) |
| `log_LKA_Torque` | Time-series of lane-keeping assist torque (Nm) |
| `log_SpeedCommand` | Time-series of speed command output (km/h) |

## License

This project is provided for educational and research purposes.