# TransFuser-MPC

A fork of [TransFuser (PAMI 2023)](https://github.com/autonomousvision/transfuser) that replaces the original PID-based lateral/longitudinal controller with a **Model Predictive Control (MPC)** formulation solved via IPOPT. The goal is to evaluate whether an optimization-based controller can outperform reactive PID control when paired with TransFuser's transformer-based perception backbone in the CARLA simulator.

## What Changed From Upstream

The core modification is in the low-level control loop:

- **Converted PID → MPC**: The waypoint-tracking controller was rebuilt using a nonlinear MPC formulation. IPOPT (via the `casadi` interface) is used as the solver, with the Coin-HSL linear solver backend for performance.
- **MPC Dynamics**: A kinematic bicycle model is used as the prediction model inside the MPC horizon.
- **Tuned Parameters**: Multiple iterations of MPC parameter tuning were committed (`tuned MPC parameters` commits), followed by a clean merge via PR #1.
- **Improved Plotting**: Added better trajectory visualization for debugging rollouts (`feat: good plotting`).

## Architecture Overview

```
CARLA Simulator
      │
      ▼
TransFuser Backbone (CNN + Transformer sensor fusion)
  - RGB front camera
  - LiDAR BEV
      │
      ▼
Waypoint Predictions
      │
      ▼
MPC Controller (replaces original PID)
  - Kinematic bicycle model
  - IPOPT solver via CasADi
  - Horizon: configurable N steps
      │
      ▼
Throttle / Steer / Brake Commands → CARLA
```

## Dependencies

- CARLA 0.9.10.1
- Python 3.8+
- PyTorch
- CasADi + IPOPT (with Coin-HSL license)
- conda environment (see `environment.yml`)

## Setup

```bash
# Clone and setup CARLA
git clone <this-repo>
cd TransFuser-MPC
chmod +x setup_carla.sh && ./setup_carla.sh

# Build conda environment
conda env create -f environment.yml
conda activate transfuser

# Download pretrained weights and data
chmod +x download_data.sh && ./download_data.sh
```

> Quick start script available: `bash QUICK_START.sh`

## Running Evaluation

```bash
# Start CARLA server first
./CarlaUE4.sh -opengl

# Run the agent with MPC controller
python leaderboard/scripts/run_evaluation.py \
  --agent=team_code/transfuser_agent.py \
  --routes=leaderboard/data/evaluation_routes/routes_lav_valid.xml \
  --scenarios=leaderboard/data/scenarios/eval_scenarios.json
```

Results are parsed with `tools/result_parser.py`.

## Documentation

| File | Description |
|---|---|
| `README.md` | This file |
| `MPC_DYNAMICS_EXPLANATION.md` | Derivation of the bicycle model and MPC formulation |
| `CODE_WALKTHROUGH.md` | Guide to navigating the codebase |
| `HOW_TO_RUN.md` | Detailed run instructions |

## Git History Summary

| Commit | Description |
|---|---|
| `2df206c` | Initial PID → MPC conversion |
| `a86f3aa`, `1325d4c` | MPC parameter tuning iterations |
| `b074314` | Final tuned parameters |
| `69620df` | Improved rollout plotting |
| `95d9b14` | Merge PR #1 (final_rollout branch) |

## References

- [TransFuser: Imitation with Transformer-Based Sensor Fusion (PAMI 2023)](https://arxiv.org/abs/2205.15997)
- [CasADi for nonlinear optimization](https://web.casadi.org/)
- [IPOPT solver](https://coin-or.github.io/Ipopt/)