# How to Run TransFuser Agent - Step by Step Guide

## Prerequisites Checklist

Before running, make sure you have:

- [ ] CARLA 0.9.10.1 installed (I see it's in `carla/` directory)
- [ ] Python environment with dependencies installed
- [ ] Model weights downloaded (or trained model)
- [ ] GPU with CUDA (for neural network inference)
- [ ] CasADi installed (optional, falls back to scipy if not available)

---

## Step 1: Install Dependencies

### Install CasADi (for MPC)
```bash
cd /home/marzukkp/Transfuser/transfuser
pip install casadi
```

Or if using conda:
```bash
conda activate tfuse  # or your environment name
pip install casadi
```

### Verify other dependencies
The main dependencies should already be in `requirements.txt`. If needed:
```bash
pip install -r team_code_transfuser/requirements.txt
```

---

## Step 2: Download Model Weights

You need trained model weights to run the agent. You have two options:

### Option A: Download Pretrained Models (Easiest)

```bash
cd /home/marzukkp/Transfuser/transfuser
mkdir -p model_ckpt/transfuser
cd model_ckpt
wget https://s3.eu-central-1.amazonaws.com/avg-projects/transfuser/models_2022.zip
unzip models_2022.zip -d transfuser/
rm models_2022.zip
cd ..
```

This will download pretrained models to `model_ckpt/transfuser/`

### Option B: Use Your Own Trained Model

If you've trained a model, place the `.pth` files and `args.txt` in:
```
model_ckpt/transfuser/
  ├── model_XX.pth
  └── args.txt
```

---

## Step 3: Start CARLA Simulator

**Open Terminal 1:**

```bash
cd /home/marzukkp/Transfuser/transfuser/carla
./CarlaUE4.sh --world-port=2000 -opengl
```

Wait for CARLA to fully load. You should see the CARLA window open.

**Note:** If you don't have a display, use headless mode:
```bash
SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh --world-port=2000 -opengl
```

---

## Step 4: Run the Agent

**Open Terminal 2** (keep Terminal 1 running):

```bash
cd /home/marzukkp/Transfuser/transfuser

# Activate your conda environment if using one
# conda activate tfuse

# Run the visualization script
./run_local_visualization.sh
```

Or with custom paths:
```bash
./run_local_visualization.sh <carla_root> <working_directory> [model_config_path]
```

Example:
```bash
./run_local_visualization.sh \
  /home/marzukkp/Transfuser/transfuser/carla \
  /home/marzukkp/Transfuser/transfuser \
  /home/marzukkp/Transfuser/transfuser/model_ckpt/transfuser
```

---

## Step 5: Watch It Drive!

Once running, you should see:

1. **CARLA Window**: Vehicle driving autonomously through the town
2. **Terminal Output**: 
   - Model loading messages
   - Route information
   - Stuck detection warnings (if vehicle gets stuck)
   - MPC optimization status
   - Safety warnings

3. **Results Saved**:
   - `results/local_test.json` - Evaluation results
   - `results/visualizations/` - Debug images (if SAVE_PATH is set)

---

## Manual Run (Alternative Method)

If the script doesn't work, you can run manually:

### Terminal 1: Start CARLA
```bash
cd /home/marzukkp/Transfuser/transfuser/carla
./CarlaUE4.sh --world-port=2000 -opengl
```

### Terminal 2: Set Environment and Run
```bash
cd /home/marzukkp/Transfuser/transfuser

# Set environment variables
export CARLA_ROOT=/home/marzukkp/Transfuser/transfuser/carla
export WORK_DIR=/home/marzukkp/Transfuser/transfuser
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

# Set configuration
export SCENARIOS=${WORK_DIR}/leaderboard/data/longest6/eval_scenarios.json
export ROUTES=${WORK_DIR}/leaderboard/data/longest6/longest6.xml
export TEAM_AGENT=${WORK_DIR}/team_code_transfuser/submission_agent.py
export TEAM_CONFIG=${WORK_DIR}/model_ckpt/transfuser
export DEBUG_CHALLENGE=1
export SAVE_PATH=${WORK_DIR}/results/visualizations
mkdir -p ${SAVE_PATH}

# Run
python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator_local.py \
  --scenarios=${SCENARIOS} \
  --routes=${ROUTES} \
  --repetitions=1 \
  --track=SENSORS \
  --checkpoint=${WORK_DIR}/results/local_test.json \
  --agent=${TEAM_AGENT} \
  --agent-config=${TEAM_CONFIG} \
  --debug=1 \
  --resume=1
```

---

## Troubleshooting

### Problem: "CARLA_ROOT not found"
**Solution:** Update the path in `run_local_visualization.sh` line 7, or pass it as argument:
```bash
./run_local_visualization.sh /path/to/carla /path/to/transfuser
```

### Problem: "Model config directory not found"
**Solution:** Download model weights (Step 2) or check the path:
```bash
ls model_ckpt/transfuser/
# Should show: model_*.pth and args.txt
```

### Problem: "Connection refused" or "Cannot connect to CARLA"
**Solution:** 
1. Make sure CARLA is running in Terminal 1
2. Check CARLA is on port 2000
3. Wait for CARLA to fully load before running the agent

### Problem: "CUDA out of memory"
**Solution:**
- Close other GPU processes
- Use a smaller model or reduce batch size
- The agent should work with ~4GB GPU memory

### Problem: "ModuleNotFoundError: No module named 'casadi'"
**Solution:**
```bash
pip install casadi
```
Or it will automatically fall back to scipy.optimize

### Problem: "ModuleNotFoundError: No module named 'carla'"
**Solution:**
```bash
export PYTHONPATH=$PYTHONPATH:/home/marzukkp/Transfuser/transfuser/carla/PythonAPI
export PYTHONPATH=$PYTHONPATH:/home/marzukkp/Transfuser/transfuser/carla/PythonAPI/carla
```

### Problem: Model loads but vehicle doesn't move
**Solution:**
- Check if MPC is being called (look for optimization messages)
- Check if waypoints are being generated
- Verify CARLA is receiving control commands
- Check console for error messages

---

## Quick Test Commands

### Check if CARLA is accessible:
```bash
python3 -c "import carla; print('CARLA imported successfully')"
```

### Check if model weights exist:
```bash
ls -lh model_ckpt/transfuser/*.pth
```

### Check if CasADi is installed:
```bash
python3 -c "import casadi; print('CasADi version:', casadi.__version__)"
```

### Test MPC import:
```bash
cd team_code_transfuser
python3 -c "from model import LidarCenterNet; print('Model imports OK')"
```

---

## Expected Output

When running successfully, you should see:

```
==========================================
Running TransFuser Agent Locally
==========================================
CARLA Root: /home/marzukkp/Transfuser/transfuser/carla
Work Directory: /home/marzukkp/Transfuser/transfuser
Model Config: /home/marzukkp/Transfuser/transfuser/model_ckpt/transfuser
Visualizations will be saved to: /home/marzukkp/Transfuser/transfuser/results/visualizations

Make sure CARLA is running:
  ./CarlaUE4.sh --world-port=2000 -opengl

Press Ctrl+C to stop the evaluation
==========================================

Loading model: model_ckpt/transfuser/model_XX.pth
[Model loading messages...]
[Route information...]
[Driving starts...]
```

---

## Stopping the Agent

Press `Ctrl+C` in Terminal 2 to stop the agent. CARLA will continue running in Terminal 1.

To stop CARLA, press `Ctrl+C` in Terminal 1 or close the CARLA window.

---

## Next Steps

Once it's running:
1. Watch the vehicle drive in CARLA
2. Check console for MPC optimization messages
3. Monitor for stuck detection and safety warnings
4. Review results in `results/local_test.json`
5. Check visualizations in `results/visualizations/` (if enabled)

Enjoy watching your MPC controller in action! 🚗

