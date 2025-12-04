#!/bin/bash

# Script to run TransFuser agent locally in CARLA with visualization enabled
# Usage: ./run_local_visualization.sh <carla_root> <working_directory> [model_config_path]

# Set default paths (modify these to match your setup)
export CARLA_ROOT=${1:-/home/marzukkp/Transfuser/transfuser/carla}
export WORK_DIR=${2:-/home/marzukkp/Transfuser/transfuser}
export MODEL_CONFIG=${3:-${WORK_DIR}/model_ckpt/transfuser/models_2022/transfuser}

# Check if CARLA_ROOT exists
if [ ! -d "$CARLA_ROOT" ]; then
    echo "Error: CARLA_ROOT not found at $CARLA_ROOT"
    echo "Please provide the correct path to your CARLA installation"
    exit 1
fi

# Check if model config exists
if [ ! -d "$MODEL_CONFIG" ]; then
    echo "Warning: Model config directory not found at $MODEL_CONFIG"
    echo "Please make sure you have downloaded model weights or trained a model"
    echo "You can download pretrained models from: https://s3.eu-central-1.amazonaws.com/avg-projects/transfuser/models_2022.zip"
fi

# Set up environment
export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

# Configuration for local visualization
export SCENARIOS=${WORK_DIR}/leaderboard/data/longest6/eval_scenarios.json
export ROUTES=${WORK_DIR}/leaderboard/data/longest6/longest6.xml
export REPETITIONS=1
export CHALLENGE_TRACK_CODENAME=SENSORS
export CHECKPOINT_ENDPOINT=${WORK_DIR}/results/local_test.json
export TEAM_AGENT=${WORK_DIR}/team_code_transfuser/submission_agent.py
export TEAM_CONFIG=${MODEL_CONFIG}

# Enable visualization and debugging
export DEBUG_CHALLENGE=1  # Enable debug mode (shows more info)
export SAVE_PATH=${WORK_DIR}/results/visualizations  # Save debug images/videos
export RESUME=0
export DATAGEN=0

# Create visualization directory
mkdir -p ${SAVE_PATH}

echo "=========================================="
echo "Running TransFuser Agent Locally"
echo "=========================================="
echo "CARLA Root: $CARLA_ROOT"
echo "Work Directory: $WORK_DIR"
echo "Model Config: $MODEL_CONFIG"
echo "Visualizations will be saved to: $SAVE_PATH"
echo ""
echo "Make sure CARLA is running:"
echo "  ./CarlaUE4.sh --world-port=2000 -opengl"
echo ""
echo "Press Ctrl+C to stop the evaluation"
echo "=========================================="
echo ""

# Run the evaluation
clean_up_and_exit() {
    if [[ -n "${child_pid:-}" ]] && ps -p "${child_pid}" > /dev/null 2>&1; then
        echo ""
        echo "Stopping evaluation (signal caught)..."
        kill -TERM "${child_pid}" 2>/dev/null
        wait "${child_pid}" 2>/dev/null
    fi
    exit 0
}

trap clean_up_and_exit SIGINT SIGTERM SIGTSTP

/home/marzukkp/anaconda3/envs/tfuse/bin/python3.7 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator_local.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--resume=${RESUME} &
child_pid=$!

wait "${child_pid}"
trap - SIGINT SIGTERM SIGTSTP

echo ""
echo "Evaluation complete! Check results at:"
echo "  Results: ${CHECKPOINT_ENDPOINT}"
echo "  Visualizations: ${SAVE_PATH}"

