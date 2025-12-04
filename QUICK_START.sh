#!/bin/bash
# Quick Start Script for TransFuser with MPC

echo "=========================================="
echo "TransFuser Quick Start"
echo "=========================================="

# Step 1: Install CasADi
echo ""
echo "Step 1: Installing CasADi for MPC..."
pip install casadi
if [ $? -eq 0 ]; then
    echo "✓ CasADi installed successfully"
else
    echo "⚠ CasADi installation failed, will use scipy fallback"
fi

# Step 2: Download model weights
echo ""
echo "Step 2: Downloading pretrained model weights..."
mkdir -p model_ckpt/transfuser
cd model_ckpt

if [ ! -f "models_2022.zip" ]; then
    echo "Downloading models (this may take a while)..."
    wget https://s3.eu-central-1.amazonaws.com/avg-projects/transfuser/models_2022.zip
fi

if [ -f "models_2022.zip" ]; then
    echo "Extracting models..."
    unzip -q -o models_2022.zip -d transfuser/
    echo "✓ Models downloaded and extracted"
    # Don't remove zip in case user wants to keep it
else
    echo "⚠ Model download failed. Please download manually:"
    echo "  wget https://s3.eu-central-1.amazonaws.com/avg-projects/transfuser/models_2022.zip"
    echo "  unzip models_2022.zip -d model_ckpt/transfuser/"
fi

cd ..

# Step 3: Check CARLA
echo ""
echo "Step 3: Checking CARLA installation..."
if [ -f "carla/CarlaUE4.sh" ]; then
    echo "✓ CARLA found at: $(pwd)/carla"
else
    echo "⚠ CARLA not found. Please ensure CARLA 0.9.10.1 is installed"
fi

# Step 4: Instructions
echo ""
echo "=========================================="
echo "Setup Complete! Next Steps:"
echo "=========================================="
echo ""
echo "1. Start CARLA (in a separate terminal):"
echo "   cd $(pwd)/carla"
echo "   ./CarlaUE4.sh --world-port=2000 -opengl"
echo ""
echo "2. Run the agent (in this terminal):"
echo "   cd $(pwd)"
echo "   ./run_local_visualization.sh"
echo ""
echo "Or run manually:"
echo "   python3 leaderboard/leaderboard/leaderboard_evaluator_local.py \\"
echo "     --scenarios=leaderboard/data/longest6/eval_scenarios.json \\"
echo "     --routes=leaderboard/data/longest6/longest6.xml \\"
echo "     --track=SENSORS \\"
echo "     --checkpoint=results/local_test.json \\"
echo "     --agent=team_code_transfuser/submission_agent.py \\"
echo "     --agent-config=model_ckpt/transfuser \\"
echo "     --debug=1 \\"
echo "     --resume=1"
echo ""
echo "=========================================="

