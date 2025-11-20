#!/bin/bash

echo "=================================================="
echo "  Robotics Math Assignment - Setup Script"
echo "=================================================="

# Check Python version
echo -e "\n[1/4] Checking Python version..."
python3 --version

# Install dependencies
echo -e "\n[2/4] Installing dependencies..."
python3 -m pip install --user -r requirements.txt

# Verify installations
echo -e "\n[3/4] Verifying installations..."
python3 -c "import numpy; print(f'✓ NumPy {numpy.__version__}')"
python3 -c "import matplotlib; print(f'✓ Matplotlib {matplotlib.__version__}')" 2>/dev/null || echo "⚠️  Matplotlib optional"

# Run quick test
echo -e "\n[4/4] Running quick verification test..."
python3 -c "
from task1_euler_quaternion import euler_to_quaternion
qw, qx, qy, qz = euler_to_quaternion(0, 0, 0)
print(f'✓ Task 1: Zero rotation quaternion = [{qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f}]')
"

python3 -c "
from task2_forward_kinematics import RobotFK
robot = RobotFK()
x, y, z = robot.forward_kinematics(0, 0, 0, 0)
print(f'✓ Task 2: Zero config position = ({x:.4f}, {y:.4f}, {z:.4f})')
"

echo -e "\n=================================================="
echo "  ✓ Setup complete! You can now run:"
echo "    python3 task1_euler_quaternion.py"
echo "    python3 task2_forward_kinematics.py"
echo "=================================================="
