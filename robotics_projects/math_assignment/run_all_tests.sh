#!/bin/bash

echo "=================================================="
echo "  Running All Tests for Math Assignment"
echo "=================================================="

echo -e "\n[Test 1] Task 1 - Euler-Quaternion Conversion"
echo "----------------------------------------------"
python3 task1_euler_quaternion.py

echo -e "\n\n[Test 2] Task 2 - Forward Kinematics"
echo "----------------------------------------------"
python3 task2_forward_kinematics.py

echo -e "\n\n=================================================="
echo "  ✓ All tests completed!"
echo "=================================================="

