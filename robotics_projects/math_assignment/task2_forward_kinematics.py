#!/usr/bin/env python3
"""
Task 2: Forward Kinematics for 4-Link Robot with Perpendicular Joints

This module computes the forward kinematics of a 4-link serial manipulator
where all joints are perpendicular to each other, using Denavit-Hartenberg (DH) parameters.

Robot Configuration:
- 4 revolute joints (J1, J2, J3, J4)
- All joints are perpendicular (α = ±90°)
- Link length: L = 1m (parameterized)
- Base frame at first joint

DH Parameter Convention:
- θᵢ: Joint angle (variable for revolute joints)
- dᵢ: Link offset along previous Z-axis
- aᵢ: Link length along common normal
- αᵢ: Twist angle between Z-axes

Author: Your Name
Date: November 2025
"""

import numpy as np
import math


class RobotFK:
    """
    Forward Kinematics calculator for 4-link perpendicular robot.
    
    The robot has 4 revolute joints with the following DH parameters:
    
    Joint | θᵢ        | dᵢ | aᵢ | αᵢ
    ------|-----------|----|----|------
      1   | θ1 (var)  | 0  | L  | π/2
      2   | θ2 (var)  | 0  | L  | π/2
      3   | θ3 (var)  | 0  | L  | π/2
      4   | θ4 (var)  | 0  | L  | 0
    
    Note: Perpendicular joints means consecutive Z-axes are at 90° to each other
    """
    
    def __init__(self, link_length=1.0):
        """
        Initialize robot with specified link length.
        
        Args:
            link_length (float): Length of each link in meters (default: 1.0)
        """
        self.L = link_length
        
        # DH parameters for perpendicular 4-link robot
        # Each row: [theta, d, a, alpha]
        # theta is variable (will be filled in during FK computation)
        self.dh_params = [
            [0, 0, self.L, np.pi/2],   # Joint 1
            [0, 0, self.L, np.pi/2],   # Joint 2
            [0, 0, self.L, np.pi/2],   # Joint 3
            [0, 0, self.L, 0]          # Joint 4
        ]
    
    def dh_transform(self, theta, d, a, alpha):
        """
        Compute the Denavit-Hartenberg transformation matrix.
        
        The DH transformation represents the pose of frame i relative to frame i-1.
        It combines 4 elementary transformations:
        1. Rotate about Z by θ
        2. Translate along Z by d
        3. Translate along X by a
        4. Rotate about X by α
        
        Matrix form:
        T = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)  a*cos(θ) |
            | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)  a*sin(θ) |
            |   0         sin(α)         cos(α)         d     |
            |   0           0               0           1     |
        
        Args:
            theta (float): Joint angle in radians
            d (float): Link offset
            a (float): Link length
            alpha (float): Twist angle in radians
        
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])
        
        return T
    
    def forward_kinematics(self, j1, j2, j3, j4, verbose=False):
        """
        Compute end-effector position given joint angles.
        
        Algorithm:
        1. Compute individual transformation matrices T_01, T_12, T_23, T_34
        2. Multiply to get total transformation: T_04 = T_01 × T_12 × T_23 × T_34
        3. Extract end-effector position from last column: [x, y, z] = T_04[0:3, 3]
        
        Args:
            j1, j2, j3, j4 (float): Joint angles in radians
            verbose (bool): Print intermediate transformations
        
        Returns:
            tuple: (x, y, z) end-effector position in meters
        
        Example:
            >>> robot = RobotFK(link_length=1.0)
            >>> x, y, z = robot.forward_kinematics(0, 0, 0, 0)
            >>> print(f"End-effector: ({x:.4f}, {y:.4f}, {z:.4f})")
        """
        # Update joint angles in DH parameters
        joint_angles = [j1, j2, j3, j4]
        
        # Compute individual transformation matrices
        T_01 = self.dh_transform(j1, *self.dh_params[0][1:])
        T_12 = self.dh_transform(j2, *self.dh_params[1][1:])
        T_23 = self.dh_transform(j3, *self.dh_params[2][1:])
        T_34 = self.dh_transform(j4, *self.dh_params[3][1:])
        
        if verbose:
            print("\nIndividual Transformation Matrices:")
            print("\nT_01 (Base to Joint 1):")
            print(T_01)
            print("\nT_12 (Joint 1 to Joint 2):")
            print(T_12)
            print("\nT_23 (Joint 2 to Joint 3):")
            print(T_23)
            print("\nT_34 (Joint 3 to End-Effector):")
            print(T_34)
        
        # Compute total transformation by matrix multiplication
        # T_04 = T_01 * T_12 * T_23 * T_34
        T_02 = np.matmul(T_01, T_12)
        T_03 = np.matmul(T_02, T_23)
        T_04 = np.matmul(T_03, T_34)
        
        if verbose:
            print("\nTotal Transformation T_04 (Base to End-Effector):")
            print(T_04)
        
        # Extract end-effector position from the last column
        x = T_04[0, 3]
        y = T_04[1, 3]
        z = T_04[2, 3]
        
        return (x, y, z)
    
    def get_full_transformation(self, j1, j2, j3, j4):
        """
        Get the complete 4x4 transformation matrix (includes orientation).
        
        Args:
            j1, j2, j3, j4 (float): Joint angles in radians
        
        Returns:
            np.ndarray: 4x4 transformation matrix with rotation and position
        """
        T_01 = self.dh_transform(j1, *self.dh_params[0][1:])
        T_12 = self.dh_transform(j2, *self.dh_params[1][1:])
        T_23 = self.dh_transform(j3, *self.dh_params[2][1:])
        T_34 = self.dh_transform(j4, *self.dh_params[3][1:])
        
        T_04 = T_01 @ T_12 @ T_23 @ T_34
        
        return T_04


def test_forward_kinematics():
    """
    Test suite for forward kinematics computation.
    
    Tests include:
    1. Zero configuration (all joints at 0°)
    2. Single joint movements
    3. Combined joint movements
    4. Edge cases (±90°, ±180°)
    5. Workspace limits
    """
    print("="*70)
    print("TASK 2: FORWARD KINEMATICS TESTS")
    print("="*70)
    
    # Initialize robot with 1m link length
    robot = RobotFK(link_length=1.0)
    
    # Test Case 1: Zero configuration
    print("\n[Test 1] Zero configuration (all joints at 0°):")
    j1, j2, j3, j4 = 0, 0, 0, 0
    x, y, z = robot.forward_kinematics(j1, j2, j3, j4, verbose=True)
    print(f"\nJoint angles: [{j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f}] rad")
    print(f"End-effector position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")
    print(f"Distance from origin: {np.sqrt(x**2 + y**2 + z**2):.4f}m")
    
    # Test Case 2: Single joint movement (J1 = 90°)
    print("\n" + "="*70)
    print("\n[Test 2] Joint 1 = 90°, others = 0°:")
    j1, j2, j3, j4 = np.pi/2, 0, 0, 0
    x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
    print(f"Joint angles: [{np.degrees(j1):.1f}°, {np.degrees(j2):.1f}°, {np.degrees(j3):.1f}°, {np.degrees(j4):.1f}°]")
    print(f"End-effector position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")
    
    # Test Case 3: All joints at 45°
    print("\n[Test 3] All joints at 45°:")
    j1 = j2 = j3 = j4 = np.pi/4
    x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
    print(f"Joint angles: [{np.degrees(j1):.1f}°, {np.degrees(j2):.1f}°, {np.degrees(j3):.1f}°, {np.degrees(j4):.1f}°]")
    print(f"End-effector position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")
    
    # Test Case 4: Maximum reach (all joints aligned)
    print("\n[Test 4] Maximum reach configuration:")
    j1, j2, j3, j4 = 0, 0, 0, 0
    x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
    max_reach = np.sqrt(x**2 + y**2 + z**2)
    print(f"Joint angles: all at 0°")
    print(f"Maximum theoretical reach: {4 * robot.L:.4f}m (4 links × {robot.L}m)")
    print(f"Actual end-effector distance: {max_reach:.4f}m")
    
    # Test Case 5: Workspace exploration
    print("\n[Test 5] Workspace exploration (various configurations):")
    print(f"\n{'J1':>6}° {'J2':>6}° {'J3':>6}° {'J4':>6}° | {'X':>8} {'Y':>8} {'Z':>8} | Distance")
    print("-" * 70)
    
    test_configs = [
        (0, 0, 0, 0),
        (np.pi/2, 0, 0, 0),
        (0, np.pi/2, 0, 0),
        (0, 0, np.pi/2, 0),
        (0, 0, 0, np.pi/2),
        (np.pi/4, np.pi/4, np.pi/4, np.pi/4),
        (np.pi/2, np.pi/2, np.pi/2, np.pi/2),
        (-np.pi/4, np.pi/4, -np.pi/4, np.pi/4),
    ]
    
    for config in test_configs:
        j1, j2, j3, j4 = config
        x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
        dist = np.sqrt(x**2 + y**2 + z**2)
        print(f"{np.degrees(j1):>6.1f}° {np.degrees(j2):>6.1f}° {np.degrees(j3):>6.1f}° {np.degrees(j4):>6.1f}° | "
              f"{x:>7.4f}m {y:>7.4f}m {z:>7.4f}m | {dist:>7.4f}m")
    
    # Test Case 6: Singularity check
    print("\n[Test 6] Checking for singularities:")
    j1, j2, j3, j4 = 0, np.pi, 0, 0
    x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
    print(f"Singular configuration: J2 = 180° (folded back)")
    print(f"End-effector position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")
    
    print("\n" + "="*70)
    print("✓ All forward kinematics tests completed successfully!")
    print("="*70)


def interactive_fk():
    """
    Interactive mode for testing custom joint angles.
    """
    print("\n" + "="*70)
    print("Interactive Forward Kinematics Calculator")
    print("="*70)
    
    robot = RobotFK(link_length=1.0)
    
    print("\nEnter joint angles (or press Ctrl+C to exit):")
    print("Example: 0 0 0 0 (all angles in degrees)")
    
    try:
        while True:
            user_input = input("\nJoint angles (J1 J2 J3 J4 in degrees): ")
            angles_deg = list(map(float, user_input.split()))
            
            if len(angles_deg) != 4:
                print("❌ Please enter exactly 4 angles!")
                continue
            
            # Convert to radians
            j1, j2, j3, j4 = [np.radians(a) for a in angles_deg]
            
            # Compute FK
            x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
            
            print(f"\n✓ End-effector position:")
            print(f"  x = {x:.6f} m")
            print(f"  y = {y:.6f} m")
            print(f"  z = {z:.6f} m")
            print(f"  Distance from origin: {np.sqrt(x**2 + y**2 + z**2):.6f} m")
            
    except KeyboardInterrupt:
        print("\n\nExiting interactive mode. Goodbye!")


if __name__ == "__main__":
    # Run test suite
    test_forward_kinematics()
    
    # Optional: Run interactive mode
    # Uncomment the line below to enable interactive testing
    # interactive_fk()
    
    print("\n💡 Tip: Import this module to use RobotFK class in your own code!")
    print("   Example: from task2_forward_kinematics import RobotFK")

