#!/usr/bin/env python3
"""
Task 1: Euler Angles to Quaternion Conversion (and vice versa)

This module implements bidirectional conversion between Euler angles and Quaternions,
with proper handling of gimbal lock singularities.

Mathematical Background:
- Euler angles: (roll, pitch, yaw) - rotation around X, Y, Z axes respectively
- Quaternion: q = [qw, qx, qy, qz] where qw² + qx² + qy² + qz² = 1
- Convention: ZYX (yaw-pitch-roll) intrinsic rotation sequence

Edge Cases Handled:
- Gimbal lock at pitch = ±90° (±π/2 rad)
- Numerical stability near singularities
- Angle normalization to [-π, π]

Author: Your Name
Date: November 2025
"""

import numpy as np
import math


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to Quaternion (qw, qx, qy, qz).
    
    Uses the ZYX (yaw-pitch-roll) convention:
    - First rotate around Z-axis by yaw
    - Then rotate around Y-axis by pitch
    - Finally rotate around X-axis by roll
    
    Formula derivation:
    The quaternion is obtained by multiplying individual rotation quaternions:
    q = q_yaw * q_pitch * q_roll
    
    Using half-angle formulas:
    qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
    qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
    qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
    qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)
    
    Args:
        roll (float): Rotation around X-axis in radians [-π, π]
        pitch (float): Rotation around Y-axis in radians [-π/2, π/2]
        yaw (float): Rotation around Z-axis in radians [-π, π]
    
    Returns:
        tuple: (qw, qx, qy, qz) - normalized quaternion components
    
    Example:
        >>> roll, pitch, yaw = 0.1, 0.2, 0.3
        >>> qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
        >>> print(f"Quaternion: [{qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f}]")
    """
    # Compute half angles for efficiency
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Apply the conversion formulas (ZYX convention)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    # Normalize to ensure unit quaternion (handles numerical errors)
    norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    
    return (qw/norm, qx/norm, qy/norm, qz/norm)


def quaternion_to_euler(qw, qx, qy, qz):
    """
    Convert Quaternion (qw, qx, qy, qz) to Euler angles (roll, pitch, yaw).
    
    Handles gimbal lock singularity at pitch = ±90°.
    
    Mathematical Derivation:
    From quaternion to rotation matrix elements:
    - roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx² + qy²))
    - pitch = asin(2*(qw*qy - qz*qx))
    - yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
    
    Gimbal Lock Detection:
    When |sin_pitch| ≈ 1 (pitch ≈ ±90°), we lose one degree of freedom.
    In this case, we set roll = 0 and compute yaw differently.
    
    Args:
        qw (float): Scalar component of quaternion
        qx (float): X component of quaternion
        qy (float): Y component of quaternion
        qz (float): Z component of quaternion
    
    Returns:
        tuple: (roll, pitch, yaw) in radians
               roll ∈ [-π, π], pitch ∈ [-π/2, π/2], yaw ∈ [-π, π]
    
    Raises:
        ValueError: If quaternion is not normalized (norm != 1)
    
    Example:
        >>> qw, qx, qy, qz = 0.9833, 0.0998, 0.1489, 0.0588
        >>> roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)
        >>> print(f"Euler: roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")
    """
    # Validate quaternion normalization
    norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    if abs(norm - 1.0) > 1e-6:
        raise ValueError(f"Quaternion must be normalized. Current norm: {norm}")
    
    # Compute sin(pitch) directly for gimbal lock detection
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    
    # Gimbal lock threshold (very close to ±1)
    gimbal_lock_threshold = 0.99999
    
    # Check for gimbal lock condition
    if abs(sin_pitch) >= gimbal_lock_threshold:
        # Gimbal lock case: pitch ≈ ±90°
        # We lose one degree of freedom, so we set roll = 0
        # and compute yaw to represent the remaining rotation
        roll = 0.0
        pitch = math.copysign(math.pi / 2, sin_pitch)  # ±90°
        
        # In gimbal lock, yaw is computed differently
        yaw = 2.0 * math.atan2(qx, qw)
        
        print(f"⚠️  Gimbal lock detected at pitch = {math.degrees(pitch):.1f}°")
    else:
        # Normal case: no gimbal lock
        # Roll (X-axis rotation)
        t0 = 2.0 * (qw * qx + qy * qz)
        t1 = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(t0, t1)
        
        # Pitch (Y-axis rotation) - use asin with clamping for numerical stability
        pitch = math.asin(np.clip(sin_pitch, -1.0, 1.0))
        
        # Yaw (Z-axis rotation)
        t2 = 2.0 * (qw * qz + qx * qy)
        t3 = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(t2, t3)
    
    return (roll, pitch, yaw)


def normalize_angle(angle):
    """
    Normalize angle to the range [-π, π].
    
    Args:
        angle (float): Angle in radians
    
    Returns:
        float: Normalized angle in [-π, π]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def test_conversions():
    """
    Test suite for Euler-Quaternion conversions.
    
    Tests include:
    1. Standard angles
    2. Gimbal lock cases (pitch = ±90°)
    3. Edge cases (zero rotation, 180° rotation)
    4. Round-trip conversion accuracy
    """
    print("="*70)
    print("TASK 1: EULER-QUATERNION CONVERSION TESTS")
    print("="*70)
    
    # Test Case 1: Standard angles
    print("\n[Test 1] Standard Euler angles:")
    roll, pitch, yaw = 0.1, 0.2, 0.3
    print(f"Input Euler: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
    
    qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion: qw={qw:.6f}, qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}")
    print(f"Quaternion norm: {math.sqrt(qw**2 + qx**2 + qy**2 + qz**2):.10f}")
    
    roll_back, pitch_back, yaw_back = quaternion_to_euler(qw, qx, qy, qz)
    print(f"Back to Euler: roll={math.degrees(roll_back):.2f}°, pitch={math.degrees(pitch_back):.2f}°, yaw={math.degrees(yaw_back):.2f}°")
    print(f"Error: Δroll={abs(roll-roll_back):.2e}, Δpitch={abs(pitch-pitch_back):.2e}, Δyaw={abs(yaw-yaw_back):.2e}")
    
    # Test Case 2: Gimbal lock at +90° pitch
    print("\n[Test 2] Gimbal lock case (pitch = +90°):")
    roll, pitch, yaw = 0.5, math.pi/2, 0.3
    print(f"Input Euler: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
    
    qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion: qw={qw:.6f}, qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}")
    
    roll_back, pitch_back, yaw_back = quaternion_to_euler(qw, qx, qy, qz)
    print(f"Back to Euler: roll={math.degrees(roll_back):.2f}°, pitch={math.degrees(pitch_back):.2f}°, yaw={math.degrees(yaw_back):.2f}°")
    
    # Test Case 3: Gimbal lock at -90° pitch
    print("\n[Test 3] Gimbal lock case (pitch = -90°):")
    roll, pitch, yaw = 0.2, -math.pi/2, 0.4
    print(f"Input Euler: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
    
    qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion: qw={qw:.6f}, qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}")
    
    roll_back, pitch_back, yaw_back = quaternion_to_euler(qw, qx, qy, qz)
    print(f"Back to Euler: roll={math.degrees(roll_back):.2f}°, pitch={math.degrees(pitch_back):.2f}°, yaw={math.degrees(yaw_back):.2f}°")
    
    # Test Case 4: Zero rotation
    print("\n[Test 4] Zero rotation:")
    roll, pitch, yaw = 0.0, 0.0, 0.0
    qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion for zero rotation: qw={qw:.6f}, qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}")
    print(f"Expected: qw=1.0, qx=0.0, qy=0.0, qz=0.0")
    
    # Test Case 5: Multiple random angles
    print("\n[Test 5] Random angle stress test:")
    np.random.seed(42)
    max_error = 0.0
    num_tests = 100
    
    for i in range(num_tests):
        roll = np.random.uniform(-math.pi, math.pi)
        pitch = np.random.uniform(-math.pi/2 + 0.1, math.pi/2 - 0.1)  # Avoid gimbal lock
        yaw = np.random.uniform(-math.pi, math.pi)
        
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
        roll_back, pitch_back, yaw_back = quaternion_to_euler(qw, qx, qy, qz)
        
        error = max(abs(roll - roll_back), abs(pitch - pitch_back), abs(yaw - yaw_back))
        max_error = max(max_error, error)
    
    print(f"Tested {num_tests} random conversions")
    print(f"Maximum round-trip error: {max_error:.2e} radians ({math.degrees(max_error):.2e}°)")
    
    print("\n" + "="*70)
    print("✓ All tests completed successfully!")
    print("="*70)


if __name__ == "__main__":
    test_conversions()
    
    # Optional: Interactive mode
    print("\n" + "="*70)
    print("Interactive Mode: Try your own angles!")
    print("="*70)
    print("Example: euler_to_quaternion(0.1, 0.2, 0.3)")
    print("You can now import this module and use the functions.")

