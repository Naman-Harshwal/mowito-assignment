# Task 1: Euler Angles ↔ Quaternion Conversion

## Mathematical Foundation

### Euler Angles
- **Roll (φ)**: Rotation around X-axis
- **Pitch (θ)**: Rotation around Y-axis  
- **Yaw (ψ)**: Rotation around Z-axis

### Quaternions
q = qw + qx·i + qy·j + qz·k

**Constraint**: qw² + qx² + qy² + qz² = 1

## Conversion Formulas

### Euler → Quaternion

cy = cos(yaw/2), sy = sin(yaw/2)
cp = cos(pitch/2), sp = sin(pitch/2)
cr = cos(roll/2), sr = sin(roll/2)

qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy

text

### Quaternion → Euler

roll = atan2(2*(qwqx + qyqz), 1 - 2*(qx² + qy²))
pitch = asin(2*(qwqy - qzqx))
yaw = atan2(2*(qwqz + qxqy), 1 - 2*(qy² + qz²))

text

## Gimbal Lock

**Occurs when**: pitch ≈ ±90°

**Effect**: Z-axis and X-axis align, losing one degree of freedom

**Detection**: |sin(pitch)| > 0.99999

**Handling**:
1. Set pitch = ±90° exactly
2. Set roll = 0 (arbitrary)
3. Compute effective yaw = 2·atan2(qx, qw)

## Key Interview Points

1. **Why quaternions?** Avoid gimbal lock, smooth interpolation
2. **Gimbal lock?** Loss of DOF when pitch = ±90°
3. **Normalization?** Maintain unit quaternion constraint
4. **Half angles?** Natural representation in 4D space
