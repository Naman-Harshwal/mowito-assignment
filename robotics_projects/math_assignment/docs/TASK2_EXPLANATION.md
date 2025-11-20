# Task 2: Forward Kinematics for 4-Link Robot

## Robot Configuration

- **Type**: Serial manipulator with 4 revolute joints
- **Links**: Each length L = 1m
- **Constraint**: All joints perpendicular (α = ±90°)

## DH Parameters

| Joint | θᵢ (var) | dᵢ | aᵢ | αᵢ   |
|-------|----------|----|----|------|
| 1     | θ₁       | 0  | L  | π/2  |
| 2     | θ₂       | 0  | L  | π/2  |
| 3     | θ₃       | 0  | L  | π/2  |
| 4     | θ₄       | 0  | L  | 0    |

## DH Transformation Matrix

T = | cos(θ) -sin(θ)cos(α) sin(θ)sin(α) a·cos(θ) |
| sin(θ) cos(θ)cos(α) -cos(θ)sin(α) a·sin(θ) |
| 0 sin(α) cos(α) d |
| 0 0 0 1 |

text

## Forward Kinematics Algorithm

    Build T₀₁, T₁₂, T₂₃, T₃₄ using DH parameters

    Multiply: T₀₄ = T₀₁ × T₁₂ × T₂₃ × T₃₄

    Extract position: [x, y, z] = T₀₄[0:3, 3]

text

## Example: Zero Configuration

Input: θ₁ = θ₂ = θ₃ = θ₄ = 0

Result: End-effector at (4L, 0, 0) = (4, 0, 0) meters

## Key Interview Points

1. **DH parameters?** (θ, d, a, α) define frame transformation
2. **Perpendicular joints?** Simplifies math, α = ±90°
3. **Matrix order?** Right-to-left: T₀₄ = T₀₁·T₁₂·T₂₃·T₃₄
4. **Position extraction?** Last column of transformation matrix
5. **Singularities?** Fully extended or folded configurations
