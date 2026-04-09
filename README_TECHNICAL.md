## IKSolver Technical Reference

This document describes the current architecture, solver behavior, and extension points for maintainers.

## 1. Architecture Overview

Core modules:

- Math primitives
	- `Vector2`, `Vector3`
	- `MathUtils` constants and helpers
- Kinematic chains
	- `IKChain2D` with `Bone2D`
	- `IKChain3D` with `Joint3D`
- Solvers
	- 2D: `CCDSolver`, `FABRIKSolver`, `JacobianSolver`
	- 3D: `CCDSolver3D`, `FABRIKSolver3D`
- Constraints
	- `JointConstraint2D`, `JointConstraint3D`

Directory mapping:

- Public API: `include/iksolver/`
- Implementations: `src/`
- Validation: `tests/test_solver.cpp`

## 2. Solver API Contract

### 2D Result-Enum Contract

2D solvers return explicit enums (not booleans):

- CCD: `CCDResult`
- FABRIK: `FABRIKResult`
- Jacobian: `JacobianResult`

Valid outcomes are:

- success
- invalid input
- unreachable target
- max-iteration exhaustion

This contract avoids ambiguous implicit bool conversion and should be preserved.

## 3. IKChain2D Notes

`IKChain2D` stores:

- Joint world position (`Bone2D::position`)
- Segment length to next joint (`Bone2D::length`)
- Joint absolute angle (`Bone2D::angle`)

Important behavior:

- Constructor from positions computes lengths automatically.
- `setJointAngle` applies optional joint constraints and calls `updateForwardKinematics()`.
- `getDegreesOfFreedom()` is `size - 1` (end effector has no outgoing segment).

## 4. IKChain3D Notes

Recent fixes and behavior:

- Constructor now assigns last joint length `0.0`.
- `setJointAngle` ignores end-effector index.
- `updateForwardKinematics()` now uses axis-angle rotation (Rodrigues formula) so joint angles affect chain geometry.

Current simplification:

- FK uses a lightweight orientation propagation model; it is suitable for demos/prototyping, not full rigid-body robotics simulation.

## 5. 2D Solver Details

### CCD (`src/CCDSolver.cpp`)

- Validates chain and parameters, returns `CCD_INVALID_INPUT` if invalid.
- Returns `CCD_UNREACHABLE_TARGET` when target distance exceeds total chain length.
- For unreachable targets, chain is stretched toward target deterministically.
- Iteratively rotates joints from end to base.

### FABRIK (`src/FABRIKSolver.cpp`)

- Returns `FABRIK_INVALID_INPUT` for invalid chain/settings.
- Uses standard backward and forward passes.
- Preserves segment lengths through directional normalization.

### Jacobian (`src/JacobianSolver.cpp`)

- Uses damped least squares pseudo-inverse for 2xN Jacobian.
- `dampedPseudoInverse()` computes:
	- $J^T (J J^T + \lambda^2 I)^{-1}$
- Returns `JACOBIAN_INVALID_INPUT` for invalid solver inputs.

## 6. 3D Solver Details

### CCD3D (`src/Solvers3D.cpp`)

- Computes a rotation axis from cross product and angle from clamped dot product.
- Applies axis and angle to each joint and updates FK.
- Constraint handling can clamp per-joint angle updates.

### FABRIK3D (`src/Solvers3D.cpp`)

- Uses position-only backward/forward passes.
- Updated to write directly to joint positions without modifying stored segment lengths.

## 7. Numeric Stability

Implemented protections:

- Clamp before `acos`
- Vector normalization fallback to zero vector
- `Vector3::angleTo` and `Vector3::projectOnto` guard against near-zero denominators

Recommended future additions:

- Unit tests for NaN/Inf propagation
- Optional hard assertions in debug builds for invalid chain states

## 8. Build System

`Makefile` now provides:

- `all`, `lib`, `demos`, `test`, `run-tests`, `clean`
- OS-aware executable suffixes
- Windows-compatible clean fallback

## 9. Test Coverage Notes

`tests/test_solver.cpp` now checks:

- vector and math primitives
- chain construction/validity
- solver result enums and edge cases
- unreachable and invalid-input behavior
- basic 3D solver smoke paths

Gaps still worth adding:

- deterministic regression fixtures for 3D convergence quality
- randomized stress tests for constraint-heavy chains
- benchmark-style iteration/performance assertions
