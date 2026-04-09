# IKSolver

C++17 inverse kinematics library for 2D and 3D chains with multiple solvers, constraints, tests, and SDL2 demos.

This project currently includes:

- 2D solvers: CCD, FABRIK, Jacobian (damped least squares)
- 3D solvers: CCD and FABRIK
- 2D and 3D chain data structures
- Joint constraints
- Vector and math utilities
- SDL2 visualization demos
- Automated test executable

## Current State

Recent improvements completed in this repository:

- Fixed incorrect test semantics caused by implicit enum-to-bool conversion
- Hardened solver input validation with explicit result codes
- Improved 3D forward kinematics so joint angles affect chain shape
- Fixed 3D solver behavior that could mutate segment lengths unintentionally
- Hardened vector math against divide-by-zero edge cases
- Improved build portability and cleanup behavior in `Makefile`
- Expanded tests with 3D solver smoke coverage

## Build And Run

### Requirements

- C++17 compiler (`g++` tested)
- `make`
- SDL2 (for demos only)

### Build Everything

```bash
make -B
```

### Build Targets

```bash
make lib        # static library only
make test       # test executable only
make demos      # SDL2 demo executables
make run-tests  # build tests and run them
make clean      # remove build artifacts
```

On Windows, executables are built with `.exe` suffix automatically.

## Quick Usage

### 2D CCD Example

```cpp
#include "iksolver/IKChain2D.h"
#include "iksolver/CCDSolver.h"

using namespace ik;

std::vector<Vector2> joints = {
    {0, 0},
    {1, 0},
    {2, 0}
};

IKChain2D chain(joints);
Vector2 target(1.5, 0.5);

CCDResult result = CCDSolver::Solve(chain, target, 1e-3f, 50);
if (result == CCD_SUCCESS) {
    // target reached
}
```

### 2D FABRIK Example

```cpp
#include "iksolver/FABRIKSolver.h"

FABRIKResult result = FABRIKSolver::Solve(chain, target, 1e-3f, 50);
```

### 2D Jacobian Example

```cpp
#include "iksolver/JacobianSolver.h"

JacobianSolver::SolverSettings settings;
settings.tolerance = 1e-3f;
settings.maxIterations = 100;
settings.dampingFactor = 0.1f;

JacobianResult result = JacobianSolver::Solve(chain, target, settings);
```

## Result Codes

All 2D solvers return enums, not booleans.

- CCD: `CCD_SUCCESS`, `CCD_INVALID_INPUT`, `CCD_UNREACHABLE_TARGET`, `CCD_MAX_ITERATIONS_EXCEEDED`
- FABRIK: `FABRIK_SUCCESS`, `FABRIK_INVALID_INPUT`, `FABRIK_UNREACHABLE_TARGET`, `FABRIK_MAX_ITERATIONS_EXCEEDED`
- Jacobian: `JACOBIAN_SUCCESS`, `JACOBIAN_INVALID_INPUT`, `JACOBIAN_UNREACHABLE_TARGET`, `JACOBIAN_MAX_ITERATIONS_EXCEEDED`

Use explicit comparisons against these enums in application code and tests.

## Project Layout

```text
include/iksolver/   Public headers
src/                Implementations
tests/              Test executable source
examples/           Minimal SDL2 demo
docs/               GitHub Pages web docs and design notes
```

## Documentation

- Technical internals: `README_TECHNICAL.md`
- Design notes: `docs/design.md`
- GitHub Pages site entry: `docs/index.html`

## GitHub Pages

The repository includes an automated Pages deployment workflow at `.github/workflows/pages.yml`.

To publish the site:

1. Push to `main`.
2. In repository settings, ensure GitHub Pages source is set to **GitHub Actions**.
3. Open the workflow run and wait for the `Deploy GitHub Pages` job to complete.

The deployed content is the `docs/` folder.

## Continuous Integration

Basic compile-and-test checks run via `.github/workflows/ci.yml` on push and pull request.

## Notes

- 3D kinematics in this project are intentionally lightweight and suitable for educational and prototyping scenarios.
- For robotics production stacks, consider integrating matrix/quaternion libraries and a stronger nonlinear optimization backend.
