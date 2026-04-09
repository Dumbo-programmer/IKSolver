# IKSolver Design Notes

## Design Goals

- Keep solver code readable for learning and experimentation.
- Provide deterministic result codes for application-level control flow.
- Keep dependencies minimal (standard library + SDL2 for demos).
- Support both 2D and 3D usage paths under a shared math/chain model.

## Solver Layering

- Chain classes (`IKChain2D`, `IKChain3D`) own geometry state.
- Solver classes mutate chain state in-place.
- Constraint objects remain lightweight and optional.

This separation keeps solver implementations independent from rendering and demo code.

## Current Tradeoffs

- 2D solvers are more mature and better covered by tests.
- 3D solver path favors simplicity over full rigid-body fidelity.
- Jacobian implementation intentionally avoids external matrix libraries.

## Extension Strategy

Preferred extension order:

1. Improve solver quality with additional deterministic tests.
2. Add optional math backend (Eigen/GLM) behind compile-time flag.
3. Add orientation goals and multi-target constraints.
4. Add performance benchmarks and CI test matrix.

## Documentation Strategy

- `ReadMe.md`: user-facing quick start and API entry.
- `README_TECHNICAL.md`: implementation and maintainer details.
- `docs/index.html`: GitHub Pages landing site and interactive web demo.
