# Repository Guidelines

## Project Structure & Module Organization
`include/cbs/` contains the public API, organized by subsystem: `bpsam/` for the main optimizer, `gbp/` for Gaussian belief propagation primitives, `robust/` for outlier rejection, and `utils/` for graph I/O and helpers. Matching implementation files live in `src/`. Utility executables are defined under `src/utils/`, while runnable examples live in `examples/` (`dpgo_g2o`, `cbs_code_offline`). Reference docs are in `docs/`, sample datasets in `data/`, the academic paper for this repository is `cbs_ral.pdf`, and vendored dependencies live in `thirdparty/gkcm/`.

## Build, Test, and Development Commands
Prefer the checked-in `pixi` tasks for a consistent environment:

- `pixi run build`: configure and build this package in `build/`.
- `pixi run build-all`: also builds sibling dependencies such as `../aria_common`.
- `cmake --preset config && cmake --build build --config Release`: direct CMake equivalent.
- `pixi run dpgo --dataset kitti_00.g2o`: run the visualization example from `build/examples/Release/`.
- `ctest --test-dir build --output-on-failure`: run any enabled CMake tests after building.

## Coding Style & Naming Conventions
This is a C++17 codebase with `.clang-format` based on Google style. Use 2-space indentation, 80-column lines, attached braces, and left-aligned pointers (`Type* ptr`). Keep headers in `include/cbs/...` and implementations in the matching `src/...` path. Follow the existing naming patterns: snake_case for files and functions, PascalCase for types, and concise lowercase commit titles.

## Testing Guidelines
There is no large first-party test suite at the top level yet; most existing tests are inside `thirdparty/gkcm/` and use GoogleTest with filenames ending in `_tests.cpp`. When adding tests for CBS code, keep them close to the relevant module, register them with CMake, and make sure they run through `ctest`. Validate example and utility changes by running the affected executable against `data/` inputs where possible.

## Commit & Pull Request Guidelines
Recent history favors short, imperative, lowercase commit subjects such as `fix crashing in campus's log` or `add soft reset to KLD, WIP`. Keep subjects under about 72 characters and describe one logical change per commit. Pull requests should explain the behavioral change, list build/test commands run, link any related issue, and include screenshots or plots when modifying visualization or trajectory outputs.

## Dependency & Configuration Notes
Core dependencies include GTSAM, TBB, `aria_common`, `aria_viz`, Boost, `yaml-cpp`, and `spdlog`. Avoid editing `thirdparty/` unless the change is intentional and isolated. If a feature depends on external datasets or sibling repos, document the required path or `pixi` task in the PR.
