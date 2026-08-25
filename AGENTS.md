# Repository Guidelines

## Project Structure & Module Organization
MuJoCo is a C/C++ CMake project with several language bindings and frontends. Core engine code lives in `src/`, public headers in `include/mujoco/`, plugins in `plugin/`, sample apps in `sample/` and `simulate/`, and test code in `test/`. Python bindings are under `python/mujoco/`, MJX lives in `mjx/mujoco/mjx/`, WebAssembly support is in `wasm/`, Unity integration is in `unity/`, and reference models/assets are in `model/`. Keep new tests close to the subsystem they validate, following the existing `test/<area>/` layout.

## Build, Test, and Development Commands
Use the top-level CMake flow for native development:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

For Python work, install build requirements first, then run package tests:

```bash
python -m pip install --upgrade --require-hashes -r python/build_requirements.txt
pytest -v --pyargs mujoco
```

For MJX changes, work from `mjx/` and run:

```bash
pytest -n auto -v -k 'not IntegrationTest' --pyargs mujoco.mjx
```

WASM contributors typically use `emcmake cmake -B build` followed by `cmake --build build`.

## Coding Style & Naming Conventions
Follow `STYLEGUIDE.md`. C code uses 2-space indentation, attached K&R braces, short names, and comments as brief block summaries. Keep C lines near 100 columns. New C++ in `test/` and `python/` follows Google style. Python formatting uses `pyink` with 2-space indentation and `isort` for imports:

```bash
pyink path/to/file.py
isort path/to/file.py
pre-commit run --all-files
```

## Testing Guidelines
Native tests use GoogleTest, Python bindings use `absltest`/`pytest`, and Unity uses NUnit. Name tests after the unit under test, using patterns already in the tree such as `*_test.cc`, `*_test.py`, and `*Tests.cs`. PRs are expected to add or extend tests for changed behavior; if you touch untested code, add baseline coverage for the existing path too.

## Commit & Pull Request Guidelines
Recent commits use short, imperative, sentence-case subjects such as `Add test for geom and mesh plugin attributes.` Keep commits focused. For non-trivial work, coordinate first per `CONTRIBUTING.md`. Pull requests should be small, include passing tests, resolve compiler warnings, and explain the change, affected platforms, and any linked issue. Add screenshots when UI-facing behavior changes.

## Git Remote Operations
Do not run `git push` (including as part of a combined command) without the user's explicit permission in the current request. Creating local commits and fetching remotes are allowed when they are in scope, but always report pending commits and ask before publishing them.
