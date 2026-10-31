# Repository Guidelines

## Project Structure & Module Organization

`road_finder` detects road geometry and junctions for the RDK X5 using C++17 and OpenCV. `src/main.cpp` handles CLI options and the processing loop; the remaining source files implement camera capture, detection, preview rendering, and UART output. Public headers live in `include/road_finder/`. Runtime calibration is in `config/road_finder.yaml`. Tests live in `tests/`, with sample images in `tests/fixtures/`. Keep generated files in the ignored `build/` directory.

## Build, Test, and Development Commands

Requires CMake 3.16+ and OpenCV 4 with core, imgproc, videoio, highgui, and imgcodecs; OpenCV 5 is also supported for development. Run commands from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

These configure, compile, and run all registered tests. For a headless image run:

```sh
./build/road_finder --source tests/fixtures/frame_784.png --no-display --no-serial --json
```

Use `--source /dev/video0` for camera input. Add `--benchmark 300` to measure processing stages over 300 frames.

## Coding Style & Naming Conventions

Follow existing C++17 style: two-space indentation, same-line opening braces, and the `road_finder` namespace. Use `PascalCase` for types, `camelCase` for functions, `snake_case` for variables and filenames, trailing underscores for private members, and `kPascalCase` for constants. Keep declarations in `.hpp` files and implementations in `.cpp` files. No formatter or linter is configured; compile cleanly with the existing `-Wall -Wextra -Wpedantic` flags.

## Testing Guidelines

Tests are standalone C++ executables registered with CTest, using local assertion helpers rather than an external framework. Name files `tests/<feature>_test.cpp` and register new executables in `CMakeLists.txt`. Cover detector geometry, configuration validation, source parsing, preview behavior, and UART packet encoding as relevant. Prefer deterministic synthetic masks and fixtures. No coverage threshold is configured; add regression cases for behavior changes. Image fixtures use Git LFS; fetch their actual contents before testing.

## Commit & Pull Request Guidelines

History favors short, descriptive Chinese commit subjects; no strict prefix convention is established. Keep commits focused. Pull requests should describe the problem, resulting behavior, and validation commands/results. Link related issues and include overlays or screenshots for detection or preview changes. Document configuration and CLI changes in `README.md`; report RDK X5 performance measurements when relevant.
