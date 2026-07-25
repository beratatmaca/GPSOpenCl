# GPSOpenCl

OpenCL-accelerated GPS L1 C/A software receiver.

## Priorities

- Accuracy and correctness come first.
- Robustness matters more than features.
- Profiling is required, not optional.
- Build a genuinely good software GPS receiver.
- Keep dependencies minimal. Justify every new one.

## Current Architecture (verified in repo)

Pipeline order: compute -> acquisition -> tracking -> nav decode -> PVT -> NMEA.

- `Source/GPSOpenClGPUHandler.*`: OpenCL context, device, kernels.
- `Source/GPSOpenClGPUCompute.*`: FFT, mixing, GPU/CPU dispatch.
- `Source/GPSOpenClCode.*`: GPS C/A code generation.
- `Source/GPSOpenClAcquisition.*`: Doppler search, FFT correlation.
- `Source/GPSOpenClTracking.*`: PLL/DLL loops, E/P/L correlators.
- `Source/GPSOpenClChannel.*`: Per-satellite acquisition and tracking state.
- `Source/GPSOpenClNavigationDecoder.*`: Preamble search, parity, subframe parsing.
- `Source/GPSOpenClPVTSolver.*`: Satellite orbit calc, WLS position, DOP.
- `Source/GPSOpenClAtmosphericCorrections.*`: Klobuchar and Saastamoinen delay models.
- `Source/GPSOpenClNmeaGenerator.*`: NMEA-0183 sentence output.
- `Source/GPSOpenClSettings.*`: INI config parsing.
- `Source/GPSOpenClApplication.*`: Wires all modules together.
- `Kernels/*.cl`: OpenCL kernels for FFT and NCO.

Data flow today:

- Reads from a file. Not real-time yet.
- Input: binary IQ file, or text signal file.
- Falls back to CPU if no OpenCL device.
- No abstract `Source` class exists yet.
- No abstract `Sink`/output class exists yet.
- No ZMQ publisher exists yet.
- No profiler module exists yet.

## Target Architecture (direction, not yet built)

This is the design we are building toward.

### Source abstraction

- Abstract `Source` base class for all inputs.
- Concrete source reads gps-sdr-sim in real time.
- Source yields raw IQ samples to the pipeline.
- Design so real hardware SDRs can plug in later.

### Module output contract

- Every module outputs one fixed binary struct.
- Struct layout must be stable and documented.
- Output type is also an abstract class.
- Downstream code consumes the struct, not internals.
- All structs live in one shared header.
- Proposed path: `Source/GPSOpenClStructs.h`.
- That header is the single source of truth.
- No module defines its own duplicate struct.
- Source, Sink, and Profiler all include it.
- Changing a struct means one file to review.

Structs are simplex: one input, one output.

- Each module has exactly one input struct.
- Each module has exactly one output struct.
- Input struct holds configuration fields only.
- Output struct holds telemetry fields only.
- Never mix config fields into a telemetry struct.
- Never mix telemetry fields into a config struct.

### Publisher / Sink

- Sink is abstract. ZMQ is one implementation.
- For now, sink implementation is a ZMQ publisher.
- Each struct is published with an identifier.
- One identifier per module or struct type.

### Profiler module

- Dedicated `Profiler` module, separate from algorithms.
- Wraps each module's compute call with timing.
- Measures per-module latency, per processing block.
- Profiler output is also a binary struct.
- Profiler struct publishes through the same Sink path.
- Report throughput, and per-stage timing breakdown.
- Profiling must not distort the timing it measures.
- Can be disabled at compile time or runtime.

### Visualization

- Terminal and dashboard visualizers come later.
- Visualizers are ZMQ subscribers, not core logic.
- Keep visualizers out of the real-time hot path.

### Deployment goal

- Whole pipeline must run on real hardware.
- No dependency that blocks embedded or edge deployment.
- GPU acceleration optional. CPU fallback is mandatory.

## Dependency Policy

- Keep core pipeline dependencies minimal.
- C++17 standard library is always fine.
- OpenCL allowed, for GPU acceleration only.
- ZMQ allowed, for source and sink transport only.
- No heavy frameworks inside the core pipeline.
- Python tools (dashboard, benchmark) may use more.
- Justify any new dependency in the PR.

## Build

```bash
cmake -S . -B build
cmake --build build
```

- Requires CMake 3.14+, and C++17.
- OpenCL headers auto-fetched if not found.
- GoogleTest auto-fetched via FetchContent.

## Test

```bash
ctest --test-dir build --output-on-failure
```

- 33 GoogleTest cases, across all modules.
- Covers acquisition, tracking, PVT, NMEA, atmosphere.
- `Tests/E2ETest.cpp` runs one full pipeline pass.
- Add a test for every new algorithm.
- Do not merge algorithm code without a test.

## Benchmarking

```bash
python3 Tools/e2e_benchmark.py --duration 5
```

- Writes `e2e_benchmark_report.json` and `.md`.
- Reports throughput in samples per second.
- Run this after any algorithm change.
- Watch for timing regressions in acquisition and tracking.

## Live Demo

```bash
python3 run_live_demo.py
```

- Builds the project, generates a signal, launches dashboard.
- Auto-compiles `gps-sdr-sim` if missing.
- Dashboard is Plotly Dash, served on `localhost:8050`.

## Code Style

- Format with `.clang-format` before committing.
- 4-space indent, Allman braces, 120 columns.
- One namespace for everything: `GPSOpenCl`.
- File naming: `GPSOpenCl<ModuleName>.h` / `.cpp`.

## Working Rules

- Never fake or stub algorithm correctness.
- Verify PLL/DLL and PVT math against references.
- Prefer measured profiling over assumed performance.
- Ask before adding a new external dependency.
- Keep GPU and CPU compute paths behaviorally identical.
- New modules must fit the Source/struct/Sink contract.
