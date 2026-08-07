# GPSOpenCl

OpenCL-accelerated GPS L1 C/A software receiver.

## Priorities

- Correctness first.
- Robustness over features.
- Profiling is required.
- Minimal dependencies.

## Current Architecture

Pipeline: compute -> acquisition -> tracking -> nav decode -> measurement assembly -> PVT -> NMEA.

- `Source/Gpu/GPSOpenClGPUHandler.*`: OpenCL context, device, kernels.
- `Source/Gpu/GPSOpenClSpectrumEngine.*`: FFT, mixing, GPU/CPU dispatch.
- `Source/Acquisition/GPSOpenClCaCodeGenerator.*`: GPS C/A code generation.
- `Source/Acquisition/GPSOpenClAcquisition.*`: Doppler search, FFT correlation.
- `Source/Tracking/GPSOpenClTracking.*`: PLL/DLL loops, E/P/L correlators.
- `Source/Tracking/GPSOpenClLockDetector.*`: Carrier and code lock indicators.
- `Source/Tracking/GPSOpenClTrackingWorkerPool.*`: Per-block worker pool running channel tracking.
- `Source/Tracking/GPSOpenClChannel.*`: Per-satellite acquisition and tracking state.
- `Source/NavDecode/GPSOpenClNavigationDecoder.*`: Preamble search, parity, subframe parsing.
- `Source/Pvt/GPSOpenClMeasurementAssembler.*`: Pseudorange and transmit-time assembly for the PVT solver.
- `Source/Pvt/GPSOpenClPVTSolver.*`: Satellite orbit calc, WLS position, DOP.
- `Source/Pvt/GPSOpenClAtmosphericCorrections.*`: Klobuchar and Saastamoinen delay models.
- `Source/Sink/GPSOpenClNmeaGenerator.*`: NMEA-0183 sentence output.
- `Source/Common/GPSOpenClSettings.*`: INI config parsing.
- `Source/Application/GPSOpenClApplication.*`: Wires all modules together.
- `Source/Sink/GPSOpenClTelemetryExporter.*`: Background console and JSON telemetry export.
- `Source/Common/GPSOpenClCommon.hpp`: Shared types and GPS constants.
- `Source/Common/GPSOpenClStructs.hpp`: Shared wire structs, single source of truth.
- `Source/Input/GPSOpenClSource.hpp`, `GPSOpenClFileSource.*`, `GPSOpenClGpsSdrSimSource.*`: Abstract `Source`, plus file and gps-sdr-sim FIFO implementations.
- `Source/Sink/GPSOpenClSink.hpp`, `GPSOpenClFileSink.*`, `GPSOpenClZmqSink.*`: Abstract `Sink`, plus `NullSink`/`CompositeSink`, file, and ZMQ publisher implementations.
- `Source/Common/GPSOpenClProfiler.*`: Per-block, per-stage timing, published through the Sink.
- `Source/Common/GPSOpenClBoundedQueue.hpp`: Bounded blocking queue between producer and consumer threads.
- `Kernels/*.cl`: OpenCL FFT, complex multiply, and magnitude kernels.

Folder layout under `Source/`: `Application/`, `Common/`, `Gpu/`, `Input/`, `Acquisition/`, `Tracking/`, `NavDecode/`, `Pvt/`, `Sink/`, each with its own `CMakeLists.txt` pulled in via `add_subdirectory`. `Application/Main.cpp` is the entry point; `Application/GPSOpenClApplication.*` does the top-level wiring.

Data flow:

- `Main.cpp` reads from file or gps-sdr-sim FIFO.
- Producer thread (Source) and consumer thread (Application) linked by bounded queue.
- ZMQ publisher gated behind `GPSOPENCL_ENABLE_ZMQ`. FileSink and NullSink also exist.
- Profiler publishes per-block timing through Sink.
- Falls back to CPU if no OpenCL device.
- Every module has Input/Output structs. Sink publish methods are typed with static_assert size guards. `Tests/StructsSourceSinkTest.cpp` exercises the wire format.

## Target Architecture

Design direction, not yet fully built.

### Source abstraction

- Abstract `Source` base class. Concrete source reads gps-sdr-sim FIFO.
- Raw IQ samples stream as bytes, parsed into `ComplexFloatVector`.
- Only Source health telemetry goes through Sink.
- Designed for future hardware SDR plug-in (RTL-SDR, USRP, BladeRF).

### gps-sdr-sim real-time fork

- Fork at `beratatmaca/gps-sdr-sim-rt`, vendored in `Tools/gps-sdr-sim`.
- Streams samples over Linux FIFO: `/tmp/gpsopencl/sim_data.fifo`.
- Control FIFO: `/tmp/gpsopencl/sim_ctrl.fifo` (plain text: `START`, `STOP`, `SET_POS`).

### Module output contract

- Every module: one input struct (config), one output struct (telemetry).
- All structs in `Source/Common/GPSOpenClStructs.hpp`. Single source of truth.
- Never mix config into telemetry or vice versa.

### Struct wire format

- First member: `uint32_t structVersion`.
- Fixed-width types only: `int32_t`, `uint32_t`, `double`. No `bool`, `size_t`, `int`.
- `#pragma pack(push, 1)`. Little-endian, no byte swap.
- Per-satellite telemetry: one message per SV, no arrays.
- Raw IQ samples are not wrapped in a struct.

### Publisher / Sink

- Abstract Sink. ZMQ PUB/SUB is default implementation.
- Endpoint: `ipc:///tmp/gpsopencl/<name>.sock`. TCP override allowed.
- Message: two frames (identifier + struct bytes).
- Fallback: NullSink or FileSink if ZMQ disabled.

### Profiler module

- Per-block, per-stage timing. Publishes through Sink.
- Must not distort the timing it measures.
- Disableable at compile time or runtime.

### Concurrency model

- Producer thread (Source) + consumer thread (Application) linked by bounded queue (8-16 blocks).
- Algorithm path backpressures. Sink path may drop.

### Visualization

- Visualizers are ZMQ subscribers, not core logic. Keep out of hot path.

### Deployment goal

- Linux only (x86_64, ARM64). Target: RPi 4/5, Jetson.
- GPU optional, CPU fallback mandatory.
- No dependency that blocks embedded deployment.

## Dependencies

- C++17 stdlib, OpenCL (GPU only), ZMQ (Sink only, gated by `GPSOPENCL_ENABLE_ZMQ`).
- FIFO for sim IPC, no library.
- Python tools may use additional packages.
- Justify any new dependency in PR.

## Build

```bash
cmake -S . -B build
cmake --build build
```

CMake 3.14+, C++17. OpenCL headers and GoogleTest auto-fetched.

## Test

```bash
ctest --test-dir build --output-on-failure
```

118 GoogleTest cases. Every new algorithm needs a test.

## Benchmarking

```bash
python3 Tools/e2e_benchmark.py --duration 5
```

Writes JSON and Markdown reports. Run after any algorithm change.

## Live Demo

```bash
python3 run_system.py
```

Builds, streams signal over FIFO, launches Dash dashboard on `localhost:8050`.

## Code Style

- `.clang-format` before committing.
- 4-space indent, Allman braces, 120 columns.
- Namespace: `GPSOpenCl`.
- File naming: `GPSOpenCl<ModuleName>.hpp` / `.cpp`.

## Commenting Rules

- **C++ headers (`.hpp`):** Doxygen only. `/** */` for classes, structs, functions. `///<` for fields. No `//` or `/* */`.
- **C++ sources (`.cpp`):** Zero comments. NOLINT pragma comments allowed.
- **Tests (`Tests/*.cpp`):** Brief rationale comments allowed. `//` on the preceding line.
- **Python (`.py`):** One `#` comment on the preceding line. No docstrings, no inline comments.
- **CMake (`CMakeLists.txt`):** One `#` comment on the preceding line.
- **OpenCL (`.cl`):** Doxygen `/** */` blocks on every kernel.
- Simple English. Short, direct, no filler.

## Working Rules

- Never fake or stub algorithm correctness.
- Verify PLL/DLL and PVT math against references.
- Prefer measured profiling over assumed performance.
- Ask before adding a new dependency.
- GPU and CPU compute paths must be algorithmically identical and agree within a small numeric tolerance. Acquisition decisions must not depend on backend beyond that tolerance.
- New modules must fit the Source/struct/Sink contract.
