# GPSOpenCl Development Plan

Roadmap toward the target architecture in `CLAUDE.md`.

## Guiding Rules

- Follow `CLAUDE.md` for every phase below.
- One struct header, single source of truth.
- Structs are simplex: one input, one output.
- Input struct is config. Output struct is telemetry.
- Add no dependency beyond OpenCL, ZMQ, gtest.
- Every phase ends with tests still green.
- Assume little-endian hardware. Linux only, for now.

## Phase 1 — Struct Header Foundation

Goal: define every struct in one place first.

- Create `Source/GPSOpenClStructs.h`.
- Define one input (config) struct per module.
- Define one output (telemetry) struct per module.
- No struct for raw IQ samples. They stay raw.
- Define `SourceInput`: FIFO path, format, sampling rate.
- Define `SourceOutput`: block index, timestamp, FIFO health.
- Define the profiler output struct.
- Every struct starts with `uint32_t structVersion`.
- Use only fixed-width types: `int32_t`, `uint32_t`, `double`.
- Pack every struct. No compiler padding allowed.
- Per-satellite output structs carry one `svId` field.
- Publish one message per SV. No 32-slot arrays.
- Migrate `Settings::Configuration` fields into input structs.
- No behavior change. Pure struct extraction.

Exit criteria:

- Project builds with no logic changes.
- All 33 existing tests still pass.

## Phase 2 — Abstract Source

Goal: decouple sample input from file reading.

- Define abstract `Source` base class.
- Move current file-reading logic into `FileSource`.
- Add `GpsSdrSimSource` for real-time simulator feed.
- Source parses raw FIFO bytes into `ComplexFloatVector`.
- Sample data itself never becomes a struct.
- Source publishes `SourceOutput` health telemetry via Sink.
- Design for a future hardware SDR source.

### gps-sdr-sim real-time fork

- Fork gps-sdr-sim into `beratatmaca/gps-sdr-sim-rt`.
- Keep vendored copy in `Tools/gps-sdr-sim`.
- Preserve upstream license, note local changes.
- Add real-time pacing to sample generation.
- Stream samples out over a Linux FIFO.
- Data FIFO at `/tmp/gpsopencl/sim_data.fifo`.
- Skip the file write entirely, for live runs.
- Add a control FIFO at `/tmp/gpsopencl/sim_ctrl.fifo`.
- Control channel supports start, stop, reconfigure commands.
- Commands are plain text lines, not binary.
- Example: `START`, `STOP`, `SET_POS 48.11,11.51,545`.
- `GpsSdrSimSource` reads the data FIFO directly.
- `GpsSdrSimSource` writes commands to the control channel.

Exit criteria:

- `FileSource` reproduces current E2E test result.
- `GpsSdrSimSource` streams live blocks correctly.
- Fork streams samples with no intermediate file.
- Commands change simulator state without a restart.

## Phase 3 — Abstract Sink and ZMQ Publisher

Goal: decouple module output from stdout/JSON.

- Define abstract `Sink` base class.
- Implement `ZmqSink` as one concrete publisher.
- Publish each output struct with an identifier.
- Use ZMQ PUB/SUB, message is two frames.
- Frame 1 is the identifier, frame 2 the struct.
- Default endpoint: `ipc:///tmp/gpsopencl/<name>.sock`.
- Allow `tcp://` override, for remote dashboards.
- Add ZMQ as a FetchContent dependency.
- Gate it behind CMake option `GPSOPENCL_ENABLE_ZMQ`.
- If disabled, build `NullSink` or `FileSink` instead.
- Keep Sink interface independent of ZMQ.

Exit criteria:

- A test subscriber receives every struct type.
- Sink can be swapped without touching modules.
- Build succeeds with `GPSOPENCL_ENABLE_ZMQ` off.

## Phase 4 — Module Struct Wiring

Goal: move every module onto the struct contract.

Apply to: Acquisition, Tracking, NavigationDecoder, PVTSolver,
AtmosphericCorrections, NmeaGenerator.

- Replace ad hoc getters with the output struct.
- Replace scattered config args with the input struct.
- Route each module's output through the Sink.
- Update unit tests for new struct signatures.

Exit criteria:

- All modules only speak structs, in and out.
- All 33 tests still pass.

## Phase 5 — Profiler Module

Goal: measure per-module compute time.

- Define a `Profiler` class, scoped RAII timer.
- Wrap each module call inside `Application`.
- Emit one profiler output struct per block.
- Publish profiler struct through the same Sink.
- Add a compile-time and runtime disable flag.

Exit criteria:

- Profiler overhead is measured and under 1%.
- Timing data is visible per module, per block.

## Phase 6 — Real-Time Application Loop

Goal: replace batch file reading with live streaming.

- Replace the block loop in `Main.cpp`.
- Drive the loop from the `Source` abstraction.
- Run `Source` and `Application` on separate threads.
- Link them with a bounded blocking queue.
- Size the queue at about 8 to 16 blocks.
- Algorithm path backpressures. It never drops samples.
- Sink/telemetry path may drop under load instead.
- Validate against `GpsSdrSimSource` for hours, not seconds.

Exit criteria:

- Runs continuously with no memory growth.
- No dropped blocks on the algorithm path.

## Phase 7 — Visualizers

Goal: terminal and dashboard as pure subscribers.

- Build a terminal visualizer, ZMQ subscriber only.
- Show C/N0, PVT fix, and NMEA lines.
- Migrate the Plotly dashboard to ZMQ subscribing.
- Remove the current JSON file polling.

Exit criteria:

- Dashboard renders live data with no file I/O.
- Visualizers can be killed without affecting the pipeline.

## Phase 8 — Hardware Deployment Validation

Goal: prove the pipeline runs on real hardware.

- Run the full pipeline on non-GPU hardware.
- Target boards: Raspberry Pi 4/5, NVIDIA Jetson.
- Confirm CPU fallback timing stays acceptable.
- Document install steps, and minimal dependency list.
- Confirm ZMQ sink works across process boundaries.

Exit criteria:

- Full pipeline runs end to end on target hardware.

## Ongoing, Every Phase

- Run `Tools/e2e_benchmark.py` after each phase.
- Compare throughput against the previous phase.
- Update `CLAUDE.md` as target becomes current.
- Do not merge a phase with failing tests.

## Risks

- Struct migration may break existing call sites.
- ZMQ dependency must stay optional at build time.
- Real-time loop can drop samples under load.
- Profiler code must not skew the numbers it reports.
