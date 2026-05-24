# AGENTS.md

## Project Scope

This repository is a VESC firmware fork for an e-bike ESC project. The main
custom product surface is cycleIQ: the e-bike controller application and its
communication with a custom display built alongside this firmware.

The associated display is also under our control, so protocol and data-flow
changes between the ESC and display are allowed when they make the system
cleaner, safer, easier to debug, or easier to extend. Do not preserve an awkward
wire protocol only because it already exists; preserve compatibility only when
there is a concrete reason to do so.

Most feature work should focus on:

- `applications/cycleIQ/`
- `applications/app_cycleiq.c`

Touch the rest of the VESC firmware only when the change is better made in a
shared layer than inside the custom app, or when integration with VESC APIs,
hardware abstraction, build configuration, safety limits, or system behavior
requires it.

## Architecture Guidance

This code runs on embedded motor-controller hardware, so correctness, timing,
and safety matter more than cleverness. Prefer simple, explicit state and
well-bounded modules over dense abstractions.

At the same time, the current architecture is not sacred. Some parts of the
cycleIQ implementation may be early-stage, experimental, or shaped by short-term
needs. If a rewrite or refactor of a section would make the software safer,
clearer, easier to maintain, easier to test, or easier to reason about, do it.
We can afford beneficial rewrites and should not keep poor structure merely to
minimize diff size.

When refactoring:

- Keep behavior intentional and explain any behavior changes.
- Prefer small modules with clear ownership and narrow interfaces.
- Separate control policy, sensor interpretation, communication, and persistent
  data where practical.
- Avoid hidden coupling through global state unless it matches established VESC
  patterns or is necessary for realtime constraints.
- Document hardware assumptions, units, ranges, and timing assumptions close to
  the code that depends on them.

## Embedded and Realtime Constraints

Be conservative around timing-sensitive code:

- Do not block in control loops, ISR paths, or timing-critical threads.
- Keep ISR work minimal; defer processing to thread context when possible.
- Avoid dynamic allocation in realtime paths unless the surrounding firmware
  already proves it is acceptable there.
- Preserve watchdog and timeout behavior unless intentionally changing the
  safety model.
- Clamp and validate values before they can affect motor current, duty cycle,
  speed limits, thermal behavior, or battery behavior.
- Treat units explicitly. Prefer names or comments that make amps, volts, RPM,
  meters, kilometers, milliseconds, and ticks unambiguous.

Motor-control changes should fail safe. If sensor data is missing, stale,
invalid, or contradictory, prefer reducing assist/current over continuing with
stale assumptions.

## cycleIQ Development Priorities

cycleIQ is an e-bike assist application, not a generic VESC demo app. Optimize
for predictable assist behavior, rider safety, display clarity, and maintainable
firmware-display communication.

Expected areas of responsibility include:

- PAS and torque-sensor interpretation.
- Ride modes, assist levels, and current targets.
- Speed, motor temperature, controller temperature, battery, trip, and range
  data.
- Communication with the custom display.
- Configuration storage and defaults.
- Integration with VESC motor-control APIs and safety limits.

Because both ends of the display protocol are controlled by this project, it is
acceptable to redesign messages, payloads, state synchronization, or command
semantics when that improves the overall system.

## Working With Upstream VESC Code

Treat upstream VESC code as a stable platform unless there is a strong reason to
change it. Before modifying shared firmware code, ask whether the change truly
belongs there or whether cycleIQ can use an existing API.

Shared-layer changes are appropriate when:

- cycleIQ needs a capability that is naturally part of the platform.
- Fixing the issue only inside cycleIQ would duplicate logic or bypass existing
  safety mechanisms.
- The change improves integration with hardware, build configuration, or VESC
  application lifecycle behavior.

Avoid broad formatting, unrelated cleanup, or speculative rewrites in upstream
areas. Keep shared changes narrow and easy to review.

## Build and Verification

Prefer verifying changes with the most relevant firmware build target available
in this checkout. If the correct hardware target is not known, inspect the repo
or ask before assuming one.

For logic that can be checked without hardware, prefer small deterministic tests
or isolated validation code where practical. For hardware-dependent behavior,
document what could not be verified locally and what should be tested on the
controller/display.

Before considering work complete:

- Check that the code builds, or state clearly why it was not built.
- Review timing-sensitive paths for blocking calls and excessive work.
- Review safety-sensitive values for bounds, stale data handling, and fail-safe
  behavior.
- Keep changes focused on the requested behavior.

## Communication Style for Future Agents

Be direct about uncertainty. If embedded or realtime tradeoffs are involved,
explain the risk and choose the safer design unless the user asks otherwise.

The user is comfortable with beneficial refactors and rewrites. Do not hesitate
to recommend architecture improvements, but keep them tied to concrete benefits
for this e-bike ESC and display system.
