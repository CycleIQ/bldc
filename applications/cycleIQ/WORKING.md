# cycleIQ Current Working State

This document describes what the cycleIQ app currently does in firmware. It is
based on `applications/app_cycleiq.c`, the modules in this directory, and the
shared `external/cycleiq-protocol` SDK.

## Startup and Shutdown

`app_custom_start()` initializes the app in this order:

1. Resets cycleIQ runtime data and initializes Walk Mode inactive.
2. Registers the CAN receive callback.
3. Resets motor current output to 0 A.
4. Initializes PAS and torque sensor state.
5. Configures PAS thresholds:
   - start pedaling threshold: 15 pedal RPM
   - maximum accepted pedal speed: 240 pedal RPM
   - 18 magnets on `CYCLEIQ_HAS_2_WIRE_PAS` hardware
   - 36 magnets otherwise
6. Starts the motor speed/temperature sensor worker.
7. Starts the main cycleIQ service thread.

`app_custom_stop()` stops the service thread first, then the sensor worker,
unregisters CAN so no new walk heartbeat can arrive, clears Walk Mode, forces
motor current to 0 A, deinitializes PAS, and clears PAS configuration.

`app_custom_configure()` currently ignores VESC app configuration.

## Main Service Loop

`service.c` runs the main `CYCLEIQ` thread every 10 ms. Each iteration:

1. Updates PAS and torque-sensor state.
2. Refreshes battery, current, controller temperature, and power fields.
3. Expires Walk Mode if its command heartbeat is stale.
4. Sends due CAN telemetry packets.
5. Computes and applies the motor current command.
6. Resets the VESC timeout watchdog.

On stop, the service thread calls `cycleiq_control_stop()` before exiting.

## Runtime Data

`data.c` owns the global `cycleiq_config` and `cycleiq_data` state.

Default configuration:

- max speed: 25 km/h
- battery internal resistance: 0.05 ohm
- wheel diameter: 0.66 m

The display protocol does not configure the ESC. Firmware validates and loads
the three local values from custom EEPROM only when its magic/version and every
stored value are valid; otherwise these defaults remain active.

Data initialization resets transient values and starts in:

- PAS support mode
- normal ride mode
- motor enabled
- normal max gear of 3
- current gear 3

Mountain ride mode raises the max gear:

- 6 when `CYCLEIQ_HIGH_POWER` is defined
- 5 otherwise

If the current gear is above the ride-mode limit, it is clamped down.

Currently refreshed from VESC APIs:

- battery voltage from `mc_interface_get_input_voltage_filtered()`
- battery level mapped linearly from 44.0 V to 54.6 V
- input battery current from `mc_interface_get_tot_current_in_filtered()`
- motor current from `mc_interface_get_tot_current_directional_filtered()`
- controller temperature from `mc_interface_temp_fet_filtered()`
- motor power as signed battery current times battery voltage, clamped to the
  protocol's signed-watt range
- watt-hours and amp-hours as session-relative net consumed deltas from the
  VESC Ah/Wh counters, without resetting the shared VESC counters

Motor speed and motor temperature are updated by `sensors.c`. Speed is computed
from RPM and configured wheel diameter:

```text
speed_mps = rpm * wheel_diameter_m * pi / 60
```

Trip distance is integrated from the cycleIQ wheel-hall speed over elapsed
system time. Trip time is session elapsed time, and average speed is trip
distance divided by elapsed trip time, including stopped time.

The app validates incoming gear, support-mode, ride-mode, and walk commands
before changing state.

## Assist Control

`control.c` converts the active support state into a motor phase-current command.
Each gear has a battery-current budget and a derived low-speed phase-current
ceiling. The phase-current ceiling is calculated as:

```text
phase_current_limit_a = battery_current_limit_a / 0.25
```

This makes the gear feel phase-current limited below approximately 33% duty, and
battery-current limited above approximately 33% duty.

Normal ride-mode gear currents:

| Gear | Battery current |
| ---: | --------------: |
| 1 | 2.0 A |
| 2 | 3.125 A |
| 3 | 5.0 A |

Mountain ride-mode gear currents:

| Gear | Battery current |
| ---: | --------------: |
| 1 | 2.5 A |
| 2 | 5.0 A |
| 3 | 9.0 A |
| 4 | 15.0 A |
| 5 | 30.0 A |
| 6 | 40.0 A |

Gear 0 maps to 0 A internally, although external gear setting rejects gear 0.

Support modes currently behave as follows:

- PAS: applies the selected gear current only while PAS reports pedaling.
- Torque: applies selected gear current multiplied by torque sensor percentage
  only while the torque sensor is active.
- Hybrid: currently applies 0 A.

If `motor_enabled` is false, target current is 0 A regardless of mode.

In normal ride mode, assist is tapered over the final 2 km/h before
`max_speed_kph` and reaches 0 A at or above the configured maximum speed.
Mountain ride mode bypasses this application-level speed limit.

Walk Mode overrides PAS, torque, gear, and ride mode while active. It uses a
fixed 2.0 A battery-current target and a derived phase-current ceiling of about
6.06 A. It applies full demand below 5 km/h, linearly tapers demand from 5 to
6 km/h, and commands 0 A at or above 6 km/h. The limit uses the greater of the
absolute VESC motor-derived speed and valid cycleIQ wheel speed. A non-finite
motor-derived speed fails safe to 0 A.

Walk Mode starts only from a valid ON command while the motor is enabled. Every
valid ON command refreshes a 1000 ms deadline. OFF, deadline expiry, power-off,
or application shutdown clears the mode. OFF and expiry clear the phase-current
accumulator and command 0 A in the next 10 ms control iteration; regular assist
is reconsidered on the following iteration.

The selected gear current is scaled by support mode and speed taper. The result
sets both:

- a battery-current target for the active gear
- a phase-current target capped by the derived phase-current ceiling

The phase-current target is also capped by `target_battery_current / duty` when
duty rises high enough for the battery-current budget to dominate. Duty is
floored at 0.02 to avoid an extreme divide near zero.

Battery-current feedback from `cycleiq_data.battery_current_a` trims the
phase-current target down when measured input current exceeds the active
battery-current target. Recovery after trimming happens through the normal
ramp-up path.

The output ramp is explicit:

- PAS ramp up: 40 A/s
- PAS ramp down: 40 A/s
- PAS fast release to zero: 80 A/s
- torque ramp up: 150 A/s
- torque ramp down while active: 150 A/s
- torque fast release to zero: 300 A/s

The final command is saturated to the live positive phase-current limit, using
`lo_current_max` when available and falling back to `l_current_max`. VESC remains
the hard safety layer for current, input current, voltage, temperature, duty,
RPM, and watchdog behavior.

`cycleiq_control_init()` and `cycleiq_control_stop()` both clear the internal
phase-current output state and command 0 A.

## PAS and Torque Sensor

`pas.c` supports both one-wire and two-wire PAS builds.

PAS input handling:

- TIM7 samples PAS input at 10 kHz.
- Inputs are software filtered for approximately 1.5 ms.
- Raw EXTI handling is intentionally a compatibility no-op; decoded PAS state
  comes from the timer sampler.
- The decoder rejects transitions that occur sooner than the filter period.
- For two-wire PAS, quadrature direction is checked with a lookup table.
- Invalid two-wire transitions are ignored.
- Backward two-wire motion resets the correct-direction counter.
- Pedaling becomes active only after enough correct-direction events:
  - half the magnet count on one-wire PAS
  - two full state transitions per magnet half-window on two-wire PAS
- Pedaling times out when the latest accepted pulse is older than the configured
  start-RPM pulse period.

Pedal RPM is updated once per pulse when the decoded state returns to 0. The RPM
estimate is low-pass filtered.

Torque sensor handling:

- During PAS init, the torque sensor zero point is measured over 10 samples with
  10 ms gaps.
- The measured zero point is multiplied by 1.03 and used as the minimum torque
  voltage.
- Runtime torque voltage has a short fixed low-pass filter for ADC noise.
- Every accepted forward PAS pulse snapshots the current torque ADC reading.
  These equal-angle samples feed a rolling half-revolution average (9 samples
  on 18-magnet two-wire hardware; 18 on 36-magnet single-wire hardware).
- The assist demand is the angular average plus 20% of the short-filtered
  torque residual, retaining initial response while rejecting pedal-stroke
  ripple.
- A fast release bypasses the angular average for invalid torque data, a stale
  forward PAS transition, or torque remaining near zero across one third of a
  crank revolution. A short torque valley by itself does not fast-release.
- Torque percentage maps calibrated minimum to 0.0 and 2.4 V to 1.0, then
  clamps to a maximum of 1.5.

## Motor Speed and Motor Temperature Sensor

`sensors.c` samples the shared motor temperature/speed ADC input.

Working behavior:

- TIM6 samples the input at 5 kHz.
- ADC values below 50 are treated as the wheel speed pulse being low.
- A falling edge into the low state is treated as a wheel pulse.
- One pulse is interpreted as one wheel rotation.
- RPM is computed from the pulse interval and low-pass filtered.
- If no pulse is seen for 3 seconds, RPM is forced to 0.
- A publish thread wakes every 10 ms, transfers the latest sampled pulse data,
  updates temperature when allowed, publishes RPM/temperature into `cycleiq_data`,
  and resets the VESC timeout watchdog.

The wheel hall switch and motor NTC share `ADC_IND_TEMP_MOTOR`. Temperature is
sampled only when the speed input is high and at least 3 ms have passed since the
last pulse and previous temperature sample. Motor temperature is computed with
`NTC_TEMP_MOTOR(3435.0f)`, clamped to `int8_t`, and low-pass filtered.

## CAN Commands From Display

`comm.c` registers an extended-ID CAN receive callback with
`comm_can_set_eid_rx_callback()`.

The shared protocol uses the display node ID `CYCLEIQ_DISPLAY_CAN_ID` (`0x6A`)
and ESC node ID `CYCLEIQ_ESC_CAN_ID` (`0x6B`). The extended CAN ID has the
destination in bits 8..15 and packet type in bits 0..7. Multi-byte payloads are
big-endian. The ESC accepts only frames addressed to `CYCLEIQ_ESC_CAN_ID`.

Implemented commands:

| Command | Behavior |
| --- | --- |
| `CYCLEIQ_COMMAND_GEAR_UP` | increments gear by one, up to the ride-mode limit |
| `CYCLEIQ_COMMAND_GEAR_DOWN` | decrements gear by one, down to zero |
| `CYCLEIQ_COMMAND_SET_SUPPORT_MODE` | selects validated PAS or torque support |
| `CYCLEIQ_COMMAND_SET_RIDE_MODE` | selects validated normal or mountain mode |
| `CYCLEIQ_COMMAND_SET_WALK` | exact one-byte boolean; ON activates/refreshes Walk Mode and OFF clears it |
| `CYCLEIQ_COMMAND_SYNC_REQUEST` | immediately publishes every telemetry packet |

Unknown commands are ignored after frame validation.

## CAN Telemetry To Display

Telemetry is sent to `CYCLEIQ_DISPLAY_CAN_ID`.

Scheduled telemetry:

| Packet | Period | Current source |
| --- | ---: | --- |
| `CYCLEIQ_TELEMETRY_LIVE` | 100 ms | speed in centi-km/h and signed watts |
| `CYCLEIQ_TELEMETRY_THERMALS` | 250 ms | motor and controller temperature in °C |
| `CYCLEIQ_TELEMETRY_BATTERY` | 500 ms | battery percentage and centivolts |
| `CYCLEIQ_TELEMETRY_STATE` | 1000 ms, or immediately on change | applied gear, modes, and confirmed Walk Mode state |

Initial periodic transmissions are staggered; a sync request bypasses those
offsets and emits state, live, thermals, and battery immediately.

The display implements Walk Mode as hold-to-run: send ON immediately when the
button is pressed, repeat ON every 250 ms while held, and send OFF immediately
on release or input cancellation. The 1000 ms ESC timeout remains the fail-safe
for a lost display, reset, or interrupted CAN connection. The display should use
the state telemetry, rather than its local button state, for its active-mode
indicator.

## Hardware Variants

The current cycleIQ hardware headers define:

- `cycleiq_mini`
  - one-wire PAS on PB11
  - torque sensor on `ADC_IND_EXT`
- `cycleiq_75_100`
  - `CYCLEIQ_HIGH_POWER`
  - two-wire PAS on PB11/PB10
  - torque sensor on `ADC_IND_EXT2`

The app also uses the motor temperature ADC input as a combined wheel speed and
motor NTC input.

## Known Incomplete Areas

These symbols or data paths exist but are not fully functional yet:

- VESC app configuration is ignored.
- Range is not estimated.
- Commands do not have an acknowledgement packet; the state telemetry confirms
  applied gear, modes, and Walk Mode.
- Battery-current limiting is implemented as conservative phase-current trim,
  not a tuned PID controller.
