# cycleIQ Current Working State

This document describes what the cycleIQ app currently does in firmware. It is
based on `applications/app_cycleiq.c`, the modules in this directory, and the
shared `external/cycleiq-protocol` SDK.

## Startup and Shutdown

`app_custom_start()` initializes the app in this order:

1. Resets cycleIQ runtime data.
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

`app_custom_stop()` stops the service thread first, then the sensor worker, then
forces motor current to 0 A, deinitializes PAS, unregisters CAN, and clears PAS
configuration.

`app_custom_configure()` currently ignores VESC app configuration.

## Main Service Loop

`service.c` runs the main `CYCLEIQ` thread every 100 ms. Each iteration:

1. Updates PAS and torque-sensor state.
2. Refreshes battery, current, controller temperature, and power fields.
3. Sends due CAN telemetry packets.
4. Computes and applies the motor current command.
5. Resets the VESC timeout watchdog.

On stop, the service thread calls `cycleiq_control_stop()` before exiting.

## Runtime Data

`data.c` owns the global `cycleiq_config` and `cycleiq_data` state.

Default configuration:

- max speed: 25 km/h
- battery internal resistance: 0.05 ohm
- wheel diameter: 0.66 m

Configurable fields are descriptor-driven in `data.c`. The descriptor table
defines the wire field ID, encoded scale, validation range, active config
offset, snapshot offset, and custom EEPROM address for each field. Startup loads
persisted config from the custom EEPROM area only when the stored magic/version
match and every stored field validates; otherwise firmware defaults remain
active.

Display config writes are staged. Single-field writes and full-snapshot writes
update the staged copy only, and snapshot writes validate every field before
applying any of them. `CYCLEIQ_CONFIG_OP_COMMIT` copies the staged config into
the active config and persists it once. `CYCLEIQ_CONFIG_OP_DISCARD` resets staged
config from active config.

Data initialization resets transient values and starts in:

- main display screen
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
- motor power as battery current times battery voltage, clamped to `uint16_t`
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

The app validates incoming setters for gear, support mode, ride mode, and
screen before changing global state.

## Assist Control

`control.c` converts the active support state into a motor phase-current command.
Each gear has a battery-current budget and a derived low-speed phase-current
ceiling. The phase-current ceiling is calculated as:

```text
phase_current_limit_a = battery_current_limit_a / 0.33
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

Assist is tapered over the final 2 km/h before `max_speed_kph` and reaches 0 A
at or above the configured maximum speed.

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

- ramp up: 12 A/s
- normal ramp down: 40 A/s
- fast release to zero: 80 A/s

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
- Runtime torque voltage is low-pass filtered each service loop.
- Torque sensor active means voltage is at or above the calibrated minimum.
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

The shared protocol uses:

- display node ID: `PEAK_CAN_ID` / `0x6A`
- ESC node ID: `CYCLEIQ_CAN_ID` / `0x6B`
- extended CAN ID high byte as destination node
- extended CAN ID low byte as command or telemetry type
- big-endian multi-byte payload fields

The ESC accepts only frames addressed to `CYCLEIQ_CAN_ID`.

Implemented commands:

| Command | Behavior |
| --- | --- |
| `CYCLEIQ_POWER_OFF` | disables motor output and immediately commands 0 A |
| `CYCLEIQ_POWER_ON` | enables motor output |
| `CYCLEIQ_COMM_GEAR_SET` | reads one byte and sets gear if valid |
| `CYCLEIQ_COMM_MODE_SET` | reads one byte and sets PAS/torque/hybrid mode if valid |
| `CYCLEIQ_COMM_RIDE_MODE_SET` | reads one byte and sets normal/mountain mode if valid |
| `CYCLEIQ_COMM_SCREEN_SET` | reads one byte and sets display screen state if valid |
| `CYCLEIQ_COMM_CONFIG_GET` | replies with a config snapshot or a single config field |
| `CYCLEIQ_COMM_CONFIG_SET` | stages config field/snapshot updates, commits staged config, or discards staged config |
| `CYCLEIQ_COMM_PROTOCOL_VERSION_GET` | replies with protocol and SDK versions |

Unknown commands are ignored after frame validation.

Config set commands reply with config ACK/error telemetry. Config get commands
reply with config field/snapshot telemetry, or config ACK/error telemetry when a
request is malformed or references an unknown field.

## CAN Telemetry To Display

Telemetry is sent to `PEAK_CAN_ID`.

Scheduled telemetry:

| Packet | Period | Current source |
| --- | ---: | --- |
| live status | 100 ms | speed and motor power |
| battery status | 500 ms | battery percent, voltage, battery current |
| motor status | 250 ms | motor temp, controller temp, motor current, motor RPM |
| controller state | 1000 ms, or immediately on change | gear, support mode, ride mode |
| battery energy | 1000 ms | watt-hours and amp-hours |
| trip primary | 1000 ms | trip distance and trip time |
| trip secondary | 1000 ms | average speed and range |
| config field | on request | one encoded config field |
| config snapshot | on request | max speed, battery resistance, wheel diameter |
| config ACK/error | on config set/error | command, status, detail |

Initial transmit offsets are staggered so the first service loop does not emit
all slower packets at once.

Current telemetry limitations:

- range is initialized but not estimated.
- controller state does not include `motor_enabled`.

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
- Hybrid support mode outputs 0 A.
- Range is not estimated.
- Non-config incoming commands are accepted without explicit acknowledgements or
  error telemetry.
- Battery-current limiting is implemented as conservative phase-current trim,
  not a tuned PID controller.
