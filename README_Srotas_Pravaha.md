# Srotas Pravaha — Lam Research Challenge 3.0, Stage 2

An ESP32-based liquid handling controller for the Lam Research Challenge 3.0 Stage 2 practical engineering challenge. The system implements automatic sump-level protection, flow monitoring and totalization, a controlled transition to flush mode, and automatic shutdown when near-zero flow indicates that the flush supply may be exhausted.

## Project Objectives

The PS asks the team to build a recirculating liquid-flow arrangement, manage changing sump level, measure and totalize flow through FT-01, investigate flow response, and automatically stop the system when the flush-water supply is exhausted.

Srotas Pravaha's controller addresses the main operational requirements through a state machine, a float-switch interlock, flow telemetry, phase-wise volume estimates, and a sustained low-flow shutdown criterion.

## Operating Sequence

### Case 1 — Process circulation and level protection

- Pump 1 runs at 50% PWM and Pump 2 at 100%; Pump 3 remains off.
- The solenoid valve is commanded open.
- When the sump float switch indicates a high level, Pump 1 stops while Pump 2 continues draining the sump. Pump 1 may resume after the float input remains clear for the configured recovery delay.
- The controller samples flow internally once per second and reports telemetry at approximately five-second intervals.
- At the end of the configured Case 1 duration, it reports the estimated average flow and estimated volume passing FT-01 before moving to the sump-drain transition.

**Current code setting:** `CASE1_DURATION_MS = 60000UL`, which is 60 seconds (1 minute). Change this to `180000UL` for a 3-minute Case 1 run if that is the duration required for your demonstration.

### Sump-drain transition

The controller commands the solenoid closed, stops Pumps 1 and 3, and keeps Pump 2 at 100% to drain the sump. Case 2 begins after the configured settling delay when the float interlock indicates that the sump level has recovered. A timeout stops the sequence in a fault state if the transition does not complete.

### Case 2 — Flushing and automatic shutdown

- The solenoid remains commanded closed to isolate the process-fluid tank.
- Pumps 1, 2 and 3 are commanded to 100% when permitted by the sump-level interlock.
- If the sump float indicates high level, Pumps 1 and 3 are paused while Pump 2 continues operating.
- After flow has been established and the minimum Case 2 run time has elapsed, the controller checks for estimated flow at or below `CASE2_NEAR_ZERO_FLOW_LPM` for the configured continuous hold time.
- If near-zero flow persists for six seconds, the controller stops all pumps, closes the solenoid, and reports completion. A six-minute Case 2 maximum provides a fallback shutdown; reaching that maximum is reported separately and is not claimed as confirmed flush exhaustion.

## Flow Measurement and Calibration

The current code uses the calibration trendline shown in the supplied graph:

```text
Q (L/min) = 0.0112 × f (Hz) + 0.0366
R² = 0.9917
```

Here, `f` is the pulse frequency measured from FT-01. The controller derives frequency from pulse counts over each sampling interval, estimates flow in L/min, and integrates flow over time to estimate throughput for Case 1, the transition, and Case 2.

The code treats a sample with no pulses as zero flow. Below the lowest supplied calibration frequency, it uses a proportional low-frequency extension so the estimated flow can approach zero. This extension is an engineering approximation, not a calibrated measurement.

**Calibration caveat:** The equation and R² shown in the graph conflict with the `Q fit` values previously supplied in the calibration table. The current code deliberately uses the graph trendline. Confirm the correct regression before relying on absolute flow or volume values. Calibration measurements cover approximately 202.65–254.30 Hz, so results outside that range are extrapolations. The estimated volume is flow-through at FT-01; in a recirculating system, it is not necessarily the net decrease in the 10 L tank's contents.

## Serial Monitor Commands

Open Serial Monitor at **115200 baud** and select a line ending such as **Newline**.

| Command | Action |
|---|---|
| `s` | Start the automatic Case 1 → transition → Case 2 sequence |
| `e` | Emergency software stop: command all pumps off and solenoid closed |
| `r` | Reset a latched FAULT state after checking and resolving the cause |
| `t1` | Run Pump 1 at 50% for 3 seconds while idle |
| `t2` | Run Pump 2 at 50% for 3 seconds while idle |
| `t3` | Run Pump 3 at 50% for 3 seconds while idle |
| `v` | Open the solenoid for a 3-second diagnostic test while idle |
| `summary` | Print accumulated flow and volume estimates when stopped |
| `?` | Display the command help |

The `e` command is a software stop, not a safety-rated emergency stop. Keep a physical means of disconnecting pump power available. Following an emergency stop, inspect the setup before restarting.

## Hardware and GPIO Map

| Function | ESP32 GPIO |
|---|---:|
| Physical switch input (diagnostic only in this version) | 34 |
| Float switch input | 35 |
| FT-01 pulse signal | 14 |
| Solenoid valve control | 27 |
| Pump 1 R_PWM / L_PWM | 33 / 32 |
| Pump 1 R_EN / L_EN | 26 / 13 |
| Pump 2 R_PWM / L_PWM | 19 / 18 |
| Pump 2 R_EN / L_EN | 17 / 16 |
| Pump 3 R_PWM / L_PWM | 25 / 23 |
| Pump 3 R_EN / L_EN | 22 / 21 |

The source supports the ESP32 Arduino core 2.x and 3.x PWM API variants.

## Why This Approach Fits the PS Requirements

- **Automatic level protection:** The float input controls the relevant pump interlock while maintaining Pump 2 operation to help clear a high sump level.
- **Two-stage sequence:** Case 1, sump drainage, and Case 2 execute in one controller-managed sequence, without requiring a manual mode change between cases.
- **Flush isolation and shutdown:** The solenoid is commanded closed for the transition and Case 2, and sustained near-zero flow can trigger automatic shutdown.
- **Measured trend and totalization:** The Serial Monitor reports flow trends and estimated phase-wise throughput, supporting the requested flow-versus-time evidence.
- **Fault containment:** Transition and Case 2 time limits stop the system rather than allowing it to run indefinitely when expected conditions do not occur.
- **Traceable operation:** The state name, pump duties, solenoid state, float state, pulse frequency and estimated flow appear in serial telemetry.

These are control-design advantages, not a claim that every PS item is independently validated. In particular, the PS asks for experimental flow calibration, characterization of valve-position response and maintenance of the nominal 3 L/min target. Flow and volume figures from this code remain dependent on resolving the calibration discrepancy and verifying the installed hardware.

## Build and Run

1. Open the `.ino` sketch in Arduino IDE.
2. Select the appropriate ESP32 board and COM port.
3. Verify the GPIO map, pump direction, driver-enable polarity, solenoid polarity, and float-switch polarity against the physical PCB.
4. Confirm that the FT-01 signal reaching GPIO14 is safe for the ESP32's 3.3 V input.
5. With the hydraulic path filled and the manual flush-line ball valve prepared according to the PS procedure, open Serial Monitor at 115200 baud.
6. Send `s` to run the sequence. Send `e` to command the outputs off at any time.

## Safety and Validation Notes

- GPIO35 is input-only and has no internal pull-up or pull-down; the float-switch circuit must provide a stable, electrically appropriate input.
- Do not run a pump against a closed or blocked hydraulic path. Confirm the system is primed and the tubing and fittings are secure.
- Some flow-sensor interfaces may produce signals above 3.3 V. Do not connect a 5 V pulse signal directly to an ESP32 GPIO.
- Serial commands depend on the ESP32 firmware and communication link. Use an independent physical power-disconnect method for emergencies.
- Validate the program on the actual ESP32 board package and test the pump, solenoid, float switch and flow sensor individually before a full powered run.

## Files

- `LAM_Research_ESP32_Controller.ino` — ESP32 controller sketch (use your final project filename if it differs).
- `README.md` — project overview, operating sequence, pin map and validation notes.

## Team

**Team Name:** Srotas Pravaha  
**Challenge:** Lam Research Challenge 3.0 — Stage 2 Practical Engineering Challenge
