# Robotic Hand FPGA Controller — Design Notes

## System Overview

A 3D-printed robotic hand controlled by MediaPipe video tracking. A friend handles the vision/MediaPipe side and sends data over UART. The Basys3 FPGA (Artix-7 XC7A35T, 100MHz) receives the data, runs a Kalman filter for motion smoothing, drives 8 MG996R servos via PWM, and reads servo position feedback via an AD7124-8 ADC.

### Signal Chain

```
MediaPipe (friend's code)
    |
    | UART 115200 baud
    v
Basys3 FPGA
    |
    |-- 8x PWM (3.3V) --> ADUM1400 isolators --> 5V PWM --> MG996R servos
    |
    |-- SPI (3.3V) --> MAX14850 isolator --> AD7124-8 ADC <-- servo feedback (analog)
    |
    |-- UART TX (readback to friend's system for charts/analysis)
```

### Servo Mapping (8 servos)

| Servo | Joint | PWM Pin |
|-------|-------|---------|
| 0 | Pinky | JA0 |
| 1 | Ring | JA1 |
| 2 | Middle | JA2 |
| 3 | Index | JA3 |
| 4 | Thumb (palm movement) | JB0 |
| 5 | Thumb (finger tension) | JB1 |
| 6 | Wrist rotation | JB2 |
| 7 | Elbow extension | JB3 |

### Hardware Components

- **Basys3** — FPGA, powered by 5V USB, 3.3V logic
- **2x ADUM1400** — digital isolators, level-shift PWM from 3.3V to 5V
- **DC2468A** — 12V to 5V step-down for servo power (8A)
- **ADP165** — 5V to 3.3V LDO for ADC-side logic
- **AD7124-8** — 24-bit sigma-delta ADC, reads 8 servo feedback channels
- **MAX14850** — SPI isolator between FPGA and ADC
- **MG996R** — high-torque servos with analog feedback pin

---

## Communication Protocol

10-byte UART frames at 115200 baud, 8N1:

```
| Byte 0      | Bytes 1-8         | Byte 9   |
|-------------|-------------------|----------|
| Direction   | ServoAngle 1..8   | Checksum |
```

- **Direction = 0xFF**: write angles to servos
- **Direction = 0xFE**: read current angles back (FPGA responds with a frame)
- **Angles**: 0-180 degrees, one byte per servo, pinky to elbow
- **Checksum**: XOR of bytes 0-8. Receiver XORs all 10 bytes — result must be 0x00 for valid frame

### Why XOR checksum?

The sender computes `checksum = byte0 XOR byte1 XOR ... XOR byte8` and puts it in byte 9. On the receiver: `byte0 XOR byte1 XOR ... XOR byte9` = anything XOR'd with itself cancels to zero. If any bit flipped during transfer, the result is non-zero and we drop the frame.

---

## Module Descriptions

### system_tick_gen.vhd

Generates a 20ms tick pulse from the 100MHz clock. A 21-bit counter counts 0 to 1,999,999 (2,000,000 counts = 20ms at 100MHz). Outputs a single-cycle pulse every time it wraps.

### pwm_gen.vhd

Generates one PWM channel for one servo. Standard servo PWM:
- 20ms period (50Hz)
- 1.0ms pulse = 0 degrees
- 2.0ms pulse = 180 degrees

At 100MHz: 1ms = 100,000 counts, 2ms = 200,000 counts. The threshold is computed as:

```
threshold = 100,000 + angle * 556
```

Where 556 ≈ 100,000 / 180. The counter runs 0 to 1,999,999. Output is high when counter < threshold. Eight instances are generated in the top level.

### uart_rx.vhd

UART receiver at 115200 baud.

**Baud rate math**: 100,000,000 / 115,200 = 868 clocks per bit. `CLKS_PER_BIT = 867` (zero-indexed).

**HALF_BIT = 433**: after detecting the start bit falling edge, we wait half a bit period to land in the middle of the bit, then sample every 868 clocks. Sampling mid-bit avoids edge glitches.

**Metastability protection**: the external RX signal is asynchronous to our clock. Two flip-flops in series (`rx_sync1`, `rx_sync2`) ensure the signal settles before our logic reads it. Initialized to '1' because UART idle is high.

**shift_reg**: the received byte is assembled bit by bit (LSB first per UART standard). After 8 data bits, the stop bit is waited through (it's a real bit on the wire), then `rx_valid` pulses and `rx_data` outputs the complete byte.

### uart_tx.vhd

Mirror of uart_rx. On `tx_start`, it sends: start bit (low for 868 clocks) → 8 data bits (868 clocks each, LSB first) → stop bit (high for 868 clocks).

### frame_rx.vhd

Assembles 10-byte frames from individual UART bytes:

1. **S_WAIT_DIR** — waits for 0xFF or 0xFE
2. **S_RECV_DATA** — collects 8 angle bytes, XOR-accumulating checksum
3. **S_RECV_CHKSUM** — receives checksum byte, verifies XOR of all 10 bytes = 0x00

Has a 1ms timeout (100,000 clocks) — if a frame isn't completed in time, resets to S_WAIT_DIR. This prevents getting stuck if bytes are lost.

### frame_tx.vhd

When direction = 0xFE (read request), serializes a response frame: 0xFE + 8 current angles + XOR checksum. Sends bytes one at a time through uart_tx, waiting for tx_busy to clear between bytes.

### mailbox.vhd

Double-buffer (ping-pong) that decouples UART timing from the 20ms servo cycle. UART deposits new angles whenever a valid frame arrives. On each 20ms tick, the latest valid angles are latched for the servo cycle. If no new frame arrived, the previous angles are held — the hand maintains its last position.

### source_mux.vhd

Combinational mux: selects between mailbox angles (UART input) and demo_rom angles based on sw0 (switch 0 on Basys3).

### demo_rom.vhd

Pre-programmed hand movement sequences stored as keyframes in ROM. Each keyframe = 8 angles + hold duration (in 20ms ticks). Sequences include:
- Open/close fist
- Wave (wrist rotation)
- Individual finger curls
- Thumbs up
- Pointing
- Elbow flex

Activated by sw0. Loops through all keyframes continuously.

### spi_master.vhd

Generic SPI master, mode 3 (CPOL=1, CPHA=1):
- SCLK idles high
- Data changed on falling edge
- Data sampled on rising edge
- SCLK = 100MHz / 20 = 5MHz

Metastability sync on MISO input. 8-bit transfers, MSB first.

### ad7124_ctrl.vhd

Controls the AD7124-8 ADC. Two phases:

**Init sequence** (one-time after reset):
1. Reset: 8 bytes of 0xFF
2. Wait 4ms
3. Configure ADC_Control: full power, continuous conversion, DATA_STATUS enabled, internal reference
4. Configure Setup 0: unipolar, input buffers, internal reference
5. Filter 0: sinc3 filter, 50Hz output data rate (matches 20ms cycle)
6. Configure channels 0-7: each maps AIN_N to AVSS (single-ended)

**Cyclic read** (continuous after init):
- Sends read command for data register
- Reads 3 data bytes + 1 status byte (status tells which channel)
- Converts raw 24-bit ADC to 0-180 degree angle internally: `angle = (adc_raw * 180) >> 24`
- After all 8 channels read, pulses `meas_valid`

Register addresses and bit definitions from the no-OS driver at `~/no-OS/drivers/adc/ad7124/ad7124.h`.

### kalman_engine.vhd

The core of the thesis. 2-state Kalman filter in Q16.16 fixed-point, time-multiplexed over 8 servo channels.

---

## Kalman Filter — Detailed Explanation

### What It Does

The Kalman filter estimates the true servo position by blending two information sources:
1. **Prediction**: based on physics (position + velocity × time)
2. **Measurement**: from the ADC reading the servo's feedback pin

Neither source is perfect — prediction drifts over time, measurements are noisy. The Kalman filter optimally combines them.

### State Model

Two variables per servo:
- `x0` = estimated position (degrees)
- `x1` = estimated velocity (degrees/second)

### Q16.16 Fixed-Point

Every number is a 32-bit signed integer where the lower 16 bits are fractional:
- `65536` = 1.0
- `1311` = 0.02 (our dt = 20ms)
- `655360` = 10.0

When multiplying two Q16.16 numbers, the result is 64-bit with 32 fractional bits. We take bits 47:16 (shift right by 16) to get Q16.16 back.

**Why not Q8.8?** Because 180 degrees in Q8.8 = 180 × 256 = 46,080 which overflows signed 16-bit (max 32,767). Q16.16 handles it easily.

### Constants (Tuning Knobs)

| Constant | Value | Real Value | Meaning |
|----------|-------|------------|---------|
| ONE_Q16 | 65536 | 1.0 | Fixed-point representation of 1 |
| DT_Q16 | 1311 | 0.02s | Update period (20ms) |
| Q00 | 655 | 0.01 deg² | Process noise for position — how much we distrust our prediction |
| Q11 | 6554 | 0.1 (deg/s)² | Process noise for velocity |
| R_NOISE | 65536 | 1.0 deg² | Measurement noise — how much we distrust the ADC |
| P_INIT | 655360 | 10.0 | Initial uncertainty (high = "I don't know yet") |

**Tuning**: Small Q = trust prediction more (smoother, slower response). Small R = trust measurement more (responsive, noisier). These can be adjusted when testing with real hardware.

### State RAM

8 servos × 5 values each = 40 words of 32-bit RAM:

| Value | What it is |
|-------|-----------|
| x0 | Estimated position |
| x1 | Estimated velocity |
| P00 | Uncertainty in position |
| P01 | Correlation between position and velocity uncertainty |
| P11 | Uncertainty in velocity |

The filter processes one servo at a time: load 5 values → compute → store back → next servo.

### FSM Steps (every 20ms)

#### 1. S_INIT_RAM (once at power-up)
Sets all positions/velocities to 0, covariance diagonals (P00, P11) to P_INIT = 10.0 (high initial uncertainty).

#### 2. S_LOAD
Reads the 5 state values for the current servo from RAM. Converts the 8-bit measured angle from ADC to Q16.16 (multiply by 65536).

#### 3. PREDICT — "What do we think will happen?"

```
x0_pred = x0 + dt × x1      -- new position = old + velocity × time
x1_pred = x1                 -- velocity unchanged in prediction
```

Example: servo at 90° moving at 100°/s → after 20ms predicts 90 + 0.02×100 = 92°.

Velocity doesn't change during prediction because we only model position and velocity (no acceleration). **But velocity does change during the update step** — see below.

Covariance prediction (uncertainty grows):
```
pp00 = p00 + 2×dt×p01 + dt²×p11 + Q00
pp01 = p01 + dt×p11
pp11 = p11 + Q11
```

#### 4. UPDATE — "Correct using the ADC measurement"

**Innovation** (the "surprise"):
```
y_innov = z_meas - x0_pred   -- how far off was our prediction?
```

**Innovation covariance** (total uncertainty):
```
s_val = pp00 + R              -- prediction uncertainty + measurement noise
```

**Kalman gain** (how much to trust the measurement):
```
k0 = pp00 / s_val             -- gain for position correction
k1 = pp01 / s_val             -- gain for velocity correction
```

If prediction uncertainty (pp00) is large relative to measurement noise (R), the gain is close to 1.0 — trust the measurement. If pp00 is small, gain is near 0 — trust the prediction.

**State correction**:
```
x0 = x0_pred + k0 × y_innov  -- corrected position
x1 = x1_pred + k1 × y_innov  -- corrected velocity
```

This is where velocity learns. If the measurement consistently shows the servo moved more than predicted, y_innov is positive, and k1 × y_innov increases the velocity estimate over time.

**Covariance update** (uncertainty shrinks after measurement):
```
p00 = (1 - k0) × pp00
p01 = (1 - k0) × pp01
p11 = -k1 × pp01 + pp11
```

#### 5. S_STORE
Writes the 5 updated values back to RAM. Converts Q16.16 position to 8-bit angle (shift right 16, clamp 0-180). Stores in output register.

#### 6. S_NEXT_CH / S_OUTPUT
Moves to next servo (0 through 7). After all 8 are processed, pushes all filtered angles to the PWM generators and returns to S_IDLE.

### Timing Budget

Per servo: ~14 multiplies + 2 divisions + 14 adds ≈ 182 clock cycles.
All 8 servos: ~1,456 cycles out of 2,000,000 available per 20ms tick = **0.07% utilization**.

### 3-State Upgrade Path

To add acceleration as a third state variable:
- State becomes `[position, velocity, acceleration]`
- P matrix grows from 2×2 to 3×3 (6 unique entries since symmetric)
- The F matrix dt²/2 term (0.0002) is too small for Q16.16, so use Q4.28 for that constant
- Computation grows to ~45 multiplies + 3 divisions per channel, still well within timing budget

---

## Pin Assignments

All Basys3 I/O pins are included in the design and constraint file. Unused pins are actively driven to safe states to prevent floating inputs and unnecessary current consumption.

### Active Pins

| Port | Basys3 Pin | Function |
|------|-----------|----------|
| clk | W5 | 100MHz oscillator |
| sw0 | V17 | Demo mode switch |
| btnC | U18 | Reset (center button) |
| RsRx | B18 | UART receive |
| RsTx | A18 | UART transmit |
| JA0-JA3 | J1, L2, J2, G2 | PWM servos 0-3 (pinky, ring, middle, index) |
| JB0-JB3 | A14, A16, B15, B16 | PWM servos 4-7 (thumb-palm, thumb-tension, wrist, elbow) |
| JC0 (CS) | K17 | SPI chip select to AD7124 |
| JC1 (MOSI) | M18 | SPI data out |
| JC2 (MISO) | N17 | SPI data in |
| JC3 (SCLK) | P18 | SPI clock |
| led[0:15] | various | Status indicators |

### Unused Pin Tie-Offs

| Port | Drive State | Reason |
|------|------------|--------|
| JA4-JA7 | '0' | Pmod upper half, driven low |
| JB4-JB7 | '0' | Pmod upper half, driven low |
| JC4-JC7 | '0' | Pmod upper half, driven low |
| seg[6:0], dp | '1' | 7-segment active-low, all segments off |
| an[3:0] | '1' | 7-segment anodes active-low, all digits off |
| vgaRed/Green/Blue | '0' | VGA outputs driven low |
| Hsync, Vsync | '0' | VGA sync driven low |
| PS2Clk, PS2Data | 'Z' | PS/2 high-impedance (open-drain bus) |
| JXADC[7:0] | 'Z' | Analog header high-impedance |
| sw[15:1] | input (read) | Unused switches, active as inputs |
| btnU/D/L/R | input (read) | Unused buttons, active as inputs |

### LED Indicators

| LED | Signal |
|-----|--------|
| 0 | ADC config done |
| 1 | ADC measurement valid |
| 2 | UART frame received |
| 3 | Checksum error |
| 4 | Kalman engine busy |
| 5 | Demo mode active (mirrors sw0) |
| 6 | UART TX busy |
| 7 | SPI busy |
| 8-15 | Servo 0 filtered angle (8-bit) |

---

## Resource Utilization (XC7A35T)

| Resource | Available | Estimated Usage | % |
|----------|-----------|-----------------|---|
| LUTs | 20,800 | ~3,000 | 14% |
| Flip-Flops | 41,600 | ~2,500 | 6% |
| BRAM36 | 50 | 2 | 4% |
| DSP48E1 | 90 | 2-4 | 4% |

---

## Building and Flashing

1. Open Vivado, create RTL project targeting `xc7a35tcpg236-1`
2. Add all `.vhd` files and `basys3_servo.xdc`
3. Set `basys3_top` as top module
4. If synthesis/implementation fails with spawn error, use Tcl console:
   ```tcl
   reset_run synth_1
   launch_runs synth_1 -jobs 1
   wait_on_run synth_1
   launch_runs impl_1 -jobs 1
   wait_on_run impl_1
   launch_runs impl_1 -to_step write_bitstream -jobs 1
   wait_on_run impl_1
   ```
5. Open Hardware Manager → Open Target → Auto Connect → Program Device

If `hw_server` is locked: close all Vivado instances, kill `hw_server.exe` and `cs_server.exe` in Task Manager, reconnect.

Digilent USB drivers: run `C:\Xilinx\Vivado\2025.2\data\xicom\cable_drivers\nt64\install_digilent.exe` as administrator if the board isn't detected.
