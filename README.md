# 🤖 RoboHead Control System UI

A modern, responsive sci-fi mission control dashboard designed specifically for **Robot Head / Gimbal Pan & Tilt control** with real-time 3D feedback and multi-protocol Wi-Fi/ROS integration.

---

## 🚀 Features

- **Directional D-Pad Controls**:
  - **LOOK UP** (Arrow Up / `W`): Tilts the robot head upwards.
  - **LOOK DOWN** (Arrow Down / `S`): Tilts the robot head downwards.
  - **LOOK LEFT** (Arrow Left / `A`): Pans the robot head left.
  - **LOOK RIGHT** (Arrow Right / `D`): Pans the robot head right.
  - **CENTER** (Space): Instantly returns head to neutral position (90°, 90°).
- **Interactive 3D Robot Head**:
  - Real-time Three.js visualization that physically rotates and nods in 3D space matching the exact servo angles.
  - Interactive mouse dragging to orbit around the robot head.
- **Wi-Fi & Network Protocols**:
  - **WebSocket Mode** (Recommended for ESP32/ESP8266): High-speed bidirectional communication with latency ping measurement.
  - **ROS Bridge Mode**: Publishes geometry messages to `/robot/head_controller/command` or `/cmd_vel` for ROS 1 / ROS 2 robots.
  - **HTTP REST Mode**: Direct GET / POST requests for microcontrollers running basic HTTP webservers.
  - **Simulation Mode**: Test controls and terminal telemetry without needing physical hardware connected.
- **Motion Adjustments**:
  - Discrete step sizes: `1°`, `5°`, `10°`, `15°`
  - Hold-to-Continuous-Movement toggle with variable speed
  - Invert Pitch (Up/Down) & Invert Yaw (Left/Right) toggles
  - Precise Angle Sliders (0° - 180°)
- **Right-Side Speed & Pulse Control Center**:
  - **Master Pulse ON / OFF**: Big glowing toggle button with status orb to activate continuous periodic pulsing.
  - **One-Shot Pulse Button**: Trigger a single high-energy shockwave pulse on demand.
  - **Pulse Frequency Slider**: Adjust rhythm from 0.5 Hz (slow) to 2.5 Hz (fast).
  - **Pulse Waveform Monitor**: Real-time animated audio/energy visualizer bars.
  - **Pan/Tilt Speed Controls**: 1x to 10x slider + quick presets (🐢 Precision 2x, ⚡ Normal 5x, 🚀 Turbo 10x).
  - **Step Angle Increments**: 1°, 5°, 10°, 15°, 30° per click.
- **Autonomous Presets**:
  - ↕️ **Sweep Up/Down**: Continuous autonomous vertical tilt sweep (ideal for Pin 9 motor!).
  - ↔️ **Sweep Left/Right**: Continuous autonomous horizontal pan sweep.
  - 🤖 **Nod "Yes"**: Programmed vertical nod animation.
  - 🤖 **Shake "No"**: Programmed horizontal head shake animation.
  - ⚡ **Neck Pulse**: Runs dynamic hydraulic recoil and energy conduit pulse wave through the neck.
  - 🛑 **Emergency Stop**: Instantly halts all autonomous sweeps and transmissions.
- **ESP32-CAM Live Feed**:
  - Toggle between 3D Head Orientation and live MJPEG video stream with HUD crosshairs.

---

## 📂 Project Structure

```
d:\head movement\
├── index.html                                        # Main Mission Control UI
├── hand_track.html                                   # Dedicated 2-Column Hand Tracking Studio
├── hand_track.js                                     # Hand tracking engine & 2-column controller
├── style.css                                         # Dark Sci-Fi / Cyberpunk Cockpit Stylesheet
├── app.js                                            # Controller, Web Serial API, and 3D visualizer
├── arduino.ino                                       # 5-Servo Kinematic Controller (Arduino / ESP32)
├── esp32_testing_Blink__1__copy_20261113121855.ino  # Integrated Motor Firmware
└── README.md                                         # Documentation & Wiring Guide
```

---

## 🛠️ How to Connect & Run

1. **Flash Firmware**:
   - Open **`esp32_testing_Blink__1__copy_20261113121855.ino`** in the Arduino IDE.
   - Connect your motor signal wire to **Pin 9** (for UP / DOWN tilt).
   - Select your board & COM port, and click **Upload**.

2. **Launch Web UI**:
   - Open **`index.html`** in **Google Chrome** or **Microsoft Edge**.
   - Click the **⚡ CONNECT** button in the top right.
   - A browser popup will ask you to select your Arduino / ESP32 COM port.
   - Click your COM port and click **Connect**.

3. **Drive the Motor**:
   - Click **LOOK UP** (or press `W` / `↑`): Motor increments angle upwards!
   - Click **LOOK DOWN** (or press `S` / `↓`): Motor decrements angle downwards!
   - Drag the **Tilt Angle Slider**: Real-time smooth servo positioning.
   - Click **⚡ Neck Pulse**: Motor runs an animated mechanical pulse recoil.

---

## 🔌 Hardware Wiring (5-Servo Robot Configuration)

| Motor Function | Signal Wire | Arduino Pin | Power (VCC) | Ground (GND) |
|---|---|---|---|---|
| **Main Servo (Tilt UP / DOWN)** | Orange / Yellow | **PIN 9** | 5V | GND |
| **Third Servo (Pan LEFT / RIGHT / HOME)** | Orange / Yellow | **PIN 6** | 5V | GND |
| **Mouth Servo (MOUTH ON / OFF)** | Orange / Yellow | **PIN 3** | 5V | GND |
| **Pulse Servo 1 (Heartbeat SG90)** | Orange / Yellow | **PIN 2** | 5V | GND |
| **Pulse Servo 2 (Heartbeat SG90)** | Orange / Yellow | **PIN 12** | 5V | GND |

### 🎛️ Serial Command Reference (9600 Baud)
| Command | Action | Servo Target |
|---|---|---|
| `UP` | Main Servo Tilts Up | Pin 9 -> 140° |
| `DOWN` | Main Servo Tilts Down | Pin 9 -> 65° |
| `LEFT` | Third Servo Pans Left | Pin 6 -> 140° |
| `RIGHT` | Third Servo Pans Right | Pin 6 -> 50° |
| `HOME` or `CENTER` | Resets to Neutral | Pin 6 -> 90°, Pin 9 -> 90° |
| `MOUTH ON` | Activates Continuous Mouth Movement | Pin 3 sweeps 90° - 130° |
| `MOUTH OFF` | Stops Mouth Movement | Pin 3 holds position |
| `PULSE ON` | Activates Dual SG90 Heartbeat Pulse | Pins 2 & 12 pulse 0° - 20° |
| `PULSE OFF` | Deactivates Pulse | Pins 2 & 12 hold 0° |
| `ESTOP` | Emergency Stop | Halts all active loops & homes servos |
| `POS <pan> <tilt>` | Real-Time Hand Tracking / Dual Coordinate | Pin 6 -> <pan>°, Pin 9 -> <tilt>° |
| `TILT <angle>` | Direct Tilt Positioning (0-180°) | Pin 9 -> <angle>° |
| `PAN <angle>` | Direct Pan Positioning (0-180°) | Pin 6 -> <angle>° |
| `BLOW` | Wind Exhale Gesture & Pulse | Pin 3 sweep & Pins 2, 12 pulse |

> ⚠️ **Important**: Always power servos from an external 5V-6V power supply (2A-4A) and connect the ground (GND) of the external power supply to the Arduino GND.

---

## 📡 Protocol Payloads

### 1. WebSocket JSON Format
Sent from browser to ESP32:
```json
{
  "cmd": "LOOK",
  "action": "UP",
  "pan": 90,
  "tilt": 105,
  "relYaw": 0.0,
  "relPitch": 15.0,
  "timestamp": 1700000000000
}
```

### 2. ROS Bridge Mode
Sent to `ws://localhost:9090`:
```json
{
  "op": "publish",
  "topic": "/robot/head_controller/command",
  "msg": {
    "linear": { "x": 90, "y": 105, "z": 0 },
    "angular": { "x": 0.0, "y": 15.0, "z": 0 }
  }
}
```

### 3. HTTP Mode
Configurable template, e.g.:
`GET http://192.168.4.1/look?pan=90&tilt=105&dir=UP`

