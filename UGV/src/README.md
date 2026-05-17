# UGV Navigation System

Autonomous navigation system for UGV (Unmanned Ground Vehicle) using UWB (Ultra-Wideband) positioning.

## 📁 Project Structure

```
UGV_Navigation/
├── UGV_Navigation.ino    ← Main Arduino sketch (START HERE)
├── tag.h                 ← Tag class definition
├── tag.cpp               ← UWB positioning logic
├── uwb-node.h            ← Low-level UWB communication
├── uwb-node.cpp          ← UWB node implementation
├── config.h              ← System configuration
├── states.h              ← State machine definitions
├── scheduler.h           ← Timing scheduler
├── scheduler.cpp         ← Scheduler implementation
├── variance.h            ← Variance calculations
└── prng.h                ← Pseudo-random number generator
```

## 🔧 Hardware Requirements

### Required:
- **ESP32 Dev Board** (30-38 pin variant)
- **DW3000 UWB Module** (x1 for UGV tag)
- **3x UWB Anchor Modules** (stationary beacons - get from drone team)
- **Motor Driver** (L298N, TB6612FNG, or similar)
- **BLDC Motors** (with tank treads)
- **Power Supply** (separate for logic and motors)

### Wiring:

#### UWB Module → ESP32:
```
DW3000          ESP32
--------        -----
RST      →      GPIO 27
IRQ      →      GPIO 34 (input only)
CS/SS    →      GPIO 4
MOSI     →      GPIO 23 (default SPI)
MISO     →      GPIO 19 (default SPI)
SCK      →      GPIO 18 (default SPI)
VCC      →      3.3V
GND      →      GND
```

#### Motors → ESP32 (Example for L298N):
```
L298N           ESP32
--------        -----
IN1      →      GPIO 25 (Left Forward)
IN2      →      GPIO 26 (Left Reverse)
IN3      →      GPIO 32 (Right Forward)
IN4      →      GPIO 33 (Right Reverse)
ENA      →      3.3V or PWM pin
ENB      →      3.3V or PWM pin
```

## 🚀 Getting Started

### 1. Install Arduino IDE & Libraries

1. Install Arduino IDE 2.x
2. Add ESP32 board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

3. Install required libraries:
   - **DW3000 Arduino Library** (check with drone team for their specific library)
   - May need: `DW1000` or custom BlackBird library

### 2. Configure the Code

Open `UGV_Navigation.ino` and update:

```cpp
// Set your target coordinates (nest location)
const double TARGET_X = 2.0;  // meters
const double TARGET_Y = 2.0;  // meters

// Set motor pins if different from defaults
const uint8_t MOTOR_LEFT_FWD = 25;
// ... etc
```

### 3. Upload & Test

1. Connect ESP32 via USB
2. Tools → Board → ESP32 Dev Module
3. Tools → Port → (select your ESP32)
4. Upload ✓
5. Open Serial Monitor (115200 baud)

## 📊 Expected Serial Output

```
========================================
UGV Navigation System Starting...
========================================
Motor pins initialized
Initializing UWB module...
UWB idle check passed
UWB initialized
UWB configured
========================================
UWB Ready!
Waiting for position lock...
Target: (2.00, 2.00)
========================================
Awaiting connections... 0/3
Found anchor 0
Awaiting connections... 1/3
Found anchor 1
Awaiting connections... 2/3
Found anchor 2
ROUND 0
Distance 0-1: 2.345
Distance 0-2: 1.987
Distance 1-2: 2.123
...
Calibration complete!
POS: 1.234, 0.567
POS: 1.245, 0.578
POS: 1.256, 0.589
```

## 🎯 Current Status

### ✅ Implemented:
- UWB positioning system
- Anchor calibration
- Position calculation (trilateration)
- Serial output of position

### ⚠️ TODO (Your Next Steps):
1. **Add position getters to Tag class** (see tag.h, tag.cpp)
2. **Implement navigation logic** (calculate heading to target)
3. **Add motor control** (tank drive based on heading error)
4. **Add curved path generation** (sine wave offset)
5. **Test with actual hardware**

## 🔍 How the System Works

### Phase 1: Calibration (Automatic)
- Tag finds 3 anchors
- Measures distances between anchors
- Calculates anchor geometry
- Takes ~5-10 seconds

### Phase 2: Localization (Continuous)
- Tag polls each anchor for distance
- Calculates position via trilateration
- Updates ~100 times per second
- Outputs: `POS: x, y`

### Phase 3: Navigation (To Be Implemented)
- Read current position (x, y)
- Calculate heading to target
- Add curved path offset
- Drive motors based on heading error

## 🐛 Troubleshooting

### "IDLE FAILED"
- **Problem:** UWB module not responding
- **Fix:** Check SPI wiring (MOSI, MISO, SCK, CS)

### "INIT FAILED"
- **Problem:** UWB initialization error
- **Fix:** Check power supply (3.3V, sufficient current)

### Never finds anchors
- **Problem:** Anchors not powered or out of range
- **Fix:** Verify anchors are running, check range (<50m)

### Position jumps around
- **Problem:** Poor signal or multipath interference
- **Fix:** Ensure line-of-sight, avoid metal obstacles

## 📚 Next Steps

1. Read the integration guide: `UWB_UGV_Integration_Guide.md`
2. Modify `tag.h` to add position getters
3. Implement navigation algorithm
4. Test incrementally (UWB first, then motors, then navigation)

## 🤝 Getting Help

- Check with the drone team for UWB library specifics
- Reference the original repo: https://github.com/Blackbird-Industries/UWB
- Test UWB positioning first before adding navigation

---

**Original UWB code by BlackBird Industries**
**UGV adaptation by [Your Name]**
