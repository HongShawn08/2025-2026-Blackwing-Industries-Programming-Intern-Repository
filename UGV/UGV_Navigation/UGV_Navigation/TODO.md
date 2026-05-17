# UGV Navigation - Implementation Checklist

## Phase 1: UWB Position Extraction ✅ (Done by drone team)
- [x] UWB hardware setup
- [x] Anchor calibration code
- [x] Position calculation (trilateration)
- [x] Serial output

## Phase 2: Code Modifications (Your Work)

### Step 1: Expose Position Data
- [ ] **Modify `tag.h`**: Add public getters
  ```cpp
  class Tag {
  private:
      double current_x = 0.0;  // ADD
      double current_y = 0.0;  // ADD
  public:
      double get_x() { return current_x; }  // ADD
      double get_y() { return current_y; }  // ADD
      bool is_localizing() { return state == LOCALIZING; }  // ADD
  };
  ```

- [ ] **Modify `tag.cpp`**: Store position in localize() function
  ```cpp
  // Around line 161, after calculating x and y:
  this->current_x = x;
  this->current_y = y;
  ```

### Step 2: Basic Motor Control
- [ ] Wire motor driver to ESP32
- [ ] Test motor functions individually:
  - [ ] `drive_forward(speed)`
  - [ ] `turn_left(speed)`
  - [ ] `turn_right(speed)`
  - [ ] `stop_motors()`

### Step 3: Simple Navigation
- [ ] Calculate distance to target
  ```cpp
  double dx = TARGET_X - current_x;
  double dy = TARGET_Y - current_y;
  double distance = sqrt(dx*dx + dy*dy);
  ```
  
- [ ] Check if arrived
  ```cpp
  if (distance < ARRIVAL_TOLERANCE) {
      stop_motors();
      mission_complete = true;
  }
  ```

- [ ] Calculate heading to target
  ```cpp
  double heading_to_target = atan2(dy, dx);
  ```

- [ ] Drive in a straight line to target (test first)

### Step 4: Add Curved Path (Optional Enhancement)
- [ ] Create path offset generator
- [ ] Add perpendicular offset to heading
- [ ] Test curved vs straight path

## Phase 3: Integration Testing

### Hardware Tests:
- [ ] UWB module responds to commands
- [ ] All 3 anchors detected
- [ ] Position updates appear in Serial
- [ ] Position is accurate (manually verify)
- [ ] Motors respond to PWM commands
- [ ] Motors drive in correct directions

### Navigation Tests:
- [ ] UGV drives toward target in straight line
- [ ] UGV stops when reaching target
- [ ] UGV handles obstacles (if applicable)
- [ ] Curved path works (if implemented)

## Phase 4: Optimization & Polish

- [ ] Tune motor speeds for optimal performance
- [ ] Adjust arrival tolerance for your application
- [ ] Add emergency stop functionality
- [ ] Add status LED indicators
- [ ] Clean up Serial output
- [ ] Add telemetry (optional)

## Known Issues to Watch For

- [ ] Memory usage - ESP32 should be fine, but monitor
- [ ] Position accuracy - depends on anchor placement
- [ ] Motor power - ensure adequate power supply
- [ ] Timing - navigation loop should run fast enough

## Questions to Answer:

1. **What UWB library is the drone team using?**
   - Answer: _______________

2. **Where are the anchors positioned?**
   - Anchor 0: (_____, _____)
   - Anchor 1: (_____, _____)
   - Anchor 2: (_____, _____)

3. **Where is the nest (target)?**
   - Target: (_____, _____)

4. **What motor driver are you using?**
   - Model: _______________
   - Pins: _______________

5. **Do you have an ESP32 yet?**
   - [ ] Yes, we have one
   - [ ] No, need to order
   - [ ] Borrowing from drone team

## Resources

- Original repo: https://github.com/Blackbird-Industries/UWB
- Integration guide: `UWB_UGV_Integration_Guide.md`
- Arduino ESP32 docs: https://docs.espressif.com/projects/arduino-esp32/

---

**Start with Step 1** - everything else builds on having access to position data!
