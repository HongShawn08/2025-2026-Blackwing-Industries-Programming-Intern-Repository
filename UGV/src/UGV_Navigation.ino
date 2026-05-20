/*
 * UGV Navigation System
 * Hardware: ESP32 + DW3000 UWB Module
 */

#include "tag.h"
#include <ESP32Servo.h>
#include <math.h>

// ============================================
// PINS
// ============================================
const uint8_t ESC_LEFT  = 25;
const uint8_t ESC_RIGHT = 26;
const uint8_t PIN_RST   = 27;
const uint8_t PIN_IRQ   = 34;
const uint8_t PIN_SS    = 4;

// ============================================
// UWB CONFIGURATION
// ============================================
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8,
    DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8),
    DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
extern dwt_txconfig_t txconfig_options;

// ============================================
// NAVIGATION PARAMETERS
// Set TARGET_X/Y to the nest's position in the anchor coordinate frame
// ============================================
const double TARGET_X          = 2.0;
const double TARGET_Y          = 2.0;
const double ARRIVAL_TOLERANCE = 0.2;   // 20 cm
const int    BASE_SPEED        = 1700;  // µs (1500 = stopped, 2000 = full forward)
const int    MAX_TURN          = 200;   // µs of differential per unit of heading error
const double MIN_MOVE_M        = 0.15;  // min displacement (m) to update heading estimate

// ============================================
// GLOBALS
// ============================================
static Tag uwb_tag(0);

Servo escLeft;
Servo escRight;

bool   mission_complete  = false;
double prev_x            = -999;
double prev_y            = -999;
double estimated_heading = 0;
bool   heading_valid     = false;

// ============================================
// HELPERS
// ============================================
void stop_motors() {
    escLeft.writeMicroseconds(1500);
    escRight.writeMicroseconds(1500);
}

void drive(int left_us, int right_us) {
    escLeft.writeMicroseconds(constrain(left_us,  1000, 2000));
    escRight.writeMicroseconds(constrain(right_us, 1000, 2000));
}

// ============================================
// SETUP
// ============================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("UGV Navigation Starting...");

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(10);

    while (!dwt_checkidlerc()) {
        Serial.println("UWB idle check failed, retrying...");
        delay(500);
    }
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        Serial.println("UWB init failed — check wiring!");
        while (1);
    }
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    if (dwt_configure(&config)) {
        Serial.println("UWB config failed!");
        while (1);
    }
    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.print("Target: (");
    Serial.print(TARGET_X); Serial.print(", ");
    Serial.print(TARGET_Y); Serial.println(")");

    escLeft.attach(ESC_LEFT,  1000, 2000);
    escRight.attach(ESC_RIGHT, 1000, 2000);
    stop_motors();
    delay(2000);  // ESC arming delay

    Serial.println("Ready — waiting for UWB anchor lock...");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    uwb_tag.update();

    if (!uwb_tag.is_localizing() || mission_complete) return;

    double x = uwb_tag.get_x();
    double y = uwb_tag.get_y();

    double dx       = TARGET_X - x;
    double dy       = TARGET_Y - y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < ARRIVAL_TOLERANCE) {
        stop_motors();
        mission_complete = true;
        Serial.println("TARGET REACHED!");
        return;
    }

    double target_heading = atan2(dy, dx);

    // Derive heading from consecutive UWB position fixes.
    // Only update when the robot has moved enough for a reliable reading.
    if (prev_x > -999) {
        double moved_x = x - prev_x;
        double moved_y = y - prev_y;
        double moved   = sqrt(moved_x * moved_x + moved_y * moved_y);
        if (moved > MIN_MOVE_M) {
            estimated_heading = atan2(moved_y, moved_x);
            heading_valid     = true;
            prev_x = x;
            prev_y = y;
        }
    } else {
        prev_x = x;
        prev_y = y;
    }

    if (!heading_valid) {
        // Drive straight until we've moved enough to estimate heading
        drive(BASE_SPEED, BASE_SPEED);
        return;
    }

    // Heading error normalized to [-pi, pi]
    double heading_error = target_heading - estimated_heading;
    while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    // Proportional steering: positive error = target is to the left = speed up left motor
    double turn = constrain(heading_error * 1.5, -1.0, 1.0);
    drive(BASE_SPEED + (int)(turn * MAX_TURN),
          BASE_SPEED - (int)(turn * MAX_TURN));

    Serial.print("POS: ("); Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2);
    Serial.print(")  DIST: "); Serial.print(distance, 2);
    Serial.print("  HDG_ERR: "); Serial.println(heading_error, 3);
}
