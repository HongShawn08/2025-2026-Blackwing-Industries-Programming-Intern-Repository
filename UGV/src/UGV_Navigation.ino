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
const uint8_t ESC_LEFT   = 25;
const uint8_t ESC_RIGHT  = 26;
const uint8_t PIN_RST    = 27;
const uint8_t PIN_IRQ    = 34;
const uint8_t PIN_SS     = 4;
const uint8_t CLAW_PIN   = 18;
const uint8_t BUTTON_PIN = 0;   // BOOT/FLASH button

// ============================================
// CLAW PARAMETERS
// ============================================
const int CLAW_OPEN   = 1000;  // µs — adjust if needed
const int CLAW_CLOSED = 2000;  // µs — adjust if needed

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
// MISSION PARAMETERS  — UPDATE DAY-OF
// ============================================
const double TARGET_X          = 2.0;    // opponent nest X (meters)
const double TARGET_Y          = 2.0;    // opponent nest Y (meters)
const double ARRIVAL_TOLERANCE = 0.2;    // 20 cm
const int    BASE_SPEED        = 1700;   // µs forward
const int    REVERSE_SPEED     = 1300;   // µs reverse
const double MIN_MOVE_M        = 0.15;
const double HEADING_THRESHOLD = 0.2;    // radians (~11°)

// Tune these on the field before competition:
//   STRAIGHT_DRIVE_TIME: run once, measure distance, scale as needed
//   WAIT_TIME: how long you have to press the BOOT button
//   BACKUP_TIME: how far to reverse after claw closes
const unsigned long STRAIGHT_DRIVE_TIME = 4000;  // ms
const unsigned long WAIT_TIME           = 5000;  // ms
const unsigned long BACKUP_TIME         = 2000;  // ms

// ============================================
// PHASE STATE MACHINE
// ============================================
enum Phase { PHASE_STRAIGHT, PHASE_WAITING, PHASE_BACKUP, PHASE_UWB };

static Tag uwb_tag(0);

Servo escLeft;
Servo escRight;
Servo clawServo;

Phase         phase             = PHASE_STRAIGHT;
unsigned long phase_start_ms    = 0;
bool          claw_closed       = false;
bool          mission_complete  = false;
double        prev_x            = -999;
double        prev_y            = -999;
double        estimated_heading = 0;
bool          heading_valid     = false;

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
    Serial2.begin(115200);
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

    escLeft.attach(ESC_LEFT,  1000, 2000);
    escRight.attach(ESC_RIGHT, 1000, 2000);
    stop_motors();
    delay(2000);  // ESC arming delay

    clawServo.attach(CLAW_PIN, 1000, 2000);
    clawServo.writeMicroseconds(CLAW_OPEN);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    phase_start_ms = millis();
    Serial.println("PHASE 1: Driving to nest...");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    uwb_tag.update();  // keep UWB warm in all phases

    // BOOT button closes claw — active in all phases
    if (!claw_closed && digitalRead(BUTTON_PIN) == LOW) {
        clawServo.writeMicroseconds(CLAW_CLOSED);
        claw_closed = true;
        Serial.println("Claw closed.");
    }

    unsigned long elapsed = millis() - phase_start_ms;

    // --------------------------------------------------
    // PHASE 1 — Drive straight for STRAIGHT_DRIVE_TIME
    // --------------------------------------------------
    if (phase == PHASE_STRAIGHT) {
        drive(BASE_SPEED, BASE_SPEED);
        if (elapsed >= STRAIGHT_DRIVE_TIME) {
            stop_motors();
            phase          = PHASE_WAITING;
            phase_start_ms = millis();
            Serial.println("PHASE 2: Stopped — press BOOT button to close claw.");
        }
        return;
    }

    // --------------------------------------------------
    // PHASE 2 — Wait for operator to close claw
    // --------------------------------------------------
    if (phase == PHASE_WAITING) {
        stop_motors();
        if (elapsed >= WAIT_TIME) {
            phase          = PHASE_BACKUP;
            phase_start_ms = millis();
            Serial.println("PHASE 3: Backing up...");
        }
        return;
    }

    // --------------------------------------------------
    // PHASE 3 — Reverse for BACKUP_TIME
    // --------------------------------------------------
    if (phase == PHASE_BACKUP) {
        drive(REVERSE_SPEED, REVERSE_SPEED);
        if (elapsed >= BACKUP_TIME) {
            stop_motors();
            phase          = PHASE_UWB;
            phase_start_ms = millis();
            Serial.print("PHASE 4: UWB navigation to (");
            Serial.print(TARGET_X); Serial.print(", ");
            Serial.print(TARGET_Y); Serial.println(")");
        }
        return;
    }

    // --------------------------------------------------
    // PHASE 4 — UWB pivot-turn navigation to opponent nest
    // --------------------------------------------------
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
        drive(BASE_SPEED, BASE_SPEED);
        return;
    }

    double heading_error = target_heading - estimated_heading;
    while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    if (abs(heading_error) <= HEADING_THRESHOLD) {
        drive(BASE_SPEED, BASE_SPEED);
    } else {
        if (heading_error > 0)
            drive(1500, BASE_SPEED);   // turn left
        else
            drive(BASE_SPEED, 1500);   // turn right
        heading_valid = false;
        prev_x = x;
        prev_y = y;
    }

    Serial.print("POS: ("); Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2);
    Serial.print(")  DIST: "); Serial.print(distance, 2);
    Serial.print("  HDG_ERR: "); Serial.println(heading_error, 3);
}
