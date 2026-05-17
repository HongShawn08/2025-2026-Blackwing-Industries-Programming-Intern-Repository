/*
 * UGV Navigation System
 * Based on BlackBird Industries UWB Tag code
 * Modified for autonomous ground vehicle navigation
 * 
 * Hardware: ESP32 + DW3000 UWB Module
 */

#include "tag.h"

// ============================================
// UWB MODULE PINS (ESP32)
// ============================================
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

// ============================================
// MOTOR CONTROL PINS (ESP32)
// TODO: Update these based on your motor driver wiring
// ============================================
const uint8_t MOTOR_LEFT_FWD = 25;
const uint8_t MOTOR_LEFT_REV = 26;
const uint8_t MOTOR_RIGHT_FWD = 32;
const uint8_t MOTOR_RIGHT_REV = 33;

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
// GLOBAL OBJECTS
// ============================================
static Tag uwb_tag(0);  // Tag ID = 0

// ============================================
// NAVIGATION PARAMETERS
// TODO: Set these to your actual target (nest) coordinates
// ============================================
const double TARGET_X = 2.0;  // meters
const double TARGET_Y = 2.0;  // meters
const double ARRIVAL_TOLERANCE = 0.2;  // 20cm tolerance

bool mission_complete = false;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("========================================"));
  Serial.println(F("UGV Navigation System Starting..."));
  Serial.println(F("========================================"));
  
  // Motor pins setup
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_REV, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_REV, OUTPUT);
  
  // Initialize motors to stopped
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_REV, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_REV, LOW);

  Serial.println(F("Motor pins initialized"));

  // ============================================
  // UWB INITIALIZATION
  // ============================================
  Serial.println(F("Initializing UWB module..."));
  
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(10); 

  while (!dwt_checkidlerc()) { 
    Serial.println("UWB IDLE CHECK FAILED - Retrying..."); 
    delay(500); 
  }
  Serial.println(F("UWB idle check passed"));
  
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { 
    Serial.println("UWB INIT FAILED - Check wiring!"); 
    while (1); 
  }
  Serial.println(F("UWB initialized"));
  
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  
  if (dwt_configure(&config)) { 
    Serial.println("UWB CONFIG FAILED"); 
    while (1); 
  }
  Serial.println(F("UWB configured"));
  
  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  
  Serial.println(F("========================================"));
  Serial.println(F("UWB Ready!"));
  Serial.println(F("Waiting for position lock..."));
  Serial.print(F("Target: ("));
  Serial.print(TARGET_X);
  Serial.print(F(", "));
  Serial.print(TARGET_Y);
  Serial.println(F(")"));
  Serial.println(F("========================================"));
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    // Update UWB positioning system
    // This runs the state machine: AWAITING_READY → STARTING_ROUND → LOCALIZING
    uwb_tag.update();
    
    // TODO: Add navigation logic here once you have position data
    // The uwb_tag.update() will print "POS: x, y" to Serial when localizing
    
    // Example navigation skeleton (to be completed):
    /*
    if (uwb_tag is in LOCALIZING state) {
        double current_x = uwb_tag.get_x();  // You'll need to add this getter
        double current_y = uwb_tag.get_y();  // You'll need to add this getter
        
        // Calculate distance to target
        double dx = TARGET_X - current_x;
        double dy = TARGET_Y - current_y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance < ARRIVAL_TOLERANCE) {
            // ARRIVED!
            stop_motors();
            mission_complete = true;
        } else {
            // Calculate heading and drive
            double heading_to_target = atan2(dy, dx);
            drive_toward_target(heading_to_target);
        }
    }
    */
}

// ============================================
// MOTOR CONTROL FUNCTIONS (STUBS)
// TODO: Implement these based on your motor driver
// ============================================

void stop_motors() {
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_REV, LOW);
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_REV, LOW);
}

void drive_forward(int speed) {
    // speed: 0-255 (PWM)
    analogWrite(MOTOR_LEFT_FWD, speed);
    analogWrite(MOTOR_RIGHT_FWD, speed);
}

void turn_left(int speed) {
    analogWrite(MOTOR_LEFT_REV, speed);
    analogWrite(MOTOR_RIGHT_FWD, speed);
}

void turn_right(int speed) {
    analogWrite(MOTOR_LEFT_FWD, speed);
    analogWrite(MOTOR_RIGHT_REV, speed);
}

// TODO: Add tank drive control based on heading error
// void drive_toward_target(double heading) { ... }
