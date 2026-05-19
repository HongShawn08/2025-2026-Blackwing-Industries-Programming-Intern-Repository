/*
 * UGV Navigation System
 * Based on BlackBird Industries UWB Tag code
 * Modified for autonomous ground vehicle navigation
 * 
 * Hardware: ESP32 + DW3000 UWB Module
 */

#include "tag.h"

#include <ESP32Servo.h>

const uint8_t ESC_LEFT = 25;
const uint8_t ESC_RIGHT = 26;

Servo escLeft;
Servo escRight;

// ============================================
// UWB MODULE PINS (ESP32)
// ============================================
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

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

  // ============================================
  //ESC setup
  // ============================================
  escLeft.attach(ESC_LEFT, 1000, 2000);
  escRight.attach(ESC_RIGHT, 1000, 2000);

  // Arm ESCs
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);
  delay(2000);
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    uwb_tag.update();
    
    if (uwb_tag.is_localizing() && !mission_complete) {
        double x = uwb_tag.get_x();
        double y = uwb_tag.get_y();
        
        // Calculate distance to target
        double dx = TARGET_X - x;
        double dy = TARGET_Y - y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance < ARRIVAL_TOLERANCE) {
            stop_motors();
            mission_complete = true;
            Serial.println("TARGET REACHED!");
        } else {
            // Calculate heading
            double heading = atan2(dy, dx);
            
            // Add curve (optional)
            double t = millis() / 1000.0;
            double curve_offset = 0.3 * sin(0.5 * t);
            heading += curve_offset;
            
            // Drive (implement based on your motor driver)
            drive_toward_heading(heading);
            
            Serial.print("Distance: ");
            Serial.println(distance);
        }
    }
}

// ============================================
// MOTOR CONTROL FUNCTIONS (STUBS)
// TODO: Implement these based on your motor driver
// ============================================

void stop_motors() {
    escLeft.writeMicroseconds(1500);
    escRight.writeMicroseconds(1500);
}

void drive_toward_heading(double heading) {
    double current_heading = 0;  // TODO: Get from IMU
    double heading_error = heading - current_heading;
    
    double turn_factor = constrain(heading_error * 2.0, -1.0, 1.0);
    
    int base_speed = 1700;
    int left_speed = base_speed + (turn_factor * 200);
    int right_speed = base_speed - (turn_factor * 200);
    
    left_speed = constrain(left_speed, 1000, 2000);
    right_speed = constrain(right_speed, 1000, 2000);
    
    escLeft.writeMicroseconds(left_speed);
    escRight.writeMicroseconds(right_speed);
}

// TODO: Add tank drive control based on heading error
// void drive_toward_target(double heading) { ... }
