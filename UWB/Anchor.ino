#include "../anchor.h"

// connection pins (ESP32)
static const uint8_t PIN_RST = 27;  // reset pin
static const uint8_t PIN_IRQ = 34;  // irq pin
static const uint8_t PIN_SS = 4;    // spi select pin

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

extern dwt_txconfig_t txconfig_options;

static auto node = Anchor(0);
// static auto node = Tag(0);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("UWB Booting (Diagnostics Mode)..."));

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(10); 

  while (!dwt_checkidlerc()) { Serial.println("IDLE FAILED"); delay(500); }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { Serial.println("INIT FAILED"); while (1); }
  
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  if (dwt_configure(&config)) { Serial.println("CONFIG FAILED"); while (1); }
  
  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  Serial.println("Ready");
}

void loop() {
    // put your main code here, to run repeatedly:
    node.update();
}
