/*
#include "dw3000.h"
#include <SPI.h>
#include "variance.h"

#define APP_NAME "SS TWR INIT v1.0"

// connection pins (ESP32)
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define RNG_DELAY_MS 3     // Faster ranging
#define PAIR_DELAY_MS 250  // Faster ranging
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SLOPE 0.954
#define INTERCEPT 0.112

#define ID 0U
#define ID0 static_cast<uint8_t>(ID)
#define ID1 static_cast<uint8_t>(ID >> 8)

// Format: Header, Sender ID, Receiver ID, other data
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, ID0, ID1, 'V', 'E', 0xE0, 0, 0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', ID0, ID1, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define SENDER_IDX_0 5
#define SENDER_IDX_1 6
#define RECEIVER_IDX_0 7
#define RECEIVER_IDX_1 8
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
static uint8_t frame_seq_nb = 0;

#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

// DELAYS
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 500

static double tof;
static double distance;
extern dwt_txconfig_t txconfig_options;
static bool paired = false;

static uint32_t sample_size = 1000;
static bool printed = false;
static StreamingStats stats;

void set_target(int id) {
  tx_poll_msg[RECEIVER_IDX_0] = uint8_t(id);
  tx_poll_msg[RECEIVER_IDX_1] = uint8_t(id >> 8);
  rx_resp_msg[SENDER_IDX_0] = uint8_t(id);
  rx_resp_msg[SENDER_IDX_1] = uint8_t(id >> 8);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("TAG Booting..."));

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(10);

  while (!dwt_checkidlerc()) {
    Serial.println("IDLE FAILED");
    delay(500);
  }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("INIT FAILED");
    while (1)
      ;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  if (dwt_configure(&config)) {
    Serial.println("CONFIG FAILED");
    while (1)
      ;
  }

  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  set_target(1);

  Serial.println("Tag Ready.");
}

void loop() {
  // if (stats.count() >= sample_size) {
  //   if (!printed) {

  //     Serial.println();
  //     Serial.print("Mean = ");
  //     Serial.print(stats.getMean(), 4);
  //     Serial.print(", std = ");
  //     Serial.println(stats.getStdDev(), 5);
  //     printed = true;
  //   }
  //   return;
  // }

  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  uint32_t start_wait = millis();
  status_reg = 0;

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    // Safety timeout in case hardware hangs
    if (millis() - start_wait > 1000) {
      Serial.println("Tag Loop Timeout");
      break;
    }
  };

  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    if (frame_len <= sizeof(rx_buffer)) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT * SLOPE + INTERCEPT;

        // stats.push(distance);

        if (!paired) {
          Serial.println("Paired");
        }
        Serial.print("DIST: ");
        Serial.print(distance);
        Serial.println(" m");
        paired = true;
      }
    }
  } else {
    // Clean up error bits
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  if (paired)
    delay(RNG_DELAY_MS);
  else
    delay(PAIR_DELAY_MS);
}
*/