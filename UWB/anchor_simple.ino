/*
#include "dw3000.h"
#include <SPI.h>

#define APP_NAME "SS TWR RESP v1.0"

// Connection pins (ESP32)
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define TAG_ID 0U
#define ID 1U
#define ID0 static_cast<uint8_t>(ID)
#define ID1 static_cast<uint8_t>(ID >> 8)
#define TAG_ID0 static_cast<uint8_t>(TAG_ID)
#define TAG_ID1 static_cast<uint8_t>(TAG_ID >> 8)

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, TAG_ID0, TAG_ID1, ID0, ID1, 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, ID0, ID1, TAG_ID0, TAG_ID1, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define SENDER_IDX_0 7
#define SENDER_IDX_1 8
#define RECEIVER_IDX_0 5
#define RECEIVER_IDX_1 6
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
static uint8_t frame_seq_nb = 0;

#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

// Delay logic
static uint64_t POLL_RX_TO_RESP_TX_DLY_UUS = 520;

#ifndef UUS_TO_DWT_TIME
#define UUS_TO_DWT_TIME 63890
#endif

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

extern dwt_txconfig_t txconfig_options;

void set_target(int id) {
  rx_poll_msg[RECEIVER_IDX_0] = uint8_t(id);
  rx_poll_msg[RECEIVER_IDX_1] = uint8_t(id >> 8);
  tx_resp_msg[SENDER_IDX_0] = uint8_t(id);
  tx_resp_msg[SENDER_IDX_1] = uint8_t(id >> 8);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("ANCHOR Booting (Diagnostics Mode)..."));

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
  set_target(0);

  Serial.println("Error: Late TX");

  Serial.println("Anchor Ready. Waiting for signals...");
  Serial.println("(Legend: '.' = Waiting, 'E' = Error/Noise, 'R' = Replying)");
}

void loop() {
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  uint32_t loop_start = millis();

  // Wait for event (Good Frame OR Error)
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
  { 
    yield();
    // Print a dot every 1 second to show we are alive but hearing nothing
    if (millis() - loop_start > 1000) {
      Serial.print(".");
      loop_start = millis();
    }
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      
      if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t resp_tx_time;
        int ret;

        poll_rx_ts = get_rx_timestamp_u64();
        
        uint64_t delay_ticks = (uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * (uint64_t)UUS_TO_DWT_TIME;
        uint64_t resp_tx_time_u64 = (poll_rx_ts + delay_ticks) >> 8;
        
        resp_tx_time = (uint32_t)resp_tx_time_u64;
        
        dwt_setdelayedtrxtime(resp_tx_time);

        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); 
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); 
        
        ret = dwt_starttx(DWT_START_TX_DELAYED);

        if (ret == DWT_SUCCESS)
        {
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { yield(); };
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
          frame_seq_nb++;
          Serial.println("R"); // R for Replying
        }
        else
        {
            ++POLL_RX_TO_RESP_TX_DLY_UUS;
            Serial.println("Error: Late TX");
        }
      }
    }
  }
  else
  {
    // If we are here, it's an error (CRC, Timeout, etc)
    Serial.print("E"); // Print E for Error
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}
*/