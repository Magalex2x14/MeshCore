#include "NRF52WirelessRadio.h"
#include <Arduino.h>
#include <nrf.h>
#include <nrf_sdm.h>
#include <helpers/TxtDataHelpers.h>

// Air protocol of NRF52RadioBridge (helpers/bridges/NRF52RadioBridge.cpp), must be kept in sync
#define NRF52_WIRELESS_BASE_ADDR     0x4D434252
#define NRF52_WIRELESS_ADDR_PREFIX   0xE7
#define NRF52_WIRELESS_WHITENING_IV  0x58
#define NRF52_WIRELESS_IRQ_PRIORITY  3
#define BRIDGE_PACKET_MAGIC          0xC03E
#define BRIDGE_PACKET_MAGIC_LEVELS   0xC03F   // + SNR x4, RSSI, noise floor (bridge.source companion)
#define BRIDGE_LEVELS_SIZE           3
#define BRIDGE_MAGIC_SIZE            2
#define BRIDGE_CHECKSUM_SIZE         2

#define MAX_RADIO_PACKET_SIZE  255
#define MAX_PAYLOAD_SIZE       (MAX_RADIO_PACKET_SIZE - (BRIDGE_MAGIC_SIZE + BRIDGE_CHECKSUM_SIZE))
#define RX_SLOTS               4    // one RX slot is always owned by the radio
#define MAX_LBT_ATTEMPTS       10   // max times a TX is deferred due to a busy channel before sending anyway

enum RadioState : uint8_t { STATE_OFF, STATE_RX, STATE_TX };

// radio buffers: [0] = length, [1..] = payload, as the RADIO DMA expects
static uint8_t rx_frames[RX_SLOTS][1 + MAX_RADIO_PACKET_SIZE];
static int8_t rx_rssi[RX_SLOTS];
static volatile uint8_t rx_head = 0; // slot the radio is receiving into (ISR owned)
static volatile uint8_t rx_tail = 0; // next slot to process (loop owned)
static uint8_t tx_frame[1 + MAX_RADIO_PACKET_SIZE];

static volatile RadioState state = STATE_OFF;
static volatile bool rx_busy = false;           // address matched, packet reception in progress
static volatile bool is_send_complete = true;
static bool tx_pending = false;                 // queued, waiting for a clear channel
static uint8_t tx_attempts = 0;
static uint32_t tx_next_attempt = 0;
static int8_t tx_power_dbm = 4;
static uint8_t channel = NRF52_WIRELESS_CHANNEL;
static char secret[16] = NRF52_WIRELESS_SECRET;   // same size as repeater bridge_secret
static KeyValueStore* prefs_store = NULL;

// counters for get bridge.stats
static volatile uint32_t stat_addr = 0, stat_crc_ok = 0, stat_crc_err = 0, stat_tx = 0;
static uint32_t stat_bad_magic = 0, stat_bad_sum = 0;
static int stat_noise_floor = 0;

static uint8_t channelToFrequency(uint8_t ch) {
  static const uint8_t freqs[NRF52WirelessRadio::NUM_CHANNELS] = { 82, 50, 24 };  // 2482, 2450, 2424 MHz (same as NRF52RadioBridge)
  if (ch < 1 || ch > NRF52WirelessRadio::NUM_CHANNELS) ch = 1;
  return freqs[ch - 1];
}

static int8_t supportedTxPower(int8_t dbm) {
  static const int8_t steps[] = {
#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
    8, 7, 6, 5,
#endif
    4, 3,
#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
    2,
#endif
    0, -4, -8, -12, -16,
  };
  for (int8_t step : steps) {
    if (dbm >= step) return step;
  }
  return -20;
}

static uint16_t fletcher16(const uint8_t *data, size_t len) {
  uint8_t sum1 = 0, sum2 = 0;
  for (size_t i = 0; i < len; i++) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

static void xorCrypt(uint8_t *data, size_t len) {
  size_t keyLen = strnlen(secret, sizeof(secret));
  if (keyLen == 0) return;
  for (size_t i = 0; i < len; i++) {
    data[i] ^= secret[i % keyLen];
  }
}

static void configureRadio() {
  // Power cycle the peripheral to get it into a known (reset) state
  NRF_RADIO->POWER = 0;
  NRF_RADIO->POWER = 1;

  // BLE 1M PHY modulation with our own address and framing, same as NRF52RadioBridge
  NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos;
  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos) |
                        (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
  NRF_RADIO->TXPOWER = (uint8_t)supportedTxPower(tx_power_dbm) << RADIO_TXPOWER_TXPOWER_Pos;
  NRF_RADIO->FREQUENCY = channelToFrequency(channel) << RADIO_FREQUENCY_FREQUENCY_Pos;

  NRF_RADIO->BASE0 = NRF52_WIRELESS_BASE_ADDR;
  NRF_RADIO->PREFIX0 = NRF52_WIRELESS_ADDR_PREFIX << RADIO_PREFIX0_AP0_Pos;
  NRF_RADIO->TXADDRESS = 0;
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Msk;

  NRF_RADIO->PCNF0 = (8 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos)
#if defined(RADIO_PCNF0_PLEN_Pos)
                     | (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos)
#endif
      ;
  NRF_RADIO->PCNF1 = (MAX_RADIO_PACKET_SIZE << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) |
                     (4 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                     (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);
  NRF_RADIO->DATAWHITEIV = NRF52_WIRELESS_WHITENING_IV;

  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Include << RADIO_CRCCNF_SKIPADDR_Pos);
  NRF_RADIO->CRCINIT = 0xFFFF;
  NRF_RADIO->CRCPOLY = 0x11021;

  NRF_RADIO->INTENCLR = 0xFFFFFFFF;
  NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk | RADIO_INTENSET_END_Msk | RADIO_INTENSET_DISABLED_Msk;

  NVIC_SetPriority(RADIO_IRQn, NRF52_WIRELESS_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(RADIO_IRQn);
}

static void startRx() {
  // radio must be DISABLED; called from ISR or with RADIO_IRQn masked
  rx_busy = false;
  NRF_RADIO->PACKETPTR = (uint32_t)rx_frames[rx_head];
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
                      RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
  state = STATE_RX;
  NRF_RADIO->TASKS_RXEN = 1;
}

static void restartRadio() {
  NVIC_DisableIRQ(RADIO_IRQn);
  configureRadio();
  startRx();
  NVIC_EnableIRQ(RADIO_IRQn);
}

extern "C" void RADIO_IRQHandler(void) {
  if (NRF_RADIO->EVENTS_ADDRESS) {
    NRF_RADIO->EVENTS_ADDRESS = 0;
    if (state == STATE_RX) rx_busy = true;
    stat_addr++;
  }

  if (NRF_RADIO->EVENTS_END) {
    NRF_RADIO->EVENTS_END = 0;
    if (state == STATE_RX) {
      rx_busy = false;
      if (NRF_RADIO->CRCSTATUS == RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
        stat_crc_ok++;
        rx_rssi[rx_head] = -(int8_t)NRF_RADIO->RSSISAMPLE;
        uint8_t next = (rx_head + 1) % RX_SLOTS;
        if (next != rx_tail) { // commit, otherwise ring is full and frame is dropped
          rx_head = next;
        }
      } else {
        stat_crc_err++;
      }
      // radio sits in RXIDLE after END, re-arm with the (possibly new) slot
      NRF_RADIO->PACKETPTR = (uint32_t)rx_frames[rx_head];
      NRF_RADIO->TASKS_START = 1;
    }
  }

  if (NRF_RADIO->EVENTS_DISABLED) {
    NRF_RADIO->EVENTS_DISABLED = 0;
    if (state == STATE_TX) {
      stat_tx++;
      is_send_complete = true;
      startRx();
    }
  }

  // flush the event clears before leaving the ISR, avoids spurious re-entry
  (void)NRF_RADIO->EVENTS_DISABLED;
}

static bool isChannelClear() {
  if (rx_busy) return false;

  NRF_RADIO->EVENTS_RSSIEND = 0;
  NRF_RADIO->TASKS_RSSISTART = 1;
  uint32_t start = micros();
  while (!NRF_RADIO->EVENTS_RSSIEND) {
    if (micros() - start > 50) return true; // no sample, don't block TX
  }
  NRF_RADIO->EVENTS_RSSIEND = 0;
  int rssi = -(int)NRF_RADIO->RSSISAMPLE;
  return rssi < NRF52_WIRELESS_LBT_RSSI && !rx_busy;
}

static void transmitFrame() {
  NVIC_DisableIRQ(RADIO_IRQn);

  // RX -> DISABLED, then ramp up TX; END_DISABLE brings us back to DISABLED and the ISR restarts RX
  state = STATE_OFF;
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  uint32_t start = micros();
  while (!NRF_RADIO->EVENTS_DISABLED && (micros() - start) < 200) {
  }
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->EVENTS_ADDRESS = 0;
  NRF_RADIO->EVENTS_END = 0;
  rx_busy = false;

  NRF_RADIO->PACKETPTR = (uint32_t)tx_frame;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  state = STATE_TX;
  NRF_RADIO->TASKS_TXEN = 1;

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);
}

bool NRF52WirelessRadio::init() {
  uint8_t sd_enabled = 0;
  sd_softdevice_is_enabled(&sd_enabled);
  if (sd_enabled) {
    MESH_DEBUG_PRINTLN("NRF52WirelessRadio: SoftDevice enabled, radio not available");
    return false;
  }

  // RADIO requires the external high frequency crystal
  if ((NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Msk | CLOCK_HFCLKSTAT_STATE_Msk)) !=
      ((CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) | CLOCK_HFCLKSTAT_STATE_Msk)) {
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    uint32_t start = millis();
    while (!NRF_CLOCK->EVENTS_HFCLKSTARTED) {
      if (millis() - start > 10) {
        MESH_DEBUG_PRINTLN("NRF52WirelessRadio: HFXO failed to start");
        return false;
      }
    }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  }

  rx_head = rx_tail = 0;
  tx_pending = false;
  is_send_complete = true;
  restartRadio();

  return true;
}

void NRF52WirelessRadio::powerOff() {
  NVIC_DisableIRQ(RADIO_IRQn);
  state = STATE_OFF;
  NRF_RADIO->INTENCLR = 0xFFFFFFFF;
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  NRF_RADIO->POWER = 0;
}

void NRF52HardwareRNG::random(uint8_t* dest, size_t sz) {
  NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Msk;
  NRF_RNG->TASKS_START = 1;
  for (size_t i = 0; i < sz; i++) {
    NRF_RNG->EVENTS_VALRDY = 0;
    while (!NRF_RNG->EVENTS_VALRDY) {
    }
    dest[i] = NRF_RNG->VALUE;
  }
  NRF_RNG->TASKS_STOP = 1;
}

uint32_t NRF52WirelessRadio::getRngSeed() {
  uint32_t seed;
  NRF52HardwareRNG rng;
  rng.random((uint8_t *)&seed, sizeof(seed));
  return seed;
}

void NRF52WirelessRadio::setTxPower(int8_t dbm) {
  tx_power_dbm = dbm;
  NRF_RADIO->TXPOWER = (uint8_t)supportedTxPower(dbm) << RADIO_TXPOWER_TXPOWER_Pos;  // applies to next TX
}

bool NRF52WirelessRadio::startSendRaw(const uint8_t* bytes, int len) {
  if (len > MAX_PAYLOAD_SIZE) {
    MESH_DEBUG_PRINTLN("NRF52WirelessRadio: TX packet too large, len=%d", len);
    return false;
  }

  uint8_t *buffer = &tx_frame[1];

  // magic header, Fletcher-16 of the payload, payload (same layout as NRF52RadioBridge)
  buffer[0] = (BRIDGE_PACKET_MAGIC >> 8) & 0xFF;
  buffer[1] = BRIDGE_PACKET_MAGIC & 0xFF;
  memcpy(&buffer[BRIDGE_MAGIC_SIZE + BRIDGE_CHECKSUM_SIZE], bytes, len);
  uint16_t checksum = fletcher16(bytes, len);
  buffer[2] = (checksum >> 8) & 0xFF;
  buffer[3] = checksum & 0xFF;

  // encrypt checksum + payload (not the magic header)
  xorCrypt(&buffer[BRIDGE_MAGIC_SIZE], len + BRIDGE_CHECKSUM_SIZE);
  tx_frame[0] = BRIDGE_MAGIC_SIZE + BRIDGE_CHECKSUM_SIZE + len;

  is_send_complete = false;
  tx_pending = true;
  tx_attempts = 0;
  tx_next_attempt = millis();
  n_sent++;

  loop();  // try to send right away
  return true;
}

void NRF52WirelessRadio::loop() {
  if (!tx_pending) return;
  if ((int32_t)(millis() - tx_next_attempt) < 0) return; // backing off

  if (tx_attempts < MAX_LBT_ATTEMPTS && !isChannelClear()) {
    tx_attempts++;
    tx_next_attempt = millis() + random(1, 4 * tx_attempts + 2);
    return;
  }

  tx_pending = false;
  transmitFrame();
}

bool NRF52WirelessRadio::isSendComplete() {
  return is_send_complete;
}

void NRF52WirelessRadio::onSendFinished() {
  if (!is_send_complete) {
    // send timed out, recover the radio and drop the frame
    MESH_DEBUG_PRINTLN("NRF52WirelessRadio: TX timeout, resetting radio");
    tx_pending = false;
    restartRadio();
    is_send_complete = true;
  }
}

bool NRF52WirelessRadio::isInRecvMode() const {
  return is_send_complete;    // if NO send in progress, then we're in Rx mode
}

bool NRF52WirelessRadio::isReceiving() {
  return rx_busy;
}

int NRF52WirelessRadio::recvRaw(uint8_t* bytes, int sz) {
  while (rx_tail != rx_head) {
    const uint8_t *frame = rx_frames[rx_tail];
    int8_t rssi = rx_rssi[rx_tail];
    size_t len = frame[0];
    uint8_t decrypted[MAX_RADIO_PACKET_SIZE];
    if (len >= BRIDGE_MAGIC_SIZE + BRIDGE_CHECKSUM_SIZE) {
      memcpy(decrypted, &frame[1], len);
    }
    rx_tail = (rx_tail + 1) % RX_SLOTS;  // slot can be reused by the radio now

    if (len < BRIDGE_MAGIC_SIZE + BRIDGE_CHECKSUM_SIZE) continue;  // too small
    uint16_t magic = (decrypted[0] << 8) | decrypted[1];
    if (magic != BRIDGE_PACKET_MAGIC && magic != BRIDGE_PACKET_MAGIC_LEVELS) {  // not a bridge packet
      stat_bad_magic++;
      continue;
    }

    uint8_t *data = &decrypted[BRIDGE_MAGIC_SIZE];
    size_t data_len = len - BRIDGE_MAGIC_SIZE;
    xorCrypt(data, data_len);

    uint16_t received_checksum = (data[0] << 8) | data[1];
    int payload_len = data_len - BRIDGE_CHECKSUM_SIZE;
    if (fletcher16(&data[BRIDGE_CHECKSUM_SIZE], payload_len) != received_checksum || payload_len > sz) {
      // failed to decrypt - likely from a different network
      n_recv_errors++;
      stat_bad_sum++;
      continue;
    }

    const uint8_t *payload = &data[BRIDGE_CHECKSUM_SIZE];
    if (magic == BRIDGE_PACKET_MAGIC_LEVELS) {
      if (payload_len < BRIDGE_LEVELS_SIZE) {
        n_recv_errors++;
        continue;
      }
      // levels the repeater received the packet with, and its noise floor
      int8_t snr_x4 = (int8_t)payload[0];
      int8_t rep_rssi = (int8_t)payload[1];
      _noise_floor = stat_noise_floor = (int8_t)payload[2];
      payload += BRIDGE_LEVELS_SIZE;
      payload_len -= BRIDGE_LEVELS_SIZE;
      if (payload_len == 0) continue;  // levels-only update

      _last_snr = snr_x4 / 4.0f;
      _last_rssi = rep_rssi;
    } else {
      _last_snr = 0;
      _last_rssi = rssi;  // 2.4GHz link
    }

    memcpy(bytes, payload, payload_len);
    n_recv++;
    return payload_len;
  }
  return 0;
}

uint32_t NRF52WirelessRadio::getEstAirtimeFor(int len_bytes) {
  // LoRa time on air (explicit header, CRC on) with the mesh radio params
  float t_sym = (float)(1 << _sf) / _bw;  // ms
  int de = t_sym > 16.0f ? 1 : 0;         // low data rate optimisation
  int num = 8 * len_bytes - 4 * _sf + 28 + 16;
  int den = 4 * (_sf - 2 * de);
  int n_payload = 8 + max((num + den - 1) / den, 0) * _cr;
  int n_preamble = _sf <= 8 ? 32 : 16;    // RadioLibWrapper::preambleLengthForSF()
  return (n_preamble + 4.25f + n_payload) * t_sym;
}

static void applyChannel() {
  if (state == STATE_OFF) return;  // not initialized, configureRadio() will pick it up
  tx_pending = false;              // retune now, drops a frame being sent
  restartRadio();
  is_send_complete = true;
}

void NRF52WirelessRadio::attachDynamicPrefs(KeyValueStore* prefs) {
  prefs_store = prefs;

  char tmp[sizeof(secret)];
  if (prefs->getByKey("br_ch", tmp, sizeof(tmp) - 1)) {
    int ch = atoi(tmp);
    if (ch > 0 && ch <= NUM_CHANNELS) channel = ch;
  }
  if (prefs->getByKey("br_sec", tmp, sizeof(tmp) - 1)) {
    StrHelper::strncpy(secret, tmp, sizeof(secret));
  }

  applyChannel();
}

bool NRF52WirelessRadio::handleCommand(const char* command, char* reply) {
  if (strcmp(command, "get bridge.channel") == 0) {
    sprintf(reply, "> %d", (uint32_t)channel);
    return true;
  }
  if (memcmp(command, "set bridge.channel ", 19) == 0) {
    int ch = atoi(&command[19]);
    if (ch > 0 && ch <= NUM_CHANNELS) {
      channel = ch;
      if (prefs_store) {
        char tmp[4];
        sprintf(tmp, "%d", ch);
        prefs_store->setByKey("br_ch", tmp);
      }
      applyChannel();
      strcpy(reply, "OK");
    } else {
      sprintf(reply, "Error: channel must be between 1-%d", NUM_CHANNELS);
    }
    return true;
  }
  if (strcmp(command, "get bridge.stats") == 0) {
    int rssi = 0;
    if (state == STATE_RX && !rx_busy) {
      NRF_RADIO->EVENTS_RSSIEND = 0;
      NRF_RADIO->TASKS_RSSISTART = 1;
      uint32_t start = micros();
      while (!NRF_RADIO->EVENTS_RSSIEND && micros() - start < 50) {
      }
      rssi = -(int)NRF_RADIO->RSSISAMPLE;
    }
    // addr: address matches (incl. other nRF traffic), crc ok/err, magic: not a bridge frame,
    // sum: bad checksum (other bridge.secret), tx: frames sent, st: radio state, hf: HFCLKSTAT
    sprintf(reply, "> addr:%u crc_ok:%u crc_err:%u magic:%u sum:%u tx:%u st:%d hf:%lx ch:%d nf:%d rssi:%d",
            stat_addr, stat_crc_ok, stat_crc_err, stat_bad_magic, stat_bad_sum, stat_tx, (int)state,
            (unsigned long)NRF_CLOCK->HFCLKSTAT, (int)channel, stat_noise_floor, rssi);
    return true;
  }
  if (strcmp(command, "get bridge.secret") == 0) {
    sprintf(reply, "> %s", secret);
    return true;
  }
  if (memcmp(command, "set bridge.secret ", 18) == 0) {
    const char* sp = &command[18];
    if (strchr(sp, ':') || strchr(sp, '|')) {   // separators of the custom prefs
      strcpy(reply, "Error, bad chars");
    } else {
      StrHelper::strncpy(secret, sp, sizeof(secret));
      if (prefs_store) prefs_store->setByKey("br_sec", secret);
      strcpy(reply, "OK");
    }
    return true;
  }
  return false;  // not handled
}
