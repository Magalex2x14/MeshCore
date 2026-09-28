#pragma once

#include <Mesh.h>

#ifndef NRF52_WIRELESS_CHANNEL
  #define NRF52_WIRELESS_CHANNEL  1             // same as repeater bridge.channel: 1 = 2482, 2 = 2450, 3 = 2424 MHz
#endif
#ifndef NRF52_WIRELESS_SECRET
  #define NRF52_WIRELESS_SECRET   "LVSITANOS"   // must match repeater bridge.secret
#endif
#ifndef NRF52_WIRELESS_LBT_RSSI
  #define NRF52_WIRELESS_LBT_RSSI -70           // dBm, channel considered busy above this level
#endif

/**
 * nRF52 hardware RNG (available as the SoftDevice is not enabled)
 */
class NRF52HardwareRNG : public mesh::RNG {
public:
  void random(uint8_t* dest, size_t sz) override;
};

/**
 * Radio driver using the nRF52 2.4GHz RADIO peripheral instead of a LoRa module.
 *
 * It speaks the same air protocol as NRF52RadioBridge (1Mbit GFSK, custom framing, bridge magic,
 * XOR secret + Fletcher-16), so a node using it joins the mesh through a nearby repeater compiled
 * with WITH_NRF52_WIRELESS_BRIDGE.
 *
 * The RADIO peripheral is driven directly, so the SoftDevice must NOT be enabled (no BLE).
 */
class NRF52WirelessRadio : public mesh::Radio {
protected:
  uint32_t n_recv, n_sent, n_recv_errors;
  float _bw;
  uint8_t _sf, _cr;
  float _last_rssi;

public:
  NRF52WirelessRadio() : _bw(LORA_BW), _sf(LORA_SF), _cr(5), _last_rssi(0) { n_recv = n_sent = n_recv_errors = 0; }

  uint32_t getRngSeed();

  /**
   * Only kept for getEstAirtimeFor(): the mesh behind the repeater is LoRa, so ACK/retry
   * timeouts must be based on the LoRa airtime, not on the 2.4GHz one.
   */
  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) { _bw = bw; _sf = sf; _cr = cr; }
  void powerOff();

  bool init();
  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;
  bool isInRecvMode() const override;
  bool isReceiving() override;
  void loop() override;

  uint32_t getPacketsRecv() const { return n_recv; }
  uint32_t getPacketsSent() const { return n_sent; }
  uint32_t getPacketsRecvErrors() const { return n_recv_errors; }
  void resetStats() { n_recv = n_sent = n_recv_errors = 0; }

  virtual float getLastRSSI() const override { return _last_rssi; }
  virtual float getLastSNR() const override { return 0; }

  float packetScore(float snr, int packet_len) override { return 0; }

  /**
   * These two functions do nothing for nRF52 2.4GHz, but are needed for the
   * Radio interface.
   */
  virtual bool setRxBoostedGainMode(bool) { return false; }
  virtual bool getRxBoostedGainMode() const { return false; }

  void setTxPower(int8_t dbm);
};
