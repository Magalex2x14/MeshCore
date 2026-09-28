#pragma once

#include <Mesh.h>

class AbstractBridge {
public:
  virtual ~AbstractBridge() {}

  /**
   * @brief Initializes the bridge.
   */
  virtual void begin() = 0;

  /**
   * @brief Stops the bridge.
   */
  virtual void end() = 0;

  /**
   * @brief Gets the current state of the bridge.
   *
   * @return true if the bridge is initialized and running, false otherwise.
   */
  virtual bool isRunning() const = 0;

  /**
   * @brief A method to be called on every main loop iteration.
   *        Used for tasks like checking for incoming data.
   */
  virtual void loop() = 0;

  /**
   * @brief A callback that is triggered when the mesh transmits a packet.
   *        The bridge can use this to forward the packet.
   *
   * @param packet The packet that was transmitted.
   */
  virtual void sendPacket(mesh::Packet* packet) = 0;

  /**
   * @brief Like sendPacket(), but also passes the link levels, for a companion on the other side
   *        of the bridge (bridge.source companion). Bridges that can't carry them just send the packet.
   *
   * @param packet The packet, or NULL to send only the levels (noise floor update without traffic).
   * @param snr_x4 SNR the packet was received with, in 1/4 dB (as mesh::Packet::_snr).
   * @param rssi RSSI the packet was received with, in dBm.
   * @param noise_floor Current noise floor, in dBm.
   */
  virtual void sendPacketWithLevels(mesh::Packet* packet, int8_t snr_x4, int8_t rssi, int8_t noise_floor) {
    if (packet) sendPacket(packet);
  }

  /**
   * @brief Processes a received packet from the bridge's medium.
   *
   * @param packet The packet that was received.
   */
  virtual void onPacketReceived(mesh::Packet* packet) = 0;
};
