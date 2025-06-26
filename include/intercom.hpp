#pragma once

#ifndef __INTERCOM_HPP__
#define __INTERCOM_HPP__

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#include "flag_manager.hpp"
extern StatusFlags statusFlags;

namespace Intercom
{
    uint8_t partnerMacLists[2][6] = {
        {0x24, 0x6F, 0x28, 0xA1, 0xB2, 0xC3},
        {0x24, 0x6F, 0x28, 0xA4, 0xB5, 0xC6}
    };

    // One is the partner, and one is myself.
    // This approch is used, so we can upload the same code.

    uint8_t partnerMac[6] = {0};


    void printMacAddress()
    {
        uint8_t baseMac[6];
        esp_err_t ret = esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
        if (ret != ESP_OK)
        {
            Serial.println(F("esp_read_mac failed"));
            return;
        }
        
        Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    }

    union InterPacket
    {
        struct intercom_packet
        {
            uint32_t message: 20;
            uint32_t state  : 4;
            uint32_t hash   : 8;
        } packet;
        uint32_t raw;
    } __attribute__((packed));

    constexpr uint64_t FNV_prime = 0x100000001b3;
    constexpr uint64_t FNV_offset_basis = 0xcbf29ce484222325;

    void SendPacket(uint32_t message)
    {
        InterPacket packet;
        packet.packet.message = message;
        packet.packet.state = statusFlags.flags.status;
        packet.packet.hash = 0;

        uint64_t intermediate_hash = FNV_offset_basis;
        for (size_t i = 0; i < sizeof(packet.raw); ++i) {
            intermediate_hash ^= ((uint8_t*)&packet.raw)[i];
            intermediate_hash *= FNV_prime;
        }
        packet.packet.hash = (uint8_t)(intermediate_hash & 0xFF);

        Serial.printf("Sending packet: message: %u, state: %u, hash: %u\n", packet.packet.message, packet.packet.state, packet.packet.hash);
        esp_err_t result = esp_now_send(partnerMac, (uint8_t*)&packet.raw, sizeof(packet.raw));
        // We can ignore the result here, because we will get a callback.
    }

    esp_now_peer_info_t peerInfo;
    int lastSuccessTime = -1;
    constexpr int peerDeadLimit = 5000; // 5 seconds
    int failedCount = 0;
    constexpr int maxFailedCount = 3;

    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        Serial.printf("Data sent to %02X:%02X:%02X:%02X:%02X:%02X, status: %d\n",
                    mac_addr[0], mac_addr[1], mac_addr[2],
                    mac_addr[3], mac_addr[4], mac_addr[5], status);

        if (status == 0) // Success
        {
            Serial.println(F("Data sent successfully"));
            lastSuccessTime = millis();
            failedCount = 0;
            statusFlags.flags.peer_dead = 0;
        }
        else
        {
            Serial.println(F("Data send failed"));
            
            // Check if the peer is dead
            if (lastSuccessTime != -1)
            {
                if (millis() - lastSuccessTime > peerDeadLimit && failedCount > maxFailedCount)
                {
                    Serial.println(F("Peer is dead"));
                    statusFlags.flags.peer_dead = 1;
                    lastSuccessTime = -1; // Reset last success time
                }
                else
                {
                    failedCount++;
                    Serial.print(F("Failed count: "));
                    Serial.println(failedCount);
                }
            }
        }
    }

    extern void ProcessData(uint32_t message, uint8_t state);

    void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)
    {
        if (len != sizeof(InterPacket))
        {
            Serial.println(F("Received data size mismatch"));
            return;
        }

        InterPacket packet;
        memcpy(&packet.raw, data, sizeof(packet.raw));

        Serial.printf("Data received from %02X:%02X:%02X:%02X:%02X:%02X: message: %u, state: %u, hash: %u\n",
                    mac_addr[0], mac_addr[1], mac_addr[2],
                    mac_addr[3], mac_addr[4], mac_addr[5],
                    packet.packet.message, packet.packet.state, packet.packet.hash);

        // Validate hash
        uint64_t intermediate_hash = FNV_offset_basis;
        uint64_t hash = packet.packet.hash;
        packet.packet.hash = 0; // Set hash to 0 for hash calculation
        for (size_t i = 0; i < sizeof(packet.raw); ++i) {
            intermediate_hash ^= ((uint8_t*)&packet.raw)[i];
            intermediate_hash *= FNV_prime;
        }
        
        if (hash != (uint8_t)(intermediate_hash & 0xFF))
        {
            Serial.println(F("Invalid hash received"));
            return;
        }

        // Validate the mac address
        if (memcmp(mac_addr, partnerMac, 6) != 0)
        {
            Serial.println(F("Received data from unknown peer. Seriously, what the fuck?"));
            return;
        }

        // Since we received something, the peer is alive
        statusFlags.flags.peer_dead = 0;
        lastSuccessTime = millis();
        failedCount = 0;
        Serial.printf("Valid packet received: message: %u, state: %u\n", packet.packet.message, packet.packet.state);

        // Process
        ProcessData(packet.packet.message, packet.packet.state);
    }

    bool init(void)
    {
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK)
        {
            Serial.println(F("ESP-NOW initialization failed."));
            statusFlags.flags.esp_now_inited = 0;
            return 0;
        }
        
        uint8_t baseMac[6];
        esp_err_t ret = esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
        if (ret != ESP_OK)
        {
            Serial.println(F("esp_read_mac failed"));
            statusFlags.flags.esp_now_inited = 0;
            return 0;
        }
        
        bool first = true;
        for (int i = 0; i < 6; i++)
        {
            if (baseMac[i] != partnerMacLists[0][i])
            {
                first = false;
                break;
            }
        }
        if (first)
            memcpy(partnerMac, partnerMacLists[1], 6);
        else
            memcpy(partnerMac, partnerMacLists[0], 6);
        
        Serial.printf("Partner MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", partnerMac[0], partnerMac[1], partnerMac[2], partnerMac[3], partnerMac[4], partnerMac[5]);

        memcpy(peerInfo.peer_addr, partnerMac, 6);
        peerInfo.channel = 0; // Use the current WiFi channel
        peerInfo.encrypt = false; // No encryption

        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
        esp_now_register_send_cb(OnDataSent);

        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println(F("Failed to add peer"));
            statusFlags.flags.esp_now_inited = 0;
            return 0;
        }

        Serial.println(F("ESP-NOW initialized successfully"));
        statusFlags.flags.esp_now_inited = 1;
        return 1;
    }

    enum messages
    {
        MSG_STATE_CHANGETO_ENGINE = 0,
        MSG_STATE_CHANGETO_FREE_ASCENT,
        MSG_STATE_CHANGETO_APOGEE,
        MSG_STATE_CHANGETO_FREE_FALL,
        MSG_STATE_CHANGETO_DROGUE_DESCENT,
        MSG_STATE_CHANGETO_MAIN_DESCENT,
        MSG_STATE_CHANGETO_TERMINAL_VELOCITY,
        MSG_STATE_CHANGETO_TOUCHDOWN,
        MSG_STATE_CHANGETO_RECOVERY,
        MSG_STATE_CHANGETO_ERROR,

        MSG_BEAT,
    };
} // namespace Intercom
#endif // __INTERCOM_HPP__