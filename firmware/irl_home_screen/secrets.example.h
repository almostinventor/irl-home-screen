// Copy this file to secrets.h (same folder) and fill in your own values.
// secrets.h is listed in .gitignore so your WiFi password never gets committed.

#pragma once

// Your 2.4 GHz WiFi network. The ESP32-S2 has no 5 GHz radio.
#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-wifi-password"

// Optional. Only needed if several access points broadcast the same network
// name and the board keeps joining one your laptop cannot reach. Put the MAC
// address (BSSID) of the access point you want, as six bytes, and uncomment.
// On Windows, `netsh wlan show interfaces` shows the BSSID your laptop is using.
// #define WIFI_BSSID { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 }
