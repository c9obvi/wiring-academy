// Copy this file to secrets.h (same folder) and change the password.
// secrets.h is gitignored so the password never lands in the public repo.
#pragma once
#define WIFI_AP_NAME     "McLaren-F1"
#define WIFI_AP_PASSWORD "change-me-8+chars"   // at least 8 characters

// Optional garage mode: the car also joins your home Wi-Fi so firmware can be updated over the air,
// and http://mclaren.local/status shows live state. Driving still only works on the car's own network.
// #define HOME_WIFI_SSID     "your-home-wifi"
// #define HOME_WIFI_PASSWORD "your-home-wifi-password"
// #define OTA_PASSWORD       "pick-an-update-password"
