// Confirmed hardware pin map (Desert Race Logging Device spec, Rev 2 + Rev 3).
// All assignments below are owner-finalized. Do not change without an
// explicit spec revision.
#pragma once

#include <cstdint>

namespace Pins {

// GNSS u-blox M8N on Serial2 (UART)
constexpr int GPS_RX = 16;   // ESP32 RX <- M8N TX
constexpr int GPS_TX = 17;   // ESP32 TX -> M8N RX

// SIM800L GSM (UART)
constexpr int GSM_RX = 26;
constexpr int GSM_TX = 27;

// LoRa RA-02 (SPI, shared bus with SD)
constexpr int LORA_CS   = 13;
constexpr int LORA_RST  = 2;   // boot-strapping pin: verify reset circuit idle level at boot
constexpr int LORA_DIO0 = 4;

// Mini SD card (SPI, shared bus with LoRa)
constexpr int SD_CS = 5;

// Shared SPI bus (LoRa + SD)
constexpr int SPI_MOSI = 23;
constexpr int SPI_MISO = 19;
constexpr int SPI_SCK  = 18;

// 8x8 NeoPixel matrix
constexpr int NEOPIXEL_DATA = 15;
constexpr int NEOPIXEL_COUNT = 64;

// OLED 128x64 SH1106, I2C (Rev 3 confirmed)
constexpr int OLED_SCL = 22;
constexpr int OLED_SDA = 21;
constexpr uint8_t OLED_I2C_ADDR = 0x3C;       // tested/default; 0x3D optional fallback only
constexpr int OLED_SCREEN_WIDTH  = 128;
constexpr int OLED_SCREEN_HEIGHT = 64;
constexpr int OLED_RESET_PIN = -1;

// Buzzer (via NPN transistor)
constexpr int BUZZER = 14;

// Keypad (3 of 4 keys used; Key3 unused). No internal pull-ups on 34/35/39 -
// external pull-ups/pull-downs are required on the physical wiring.
constexpr int KEY1 = 34;
constexpr int KEY2 = 39;
constexpr int KEY4 = 35;

// 2S Li-ion battery sense (voltage divider into ADC)
constexpr int BATTERY_ADC = 36;

} // namespace Pins
