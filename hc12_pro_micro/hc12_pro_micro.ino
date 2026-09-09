// Arduino Pro Micro (ATmega32U4) <-> HC-12 serial bridge.
//
// Wiring:
//   HC-12 TXD  -> Pro Micro RX1 (pin 0)
//   HC-12 RXD  -> Pro Micro TX1 (pin 1)
//   HC-12 VCC  -> Pro Micro pin 15
//   HC-12 GND  -> Pro Micro GND
//   HC-12 SET  -> Pro Micro A0
//
// SET is active-low: it must be held HIGH for normal transparent
// send/receive (a LOW puts the module into AT-command config mode).
// Leaving it floating - even wired to a pin that's never set to OUTPUT -
// lets it drift or pick up noise, which corrupts data on the link. A0 is
// driven HIGH here explicitly so that never happens.
//
// Pin 15 is driven HIGH for the module's whole lifetime so the HC-12 is
// always powered - it is not a data line and never toggles.
//
// NOTE: an AVR I/O pin sources ~20 mA safely (40 mA absolute max). HC-12
// can draw close to 100 mA on transmit at its higher power levels. Powering
// it directly from pin 15 is fine at low power settings; for full transmit
// power, drive a transistor/MOSFET from pin 15 instead and let it switch
// the HC-12's VCC.
//
// Behavior: anything typed into the USB Serial Monitor is sent out over
// the HC-12 (Serial1), and anything the HC-12 receives over the air is
// printed back to the Serial Monitor - a simple full-duplex bridge for
// talking to another HC-12-equipped device.

const uint8_t HC12_POWER_PIN = 15;
const uint8_t HC12_SET_PIN = A0;

void setup() {
  pinMode(HC12_POWER_PIN, OUTPUT);
  digitalWrite(HC12_POWER_PIN, HIGH); // keep the HC-12 powered on

  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, HIGH); // normal mode, not AT-command mode

  Serial.begin(9600);  // USB CDC to PC (Serial Monitor)
  Serial1.begin(9600); // hardware UART to HC-12
}

void loop() {
  // PC -> HC-12
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }

  // HC-12 -> PC
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
