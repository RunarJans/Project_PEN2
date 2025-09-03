
# Gebruikte Hardware in Project_PEN2

## 1. **CY8CKIT-062-WIFI-BT (PSoC 6 Pioneer Kit)**
- **Doel**: Centrale microcontroller voor dit project.
- **Eigenschappen**:
  - Dual-core ARM Cortex-M4 + M0+.
  - Onboard programmer/debugger.
  - USB-seriële bridge, knoppen en LED’s.
  - Geïntegreerde I/O en uitbreidingsheaders.

## 2. **TFT Shield (ST7789V via SPI)**
- **Doel**: Visuele output voor oscilloscoop, multimeter en scanners.
- **Interface**: SPI met `mtb_st7789v` driver.
- **Opmerking**: EmWin wordt gebruikt voor grafische aansturing.

## 3. **Potentiometer 1 kΩ**
- **Doel**: Spanning genereren (0-3.3V) voor ADC-testen.
- **Aansluiting**: Middelste pin naar ADC P10_0, zijkanten naar 3V3 en GND.

## 4. **Referentieweerstand 10 kΩ**
- **Doel**: Gebruikt in spanningsdelerconfiguratie om een onbekende weerstand te bepalen.
- **Formule**: `Rx = Rref * Vnode / (Vcc - Vnode)`.

## 5. **BMP280 Sensor (I²C)**
- **Doel**: I²C-scanner test.
- **Eigenschappen**:
  - Ondersteunt zowel **I²C** als **SPI**.
  - **I²C**: standaard adres = 0x76 of 0x77.
  - **SPI**: mogelijk indien CSB pin op GND (gebruikt 4 draden).

## 6. **LED + serieweerstand (330Ω – 1kΩ)**
- **Doel**: Testen van digitale outputpin in Digital Pin Tester.
- **Aansluiting**: P13_6 → weerstand → LED → GND.

## 7. **Jumper wires & Breadboard**
- **Doel**: Voor het opbouwen van meetopstellingen (spanningsdeler, potmeter, LED, enz.).

---
