# Hardware documentatie – Project_PEN2

Dit bestand beschrijft alle gebruikte hardwarecomponenten in het project, hun functie en enkele relevante eigenschappen.  
Elke sectie bevat ook een link naar een foto in de [`pictures/`](../pictures) map.

---

## PSoC6 Pioneer Kit (CY8CKIT-062-WiFi-BT)
- **Functie:** Hoofdcontroller, voert alle code uit.  
- **Eigenschappen:**
  - Dual-core ARM Cortex-M4/M0+.
  - Ingebouwd WiFi/BT.  
  - Onboard programmer/debugger.  
  - TFT-shield interface (J2).  
- **Foto:** ![PSoC6 Board](../pictures/psoc6_board.png)

---

## TFT Shield (ST7789V)
- **Functie:** Weergave van oscilloscoop, multimeter en menu’s.  
- **Eigenschappen:**
  - 320×240 pixels.  
  - Aangestuurd via `mtb_st7789v` driver + emWin.  
- **Foto:** ![TFT Shield](../pictures/tft_shield.png)

---

## Potentiometer (1 kΩ)
- **Functie:** Referentie voor ADC-kalibratie en spanningsmeting.  
- **Eigenschappen:**
  - Lineair, instelbaar 0–3.3 V op de ADC-pin.  
- **Foto:** ![Potmeter](../pictures/potmeter.png)

---

## Referentieweerstand (Rref = 10 kΩ)
- **Functie:** Wordt gebruikt voor de spanningsdeler in de weerstandsmodus.  
- **Eigenschappen:**
  - Precisie belangrijk (meet exact met multimeter en vul waarde in code).  
- **Foto:** ![Rref](../pictures/rref.png)

---

## BMP280 Sensor (optioneel, I²C)
- **Functie:** Testdevice voor I²C-scanner.  
- **Eigenschappen:**
  - Meet temperatuur en druk.  
  - Adres = 0x76 of 0x77 (configureerbaar).  
  - Kan ook via **SPI** werken als de pinnen anders aangesloten worden.  
- **Foto:** ![BMP280](../pictures/bmp280.png)

---

## LED + serieweerstand
- **Functie:** Voor test van de Digital Pin Tester (P13_6).  
- **Eigenschappen:**
  - Typisch 330 Ω – 1 kΩ in serie.  
  - Visuele indicatie van HIGH/LOW.  
- **Foto:** ![LED test](../pictures/led_test.png)

---

## Breadboard + jumpers
- **Functie:** Verbinding van losse componenten (potmeter, Rref, LED, BMP280) met PSoC board.  
- **Foto:** ![Breadboard](../pictures/breadboard.png)
