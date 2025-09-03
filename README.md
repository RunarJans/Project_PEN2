# Aangepaste Projectopdracht: PSoC 6 Multimeter & Oscilloscoop met Geïntegreerd TFT-scherm

Dit is een **ModusToolbox**-project voor de **CY8CKIT-062-WiFi-BT (PSoC 6 Pioneer Kit)** met het geïntegreerde **TFT-shield**. De app combineert een **multimeter** (spanning/weerstand/continuïteit), een **oscilloscoop**, een **I²C-scanner** en **digitale pin-tests**. Weergave gebeurt op het TFT – **geen extern display**.

---

## Inhoud
- [Doel en eisen](#doel-en-eisen)
- [Wat is ModusToolbox?](#wat-is-modustoolbox)
- [Functionaliteiten & modi (wat de code nu doet)](#functionaliteiten--modi-wat-de-code-nu-doet)
- [Benodigde hardware](#benodigde-hardware)
- [Aansluitschema (pinnen)](#aansluitschema-pinnen)
- [Modusbediening](#modusbediening)
- [Builden & flashen (ModusToolbox)](#builden--flashen-modustoolbox)
- [Repository-structuur](#repository-structuur)
- [Kalibratie](#kalibratie)
- [Gebruikstips / Troubleshooting](#gebruikstips--troubleshooting)
- [Bekende beperkingen / status](#bekende-beperkingen--status)
- [Externe testsignalen (Pico/Arduino & tweede PSoC)](#externe-testsignalen-picoarduino--tweede-psoc)
- [Veiligheid](#veiligheid)
- [Licentie](#licentie)

---

## Doel en eisen
**Doel:** ontwerp een **digitale multimeter en oscilloscoop** op de PSoC 6, met **grafische weergave op het geïntegreerde TFT**.

**Eisen (samengevat):**
- **Oscilloscoop**: grafische weergave (max ~50 kHz gewenst) + **instelbare tijdbasis (ms/div)**.
- **Multimeter**: **spanning (0–3.3 V)**, **weerstand** via spanningsdeler, **continuïteit**.
- **I²C-scanner**: detecteer apparaten en toon adressen.
- **Digital Pin Tester**: digitale pinnen testen (OUTPUT / INPUT_PULLUP).
- Alles **zonder externe displays**.

---

## Wat is ModusToolbox?
**ModusToolbox** is het Infineon/Cypress ecosysteem voor PSoC-ontwikkeling:
- **BSP/HAL/PDL**: board-, hardware- en driverlagen.
- **getlibs / mtb_shared**: haalt libs binnen (emWin, HAL, drivers).
- **emWin + mtb_st7789v**: GUI-stack en TFT-driver.
- Werkt via **IDE** of **make**: `make getlibs`, `make build`, `make program`.

---

## Functionaliteiten & modi (wat de code nu doet)

De code bevat **8 modi** en gebruikt **P10_0** als gecombineerde ADC-ingang.  
*Belangrijke defines in code:* `SCOPE_SAMPLES=320`, `FS_MAX_HZ=100000`, T/div-presets `{0.25, 0.5, 1, 2, 5, 10} ms/div`.

| # | Modus | Wat je ziet / hoe het werkt | Stand van zaken |
|---|------|------------------------------|-----------------|
| 1 | **Oscilloscoop** | Grafiek van ADC-samples, **T/div** instelbaar (lange druk). Per frame 320 samples. | Werkt. Fs past zich aan T/div aan (max 100 kS/s). |
| 2 | **PWM-scope (ADC extern)** | Probeert **f**, **duty**, **Vpp/Vhigh** uit 320 ADC-samples te schatten. Verwacht nette blokgolf op **P10_0**. | Werkt voor lage kHz; heuristisch. |
| 3 | **Spanningsmeter** | 0–3.3 V, **softwaregemiddelde ×10**, schaal via kalibratie-constanten. | Werkt; nauwkeurigheid = kalibratie. |
| 4 | **Weerstandsmeter** | Deler met **Rref=10 kΩ**: `Rx = Rref * Vnode / (Vcc - Vnode)`. | Werkt; kalibreer `VCC_V` en `RREF_OHM`. |
| 5 | **Continuïteit** | **P10_6** met **INPUT_PULLUP**, **LOW = contact**. | Werkt. |
| 6 | **I²C-scanner** | Scant **0x08..0x77** op **SCL=P6_0, SDA=P6_1** @100 kHz, toont hits. | Werkt (BMP280 op 0x76/0x77). |
| 7 | **Digital Pin Tester** | **P13_6** toggelt elke 500 ms; daarna kort als input met pull-up teruglezen. | Werkt; ideaal met LED + R. |
| 8 | **Digital Pin Monitor** | **P13_7** als **INPUT_PULLUP**; toont HIGH/LOW. | Werkt. |

---

## Benodigde hardware
- **CY8CKIT-062-WiFi-BT** + geïntegreerd **TFT**.
- **Potentiometer 1 kΩ** (ADC-test/kalibratie).
- **Referentieweerstand 10 kΩ** (Rref).
- **BMP280** I²C-sensor (0x76/0x77).
- **LED + serieweerstand** (330 Ω–1 kΩ) voor Digital Pin Tester.
- **Breadboard & jumpers**.

---

## Aansluitschema (pinnen)

| Functie | PSoC-pin | Detail |
|---|---|---|
| ADC (oscilloscoop / volt / weerst / PWM-scope) | **P10_0** | **Max 3.3 V**. Bij PWM-test: potmeter/deler **los**. |
| Board-knop | **CYBSP_USER_BTN** | **Kort**: volgende modus. **Lang**: T/div (alleen osc-modi). |
| Continuïteit | **P10_6** | **INPUT_PULLUP**, **LOW = contact**. |
| I²C | **SCL=P6_0**, **SDA=P6_1** | 100 kHz. Pull-ups 4.7–10 kΩ naar 3V3. |
| Digital Pin Tester (OUTPUT) | **P13_6** | LED + R naar GND. |
| Digital Pin Monitor (INPUT_PULLUP) | **P13_7** | Extern logisch signaal. |
| TFT (emWin) | J2/D-pinnen | In code gemapt via `mtb_st7789v_pins_t`. |

**Altijd** **GND↔GND** verbinden met je externe bron (Pico/andere PSoC).

---

## Modusbediening
- **Korte druk** op **USER BTN** → volgende modus (wrapt 8→1).
- **Lange druk** (>700 ms) in **Oscilloscoop** of **PWM-scope** → volgende **T/div**.
- Header toont links de modus, rechts **T/div** en effectieve **Fs**.

---

## Builden & flashen (ModusToolbox)

### IDE
1. Installeer **ModusToolbox**.
2. **Clone** deze repo.
3. Open via *File → Open* of *Import Existing Application*.
4. Board: **CY8CKIT-062-WIFI-BT**.
5. Build & Program.

### CLI
```bash
make getlibs
make build
make program
```

---

## Repository-structuur

```
.
├─ README.md
├─ CODE_OVERVIEW.md                  # uitleg over architectuur en codeflow
├─ docs/
│  ├─ modes/                         # per-modus uitleg + foto’s/screencaps
│  │  ├─ 01-oscilloscoop.md / .png
│  │  ├─ 02-pwm-scope.md   / .png
│  │  ├─ 03-voltmeter.md   / .png
│  │  ├─ 04-weerstand.md   / .png
│  │  ├─ 05-continuiteit.md/ .png
│  │  ├─ 06-i2c-scanner.md / .png
│  │  ├─ 07-dpin-tester.md / .png
│  │  └─ 08-dpin-monitor.md/ .png
│  └─ hardware/
│     ├─ psoc6-pioneer-kit.jpg
│     ├─ bmp280.jpg
│     ├─ potmeter-1k.jpg
│     ├─ rref-10k.jpg
│     └─ led-test.jpg
├─ examples/                         # externe testgenerators
│  ├─ pico-arduino-blokgolf.ino
│  └─ psoc-pwm-gen.c
├─ source/                           # ModusToolbox project (code, headers)
│  ├─ main.c
│  ├─ tft_task.c                     # UI en modi (emWin) – bevat alle functionaliteit
│  └─ ...
└─ mtb_shared/                       # libs (via getlibs)
```

---

## Kalibratie

Pas deze defines in de code aan je **eigen board** aan:

```c
/* Ruw→Volt mapping */
#define ADC_MIN  20
#define ADC_MAX  4085
#define V_MIN    0.02f
#define V_MAX    3.27f

/* Voeding en referentieweerstand (weerstandsmeter) */
#define VCC_V    3.28f
#define RREF_OHM 10000.0f
```

**Aanpak:**
1. Meet **0 V** en **~3.3 V** op **P10_0** met een DMM → stel `ADC_MIN/MAX` en `V_MIN/MAX`.
2. Meet **VCC_V** precies (bv. 3.28 V) en **RREF_OHM** (bv. 9950 Ω) en vul in.

---

## Gebruikstips / Troubleshooting

- **Vlakke lijn** in oscilloscoop: T/div past niet; **lange druk** voor andere T/div. Check **GND↔GND**. Koppel pot/deler **los** bij PWM-test op **P10_0**.
- **PWM-scope f/duty raar**: te weinig samples per periode; kies **kortere T/div** (hogere Fs) of verlaag bronfrequentie (±1 kHz werkt goed).
- **Weerstandsmeting klopt niet**: kalibreer **VCC_V** en **RREF_OHM**; vermijd uitersten (Rx ≪/≫ Rref).
- **I²C**: zorg voor pull-ups; BMP280 gebruikt **0x76/0x77**.

---

## Bekende beperkingen / status

- **50 kHz grafisch**: met `FS_MAX_HZ=100 kS/s` zijn er ~2 samples/periode @50 kHz ⇒ **nauwelijks detail**. De grafiek is bedoeld voor lage kHz. Voor echte HF-weergave is **ADC+DMA** met hogere Fs en triggers nodig (niet geïmplementeerd).
- **PWM-frequentiebepaling** via ADC-samples is **heuristiek** en T/div-afhankelijk.
- Nauwkeurigheid **volt/weerstand** hangt direct af van je kalibratie.

---

## Externe testsignalen (Pico/Arduino & tweede PSoC)

**Waarom**: om de oscilloscoop/PWM-scope te valideren zonder lab-signaalgenerator.

### 1) Raspberry Pi Pico (Arduino IDE) – eenvoudige blokgolf

Gebruik dit om **P13_7 (Digital Pin Monitor)** of **P10_0 (PWM-scope via ADC)** te testen.

```cpp
// Pin kiezen, bijvoorbeeld GP2
const int testPin = 2;

void setup() {
  pinMode(testPin, OUTPUT);  // Zet pin als output
}

void loop() {
  digitalWrite(testPin, HIGH);  // 3.3V
  delay(1000);                  // 1 seconde
  digitalWrite(testPin, LOW);   // 0V
  delay(1000);                  // 1 seconde
}
```

> Dit is ~0,5 Hz → te traag voor korte T/div. Wil je een nette blok bij de PWM-scope, maak ~1 kHz:

```cpp
const int testPin = 2;

void setup() { pinMode(testPin, OUTPUT); }

void loop() {
  digitalWrite(testPin, HIGH); delayMicroseconds(500); // ~1 kHz, 50% duty
  digitalWrite(testPin, LOW);  delayMicroseconds(500);
}
```

**Aansluiten:** `GP2 → P10_0` (voor PWM-scope via ADC) of `GP2 → P13_7` (digitale monitor), **GND↔GND**.

---

### 2) Tweede PSoC – PWM-generator met HAL

Gebruik dit om een PWM in te sturen op **P10_0** of als logisch signaal op **P13_7**.  
**Let op:** in dit voorbeeld is de **frequentieparameter** de *tweede* parameter; standaard hieronder op **1 Hz**. Voor ~**1 kHz** zet je die op **1000**.

```c
#include "cyhal.h"
#include "cybsp.h"

int main(void)
{
    cyhal_pwm_t pwm_obj;

    cybsp_init();
    __enable_irq();

    cyhal_pwm_init(&pwm_obj, P0_3, NULL);
    cyhal_pwm_set_duty_cycle(&pwm_obj, 75, 1);   // duty=75%, freq=1 Hz
    // Voor 1 kHz test: cyhal_pwm_set_duty_cycle(&pwm_obj, 50, 1000);

    cyhal_pwm_start(&pwm_obj);

    for (;;) { }
}
```

**Aansluiten:** `P0_3 (PWM) → P10_0` of `P13_7`, **GND↔GND**.  
**Tip:** voor de huidige T/div-stappen is **~1 kHz** beter zichtbaar dan 1 Hz.

---

## Veiligheid
- **Nooit > 3.3 V** direct op **P10_0**.
- Gebruik een serieweerstand bij LED-tests.
- Koppel de potmeter/deler **los** bij het inmeten van externe PWM op **P10_0**.

---

## Licentie
Kies en voeg een licentie toe (bijv. **MIT** of **BSD-3-Clause**).
