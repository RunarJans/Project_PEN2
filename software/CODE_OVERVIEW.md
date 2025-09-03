# Code Overview – PSoC6 Multimeter & Oscilloscoop

Dit document legt de volledige code uit van het ModusToolbox-project.  
Het project draait op de **PSoC 6 Pioneer Kit (CY8CKIT-062-WiFi-BT)** met **ST7789V TFT-shield** en gebruikt **FreeRTOS** voor de taakafhandeling.

---

## Download

Voor de complete ModusToolbox-projectbestanden (ZIP):  
Ga naar de [Releases](https://github.com/RunarJans/Project_PEN2/releases/tag/v1.0.0) pagina van de repo en download de laatste release-asset.

---

## Bestanden & structuur

Belangrijkste bronbestand: **`tft_task.c`**.  
Verder in de repo:
- **`hardware/`** – componenten & wiring uitleg.  
- **`pictures/`** – screenshots per modus (voor README’s).  
- **`software/`** – deze uitleg en (optioneel) tools of exports.  

### Belangrijkste includes
```c
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "GUI.h"
#include "mtb_st7789v.h"
#include "FreeRTOS.h"
#include "task.h"
```
- **cy_pdl / cyhal / cybsp**: PSoC drivers, HAL en board support.  
- **GUI.h**: emWin (Segger) GUI-bibliotheek.  
- **mtb_st7789v.h**: driver/shield mapping voor het TFT.  
- **FreeRTOS**: scheduler en timing.

---

## Pinnen & constanten

- **ADC_INPUT_PIN** = `P10_0` → gedeeld voor osc/voltmeter/weerstand.  
- **BTN_PIN** = `CYBSP_USER_BTN` → korte druk = volgende modus, lange druk = T/div wisselen.  
- **CONTINUITY_PIN** = `P10_6` → input met pull-up; LOW = contact.  
- **DPIN_TEST_PIN** = `P13_6` → output-toggle + teruglezen.  
- **DPIN_MONITOR_PIN** = `P13_7` → pure input-monitor.

ADC-kalibratie (pas aan op jouw board):
```c
#define ADC_MIN 20    // raw bij 0 V
#define ADC_MAX 4085  // raw bij ~3.27 V
#define V_MIN   0.02f
#define V_MAX   3.27f
```

I²C (100 kHz) gebruikt **P6_0 (SCL)** en **P6_1 (SDA)** voor de scanner.  
Display: 320×240, oscilloscoop gebruikt 320 samples breed.  
Sample-timer: maximaal 100 kHz, start op 20 kHz.

---

## Globale state & buffers

- `g_mode` → actieve UI-modus.  
- `g_ms_div_presets` → T/div presets (0.25–10 ms/div).  
- `s_scope_buf[320]` → ringbuffer met ADC-samples voor de scope.  
- `s_sample_tick` → vlag uit timer-ISR dat er een samplemoment is.

---

## TFT initialisatie & tekenen

Initialisatie:
```c
mtb_st7789v_init8(&tft_pins);
GUI_Init();
GUI_SetBkColor(GUI_BLACK);
GUI_Clear();
```

UI-elementen:
- `header(left, right)` → zwarte balk met witte tekst.  
- `draw_grid()` → rasterlijnen voor scope.  
- Scope-trace: `GUI_DrawLine()` tussen opeenvolgende samples.

---

## Sample-timer & T/div

```c
cyhal_timer_init(&s_sample_tmr, NC, NULL);
cyhal_timer_set_frequency(&s_sample_tmr, hz);
cyhal_timer_register_callback(&s_sample_tmr, isr_sample, NULL);
cyhal_timer_enable_event(&s_sample_tmr, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);
```

De ISR zet `s_sample_tick = true`.  
`apply_timebase()` berekent Fs op basis van het venster (10 divisies × ms/div) zodat de scopebreedte (320 punten) overeenkomt met de gekozen T/div.

---

## ADC-helpers

- `adc_read_avg10()` → gemiddelde van 10 samples (rustige metingen).  
- `cyhal_adc_read()` → directe sample (scherpe edges, PWM).  
- `adc_raw_to_v()` → raw ADC → volt via kalibratie.

---

## Button-logica

- **Korte druk**: volgende modus (wrap naar begin).  
- **Lange druk (>700 ms)**: volgende T/div preset, herconfigureert sample-timer.  
- Debounce met `DEBOUNCE_MS`.

---

## Modus-implementaties

### 1) Oscilloscoop – `mode_osc_step()`
Leest ADC, vult buffer, tekent grid en trace. Toont T/div en Fs in header.

### 2) PWM-scope – `mode_osc_pwm_step()`
Externe PWM op P10_0. Analyseert frequentie, duty-cycle, Vpp, Vhigh en toont overlay.

### 3) Spanningsmeter – `mode_volt_step()`
Meet spanning, toont waarde in V en blauwe balkmeter.

### 4) Weerstandsmeter – `mode_res_step()`
Meet Rx via spanningsdeler. Toont in Ω, kΩ of MΩ met open/short detectie.

### 5) Continuïteit – `mode_cont_step()`
LOW op P10_6 = contact. Toont in groen/rood.

### 6) I²C-scanner – `mode_i2c_step()`
Probeert adressen 0x08–0x77, toont tabel met gevonden devices.

### 7) Digital Pin Tester – `mode_dpin_step()`
Toggelt P13_6 elke 500 ms en leest status terug.

### 8) Digital Pin Monitor – `mode_dpin_monitor_step()`
Toont status van P13_7 live (HIGH/LOW).

---

## FreeRTOS-taak

De hoofdtaak selecteert op basis van `g_mode` de juiste `mode_*_step()`.  
Elke modus heeft een korte `vTaskDelay()` om CPU en TFT te sparen.  
`taskYIELD()` geeft andere FreeRTOS-taken de kans om te draaien.

---

## Test-tips

- Oscilloscoop/PWM: verbind externe PWM met P10_0 en GND↔GND.  
- Voltmeter: gebruik de potmeter op het shield.  
- Weerstand: meet via spanningsdeler met bekende Rref.  
- I²C: sluit BMP280 aan (adres 0x76/0x77) met 4.7k pull-ups.

---

## Verwante bestanden

- Hardwareuitleg: [`../hardware/hardware.md`](../hardware/hardware.md)  
- Foto’s per modus: [`../pictures/`](../pictures)  
- Release ZIP-download: [Releases](https://github.com/RunarJans/Project_PEN2/releases/tag/v1.0.0)

---

## Samenvatting

Deze code demonstreert hoe je met **PSoC6, FreeRTOS en emWin** een geïntegreerde multimeter, oscilloscoop, I²C-scanner en digitale pin-tools op een TFT-scherm kunt realiseren.
