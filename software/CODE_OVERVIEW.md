# Code Overview – PSoC6 Multimeter & Oscilloscoop

Dit document legt de volledige code uit van het ModusToolbox-project.  
Het project draait op de **PSoC 6 Pioneer Kit (CY8CKIT-062-WiFi-BT)** met **ST7789V TFT-shield** en gebruikt **FreeRTOS** voor de taakafhandeling.

---

## 📥 Download

➡️ Voor de complete ModusToolbox-projectbestanden (ZIP):  
Ga naar de **[Releases](../../releases)** pagina van de repo en download de laatste release-asset.

---

## ⚙️ Bestanden & structuur

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

## 🔌 Pinnen & constanten

- **ADC_INPUT_PIN** = `P10_0` → gedeeld voor **osc/voltmeter/weerstand**.  
- **BTN_PIN** = `CYBSP_USER_BTN` → **kort** = volgende modus, **lang** = T/div wisselen.  
- **CONTINUITY_PIN** = `P10_6` → input met pull-up; **LOW = contact**.  
- **DPIN_TEST_PIN** = `P13_6` → output-toggle + teruglezen.  
- **DPIN_MONITOR_PIN** = `P13_7` → pure input-monitor.

ADC-kalibratie (pas aan op jouw board):
```c
#define ADC_MIN 20    // raw bij 0 V
#define ADC_MAX 4085  // raw bij ~3.27 V
#define V_MIN   0.02f
#define V_MAX   3.27f
```
→ hiermee mapt `adc_raw_to_v()` de ADC-raw naar volt.

I²C (100 kHz) gebruikt **P6_0 (SCL)** en **P6_1 (SDA)** voor de scanner.

Display: **320×240**, oscilloscoop gebruikt **320 samples** breed.

Sample-timer: **max 100 kHz**, start op **20 kHz**.

---

## 🧠 Globale state & buffers

- `g_mode` → actieve UI-modus.  
- `g_ms_div_presets` → T/div presets (0.25–10 ms/div).  
- `s_scope_buf[320]` → ringbuffer met ADC-samples voor de scope.  
- `s_sample_tick` → vlag uit timer-ISR dat er een samplemoment is.

---

## 📟 TFT initialisatie & tekenen

Initialisatie:
```c
mtb_st7789v_init8(&tft_pins);
GUI_Init();
GUI_SetBkColor(GUI_BLACK);
GUI_Clear();
```

Tekenen van UI:
- **`header(left, right)`** → zwarte balk met witte tekst.  
- **`draw_grid()`** → rasterlijnen voor scope.  
- Scope-trace: met **`GUI_DrawLine()`** tussen opeenvolgende samples.

---

## ⏱ Sample-timer & T/div

```c
cyhal_timer_init(&s_sample_tmr, NC, NULL);
cyhal_timer_set_frequency(&s_sample_tmr, hz);
cyhal_timer_register_callback(&s_sample_tmr, isr_sample, NULL);
cyhal_timer_enable_event(&s_sample_tmr, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);
```
- De ISR zet `s_sample_tick = true`.  
- **`apply_timebase()`** berekent **Fs** op basis van het venster (10 divisies × ms/div) zodat de scopebreedte (320 punten) netjes de gekozen T/div bestrijkt.

---

## 🧮 ADC-helpers

- **`adc_read_avg10()`**: gemiddelde van 10 samples voor **rustige metingen** (voltmeter/weerstand).  
- **`cyhal_adc_read()`** direct: voor **scherpe edges** in de scope/PWM-scope.  
- **`adc_raw_to_v()`**: mapt raw → volt via jouw kalibratie.

---

## 🔘 Button-logica

- **Korte druk** → volgende modus (met wrap naar eerste).  
- **Lange druk (>700 ms)** → volgende **T/div preset** en herconfigureer sample-timer.  
Debounce met `DEBOUNCE_MS`.

---

## 📊 Modus-implementaties

### 1) Oscilloscoop – `mode_osc_step()`
- Leest ADC (gemiddeld) en schrijft naar `s_scope_buf`.  
- Wanneer de buffer vol is: scherm wissen, grid tekenen en trace tekenen.  
- Rechtsboven toont `format_right_header()` de **T/div** en **Fs**.

### 2) PWM-scope – `mode_osc_pwm_step()`
- Externe PWM op **`P10_0`** (denk aan **GND↔GND**).  
- Bij eerste keer wordt T/div kort gezet (0.5 ms/div) via `apply_timebase()`.  
- Leest samples zonder averaging.  
- Berekent:
  - **`vmax`/`vmin`** → **`Vpp`** en **`Vhigh`**.  
  - Twee **stijgende flanken** met hysterese → **periode** → **frequentie**.  
  - **Duty-cycle** uit `high_samples / period_samples`.  
- Toont trace + overlay: **f**, **duty**, **Vhigh**, **Vpp**.  
  > Als geen stabiele edges: hint om T/div te verkorten of signaal te checken.

### 3) Spanningsmeter – `mode_volt_step()`
- Gemiddelde van 10 samples.  
- Tekst met **V** + blauwe **balk** (0–VCC).

### 4) Weerstandsmeter – `mode_res_step()`
- Schema: `3V3 —[Rref]—●—[Rx]—GND`, ADC meet in de **node ●**.  
- Formule: `Rx = Rref * Vnode / (Vcc − Vnode)`.  
- Toont automatisch in Ω, kΩ of MΩ, met open/short detectie.

### 5) Continuïteit – `mode_cont_step()`
- Init `P10_6` als **INPUT_PULLUP** (1).  
- **LOW** → “Contact!” in groen, anders “Geen contact” in rood.

### 6) I²C-scanner – `mode_i2c_step()`
- Init I²C master op 100 kHz.  
- Loopt adressen **0x08..0x77** af, probeert 0-byte write of 1-byte read.  
- Tekent 8×16 tabel met **gevonden adressen**.

### 7) Digital Pin Tester – `mode_dpin_step()`
- `P13_6` als **output**, toggelt elke 500 ms.  
- Lees hem **kort als input** (met pull-up) om de status te tonen.  
- Zet hem terug als output (met huidige state).

### 8) Digital Pin Monitor – `mode_dpin_monitor_step()`
- `P13_7` als **INPUT_PULLUP**.  
- Toont live HIGH/LOW.

---

## 🔄 FreeRTOS-taak

Hoofdloop kiest per `g_mode` de juiste `mode_*_step()` en voegt kleine `vTaskDelay()` toe per modus om CPU/TFT te sparen.  
Tussen de calls door geeft `taskYIELD()` andere taken kans (als aanwezig).

---

## 🧪 Test-tips

- **Oscilloscoop/PWM**: hang een PWM-bron op `P10_0`, verbind **GND↔GND**.  
- **Voltmeter**: gebruik de potmeter op de ADC-ingang.  
- **Weerstand**: meet via spanningsdeler met bekende `Rref`.  
- **I²C**: sluit bijv. **BMP280 (0x76/0x77)** aan met pull-ups naar 3V3.

---

## 🔗 Verwante bestanden

- Hardwareuitleg: [`../hardware/hardware.md`](../hardware/hardware.md)  
- Foto’s per modus: [`../pictures/`](../pictures)  
- Releases/ZIP-download: zie **[Releases](../../releases)**

---

## ✅ Samenvatting

Deze code laat zien hoe je met **PSoC6 + FreeRTOS + emWin** een compacte **multimeter + oscilloscoop + I²C-scanner + digitale pin-tools** bouwt met een geïntegreerd TFT-scherm.
