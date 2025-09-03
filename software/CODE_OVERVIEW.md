
# Code Overview – Project_PEN2

Dit bestand beschrijft de structuur, werking en functionaliteit van de software in ModusToolbox.

## Structuur van het project

| Bestand | Functie |
|--------|---------|
| `main.c` | Initieert hardware en start `tft_task()` |
| `tft_task.c` | Hoofdlogica, bedient emWin UI, schakelt tussen modi |
| `tft_task.h` | Header met mode-definities en prototypes |
| `modustoolbox.mk` | Build-configuratie voor board en componenten |

## Grafische output: TFT-scherm

- **Aangestuurd via:** `mtb_st7789v` + `emWin` (GUI lib).
- **Initialisatie:** via `GUI_Init()` en `GUI_Clear()`.
- **Tekst weergeven:** `GUI_DispStringAt(text, x, y)`.
- **Rechthoeken/grafiek tekenen:** `GUI_DrawRect`, `GUI_DrawLine`, `GUI_DrawGraph`.
- **Schermgrootte**: 320x240 px (landscape).

## Modusbeheer (Modes)

Modi worden cyclisch doorlopen met korte knopdruk (CYBSP_USER_BTN).
Elke modus heeft een unieke ID (0–7) en een aparte `render_XXX()` functie in `tft_task.c`.

- **Oscilloscoop (0):** Leest 320 ADC-samples op P10_0 en tekent lijnvormig signaal.
- **PWM Scope (1):** Detecteert frequentie en duty cycle door analyse van ADC-grafiek.
- **Spanningsmeter (2):** Berekent spanning op P10_0 en toont deze numeriek.
- **Weerstandsmeter (3):** Gebruikt spanningsdeler om Rx te berekenen en tonen.
- **Continuïteit (4):** Toont “VERBONDEN” als P10_6 LOW is.
- **I²C Scanner (5):** Doorloopt adresruimte 0x08..0x77 en toont respons op display.
- **Digital Pin Tester (6):** Zet P13_6 HIGH/LOW → LED moet knipperen.
- **Digital Pin Monitor (7):** Leest toestand van P13_7 (digitaal) en toont status.

## Sampling & ADC

- ADC loopt via polling (`cyhal_adc_read_uv()`).
- Er wordt een array van 320 waarden gevuld voor de oscilloscoop.
- Spanning wordt omgerekend met kalibratieconstants in `tft_task.c`.

## Instelbare tijdbasis (T/div)

- Lang drukken op knop (>700 ms) verandert `t_div_ms` (tijd per divisie).
- Wordt toegepast in de oscilloscoopmodus om de sample rate aan te passen.

## Kalibratieparameters

```c
#define VCC_V 3.28f
#define RREF_OHM 10000.0f
#define ADC_MIN  20
#define ADC_MAX  4085
#define V_MIN    0.02f
#define V_MAX    3.27f
```

- Essentieel voor correcte spannings- en weerstandsmetingen.
