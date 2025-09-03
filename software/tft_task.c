#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "GUI.h"
#include "mtb_st7789v.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* ===== Pins ===== */
#define ADC_INPUT_PIN       P10_0               /* gedeeld voor spanning/weerstand/osc */
#define BTN_PIN             CYBSP_USER_BTN      /* korte druk = modus, lange druk = T/div (osc) */
#define CONTINUITY_PIN      P10_6               /* continuïteit: INPUT_PULLUP, LOW = contact */

#define DPIN_TEST_PIN       P13_6               /* Digital Pin Tester (output toggle + teruglezen) */
#define DPIN_MONITOR_PIN    P13_7               /* Digital Pin Monitor (pure input) */

/* === ADC kalibratie (pas aan op jouw board) === */
#define ADC_MIN  20      // gemeten raw bij 0 V
#define ADC_MAX  4085    // gemeten raw bij ~3.27 V
#define V_MIN    0.02f   // gemeten spanning laag
#define V_MAX    3.27f   // gemeten spanning hoog

/* I2C (geïntegreerde SCL/SDA) */
#define I2C_SCL             P6_0
#define I2C_SDA             P6_1
#define I2C_FREQ_HZ         100000u
#define I2C_TIMEOUT_MS      10u
#define I2C_ADDR_MIN        0x08
#define I2C_ADDR_MAX        0x77

/* ===== Display ===== */
#define DISP_W              320
#define DISP_H              240
#define SCOPE_SAMPLES       DISP_W

/* ===== Sample timer & UI ===== */
#define FS_MAX_HZ           100000u
#define SAMPLE_START_HZ     20000u
#define DEBOUNCE_MS         60u
#define LONGPRESS_MS        700u

/* ===== Meetconstanten (voor weerstandsmeting) ===== */
#define VCC_V               3.28f               /* zet hier je GEMETEN 3V3 (bv 3.28) */
#define RREF_OHM            10000.0f            /* gemeten referentie (bv 9950) */
#define RX_MIN_OHM          0.1f

/* Modi: oscilloscoop, PWM-scope(ADC extern), voltmeter, weerstand, continuïteit, I2C-scanner, pin test, pin monitor */
typedef enum {
    MODE_OSC = 0,
    MODE_OSC_PWM,   /* PWM meten via ADC op P10_0 (externe bron, bv. Pico) */
    MODE_VOLT,
    MODE_RES,
    MODE_CONT,
    MODE_I2C,
    MODE_DPIN,        /* P13_6: output toggle + teruglezen */
    MODE_DPIN_MON,    /* P13_7: alleen input monitoren */
    MODE_COUNT
} ui_mode_t;

static volatile ui_mode_t g_mode = MODE_OSC;

/* T/div presets (ms/div), 10 divisies breed */
static const float g_ms_div_presets[] = { 0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f };
#define TB_COUNT (sizeof(g_ms_div_presets)/sizeof(g_ms_div_presets[0]))
static uint8_t  g_tb_index   = 2;              /* start 1.0 ms/div */
static uint32_t g_fs_current = SAMPLE_START_HZ;
static float    g_ms_div_eff = 1.0f;

/* TFT mapping (zoals je werkende project) */
static const mtb_st7789v_pins_t tft_pins = {
    .db08 = CYBSP_J2_2,  .db09 = CYBSP_J2_4,
    .db10 = CYBSP_J2_6,  .db11 = CYBSP_J2_10,
    .db12 = CYBSP_J2_12, .db13 = CYBSP_D7,
    .db14 = CYBSP_D8,    .db15 = CYBSP_D9,
    .nrd  = CYBSP_D10,   .nwr  = CYBSP_D11,
    .dc   = CYBSP_D12,   .rst  = CYBSP_D13
};

/* HAL objecten */
static cyhal_adc_t         s_adc;
static cyhal_adc_channel_t s_adc_ch;
static cyhal_timer_t       s_sample_tmr;
static cyhal_i2c_t         s_i2c;

/* ISR vlag */
static volatile bool s_sample_tick = false;

/* Oscilloscoop buffers (raw ADC, -2048..+2047) */
static int16_t  s_scope_buf[SCOPE_SAMPLES];
static uint32_t s_scope_idx = 0;
static bool     s_scope_ready = false;

/* Continuïteit init-flag */
static bool     s_cont_pin_inited = false;

/* I2C-scan cache */
static uint8_t  s_i2c_found[128];
static uint8_t  s_i2c_found_cnt = 0;
static TickType_t s_i2c_next_scan = 0;

/* ===== Helpers ===== */
/* Correcte ADC-conversie: -2048..+2047 -> 0..4095 -> kalibratie naar V */
static inline float adc_raw_to_v(int16_t raw)
{
    int32_t unsigned_val = raw + 2048;   // 0..4095
    if (unsigned_val < ADC_MIN) unsigned_val = ADC_MIN;
    if (unsigned_val > ADC_MAX) unsigned_val = ADC_MAX;

    float frac = (unsigned_val - ADC_MIN) / (float)(ADC_MAX - ADC_MIN);
    return V_MIN + frac * (V_MAX - V_MIN);
}

static int16_t adc_read_avg10(cyhal_adc_channel_t* ch)
{
    int32_t acc = 0;
    for (int i = 0; i < 10; ++i) { acc += cyhal_adc_read(ch); }
    return (int16_t)(acc / 10);
}

static void header(const char* left, const char* right)
{
    GUI_SetColor(GUI_BLACK); GUI_FillRect(0, 0, DISP_W, 29);
    GUI_SetColor(GUI_WHITE); GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT);  GUI_DispStringAt(left, 6, 6);
    if (right) { GUI_SetTextAlign(GUI_TA_RIGHT); GUI_DispStringAt(right, DISP_W - 6, 6); }
}

static void format_right_header(char* out, size_t n, float ms_div_eff, uint32_t fs_hz)
{
    if (ms_div_eff < 1.0f) {
        float us = ms_div_eff * 1000.0f;
        snprintf(out, n, "T/div=%.0f us  Fs=%.1f kS/s", us, fs_hz / 1000.0f);
    } else {
        snprintf(out, n, "T/div=%.2f ms  Fs=%.1f kS/s", ms_div_eff, fs_hz / 1000.0f);
    }
}

/* ===== ISRs ===== */
static void isr_sample(void* arg, cyhal_timer_event_t ev)
{
    (void)arg; (void)ev;
    s_sample_tick = true;
}

/* ===== Sample timer ===== */
static void init_sample_timer(uint32_t hz)
{
    cy_rslt_t r;
    cyhal_timer_cfg_t cfg = (cyhal_timer_cfg_t){0};
    cfg.is_continuous = true;
    cfg.is_compare    = false;
    cfg.direction     = CYHAL_TIMER_DIR_UP;
    cfg.period        = 0;
    cfg.compare_value = 0;
    cfg.value         = 0;

    r = cyhal_timer_init(&s_sample_tmr, NC, NULL);            CY_ASSERT(r == CY_RSLT_SUCCESS);
    r = cyhal_timer_configure(&s_sample_tmr, &cfg);           CY_ASSERT(r == CY_RSLT_SUCCESS);
    r = cyhal_timer_set_frequency(&s_sample_tmr, hz);         CY_ASSERT(r == CY_RSLT_SUCCESS);
    cyhal_timer_register_callback(&s_sample_tmr, isr_sample, NULL);
    cyhal_timer_enable_event(&s_sample_tmr, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);
    r = cyhal_timer_start(&s_sample_tmr);                     CY_ASSERT(r == CY_RSLT_SUCCESS);
}

static void apply_timebase(uint8_t tb_index)
{
    if (tb_index >= TB_COUNT) { tb_index = 0; }
    g_tb_index = tb_index;

    float ms_div = g_ms_div_presets[g_tb_index];
    float window_s_desired = (ms_div * 10.0f) / 1000.0f;      /* 10 divisies breed */
    float fs_desired = (float)SCOPE_SAMPLES / window_s_desired;

    uint32_t fs_target = (uint32_t)roundf(fs_desired);
    if (fs_target > FS_MAX_HZ) { fs_target = FS_MAX_HZ; }

    (void)cyhal_timer_set_frequency(&s_sample_tmr, fs_target);
    g_fs_current = fs_target;

    float window_s_eff = (float)SCOPE_SAMPLES / (float)g_fs_current;
    g_ms_div_eff = (window_s_eff / 10.0f) * 1000.0f;
}

/* ===== Tekenhulp (oscilloscoop) ===== */
static void draw_grid(void)
{
    GUI_SetColor(GUI_GRAY);
    for (int i = 1; i < 10; ++i) {
        int x = (DISP_W * i) / 10;
        GUI_DrawLine(x, 30, x, DISP_H - 1);
    }
    for (int j = 1; j < 10; ++j) {
        int y = 30 + ((DISP_H - 30) * j) / 10;
        GUI_DrawLine(0, y, DISP_W - 1, y);
    }
}

/* ===== Modus: Oscilloscoop (analoge ADC) ===== */
static void mode_osc_step(void)
{
    if (!s_sample_tick) { return; }
    s_sample_tick = false;

    int16_t s = adc_read_avg10(&s_adc_ch);
    s_scope_buf[s_scope_idx++] = s;
    if (s_scope_idx >= SCOPE_SAMPLES) {
        s_scope_idx = 0;
        s_scope_ready = true;
    }

    if (s_scope_ready) {
        s_scope_ready = false;

        char right[48];
        format_right_header(right, sizeof(right), g_ms_div_eff, g_fs_current);
        header("Modus: Oscilloscoop", right);

        GUI_SetColor(GUI_BLACK);
        GUI_FillRect(0, 30, DISP_W, DISP_H);
        draw_grid();

        GUI_SetColor(GUI_GREEN);
        for (int i = 0; i < (SCOPE_SAMPLES - 1); ++i) {
            int y1 = DISP_H - ((s_scope_buf[i]   + 2048) * (DISP_H - 30) / 4096);
            int y2 = DISP_H - ((s_scope_buf[i+1] + 2048) * (DISP_H - 30) / 4096);
            if (y1 < 30) { y1 = 30; } if (y1 >= DISP_H) { y1 = DISP_H - 1; }
            if (y2 < 30) { y2 = 30; } if (y2 >= DISP_H) { y2 = DISP_H - 1; }
            GUI_DrawLine(i, y1, i + 1, y2);
        }
    }
}

/* ===== Modus: Oscilloscoop PWM (externe bron op P10_0) ===== */
static void mode_osc_pwm_step(void)
{
    /* Eerste keer: kortere T/div en buffer schoon */
    static bool init_done = false;
    if (!init_done) {
        /* Kort venster voor ~1 kHz: 0.5 ms/div (index 1) */
        g_tb_index = 1;              /* 0:0.25ms, 1:0.5ms, 2:1.0ms, ... */
        apply_timebase(g_tb_index);
        s_scope_idx = 0;
        s_scope_ready = false;
        init_done = true;
    }

    if (!s_sample_tick) { return; }
    s_sample_tick = false;

    /* Geen averaging: we willen scherpe edges zien */
    int16_t raw = cyhal_adc_read(&s_adc_ch);
    s_scope_buf[s_scope_idx++] = raw;
    if (s_scope_idx >= SCOPE_SAMPLES) {
        s_scope_idx = 0;
        s_scope_ready = true;
    }

    if (s_scope_ready) {
        s_scope_ready = false;

        /* Analyseer amplitude en edges */
        float vmin =  1e9f, vmax = -1e9f;
        for (int i = 0; i < SCOPE_SAMPLES; ++i) {
            float v = adc_raw_to_v(s_scope_buf[i]);
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
        float vpp = vmax - vmin;

        /* Hysterese rond midden om ruis te negeren */
        float thr = (vmax + vmin) * 0.5f;
        float hys = fmaxf(0.05f * vpp, 0.02f);   /* 5% Vpp of min 20mV */
        float thr_low  = thr - hys;
        float thr_high = thr + hys;

        int first_rise = -1, second_rise = -1, fall_after_first = -1;
        for (int i = 1; i < SCOPE_SAMPLES; ++i) {
            float vp = adc_raw_to_v(s_scope_buf[i-1]);
            float vq = adc_raw_to_v(s_scope_buf[i]);
            if (first_rise < 0 && vp < thr_low && vq > thr_high) {
                first_rise = i;
            } else if (first_rise >= 0 && second_rise < 0 && vp < thr_low && vq > thr_high) {
                second_rise = i; break;
            }
            if (first_rise >= 0 && fall_after_first < 0 && vp > thr_high && vq < thr_low) {
                fall_after_first = i;
            }
        }

        float f_hz = 0.0f, duty_pc = -1.0f;
        if (first_rise >= 0 && second_rise > first_rise) {
            int period_samples = (second_rise - first_rise);
            if (period_samples > 0) {
                f_hz = ((float)g_fs_current) / (float)period_samples;
                if (fall_after_first > first_rise && fall_after_first < second_rise) {
                    int high_samples = (fall_after_first - first_rise);
                    duty_pc = 100.0f * ((float)high_samples / (float)period_samples);
                }
            }
        }

        /* Tekenen */
        char right[64];
        format_right_header(right, sizeof(right), g_ms_div_eff, g_fs_current);
        header("Modus: PWM-scope (extern op P10_0)", right);

        GUI_SetColor(GUI_BLACK);
        GUI_FillRect(0, 30, DISP_W, DISP_H);
        draw_grid();

        GUI_SetColor(GUI_GREEN);
        for (int i = 0; i < (SCOPE_SAMPLES - 1); ++i) {
            int y1 = DISP_H - ((s_scope_buf[i]   + 2048) * (DISP_H - 30) / 4096);
            int y2 = DISP_H - ((s_scope_buf[i+1] + 2048) * (DISP_H - 30) / 4096);
            if (y1 < 30) { y1 = 30; } if (y1 >= DISP_H) { y1 = DISP_H - 1; }
            if (y2 < 30) { y2 = 30; } if (y2 >= DISP_H) { y2 = DISP_H - 1; }
            GUI_DrawLine(i, y1, i + 1, y2);
        }

        /* Overlay: metingen + hints */
        GUI_SetColor(GUI_WHITE);
        GUI_SetFont(GUI_FONT_16B_1);
        char l1[96], l2[96];
        if (f_hz > 0.0f) {
            if (duty_pc >= 0.0f)
                snprintf(l1, sizeof(l1), "f \xE2\x89\x88 %.1f Hz (%.3f kHz), duty \xE2\x89\x88 %.1f%%",
                         f_hz, f_hz/1000.0f, duty_pc);
            else
                snprintf(l1, sizeof(l1), "f \xE2\x89\x88 %.1f Hz (%.3f kHz), duty: n/a",
                         f_hz, f_hz/1000.0f);
        } else {
            snprintf(l1, sizeof(l1), "Geen stabiele edges: kortere T/div of signaal checken");
        }
        snprintf(l2, sizeof(l2), "Vhigh \xE2\x89\x88 %.3f V   Vpp \xE2\x89\x88 %.3f V", vmax, vpp);

        GUI_DispStringAt("Bron: externe PWM -> P10_0 (GND↔GND, pot/deler loskoppelen)", 8, 34);
        GUI_DispStringAt(l1, 8, 54);
        GUI_DispStringAt(l2, 8, 74);
    }
}

/* ===== Modus: Spanningsmeter ===== */
static void mode_volt_step(void)
{
    if (!s_sample_tick) { return; }
    s_sample_tick = false;

    int16_t s = adc_read_avg10(&s_adc_ch);
    float v = adc_raw_to_v(s);

    header("Modus: Spanningsmeter", NULL);

    GUI_SetColor(GUI_BLACK);
    GUI_FillRect(0, 30, DISP_W, DISP_H);

    GUI_SetColor(GUI_WHITE);
    GUI_SetFont(GUI_FONT_24_ASCII);
    char buf[48];
    snprintf(buf, sizeof(buf), "Spanning: %.3f V", v);
    GUI_DispStringAt(buf, 40, 70);

    int bar_len = (int)(v / VCC_V * (DISP_W - 80));
    if (bar_len < 0) { bar_len = 0; }
    GUI_SetColor(GUI_BLUE);
    GUI_FillRect(40, 130, 40 + bar_len, 165);
    GUI_SetColor(GUI_WHITE);
    GUI_DrawRect(40, 130, DISP_W - 40, 165);
}

/* ===== Modus: Weerstand (Rref-deler) =====
   Schema: 3.3V -[Rref]-●-[Rx]-GND, ADC = ●
   Rx = Rref * Vnode / (Vcc - Vnode)
*/
static void mode_res_step(void)
{
    if (!s_sample_tick) { return; }
    s_sample_tick = false;

    int16_t raw = adc_read_avg10(&s_adc_ch);
    float v_node = adc_raw_to_v(raw);

    header("Modus: Weerstand (Rref)", NULL);

    GUI_SetColor(GUI_BLACK);
    GUI_FillRect(0, 30, DISP_W, DISP_H);

    GUI_SetColor(GUI_WHITE);
    GUI_SetFont(GUI_FONT_16B_1);
    char info[64];
    snprintf(info, sizeof(info), "Vnode=%.3f V   Vcc=%.2f V", v_node, VCC_V);
    GUI_DispStringAt(info, 10, 40);

    if (v_node <= 0.02f) {
        GUI_SetFont(GUI_FONT_24_ASCII);
        GUI_SetColor(GUI_RED);
        GUI_DispStringAt("Rx ~ 0 \xE2\x84\xA6 (kort?)", 40, 90);
    }
    else if (v_node >= (VCC_V - 0.02f)) {
        GUI_SetFont(GUI_FONT_24_ASCII);
        GUI_SetColor(GUI_RED);
        GUI_DispStringAt("Rx >> Rref (open?)", 40, 90);
    }
    else {
        float denom = (VCC_V - v_node);
        if (denom < 0.001f) { denom = 0.001f; }
        float rx = RREF_OHM * (v_node / denom);
        if (rx < RX_MIN_OHM) { rx = RX_MIN_OHM; }

        char line[64];
        GUI_SetFont(GUI_FONT_24_ASCII);
        GUI_SetColor(GUI_GREEN);
        if (rx < 1000.0f) {
            snprintf(line, sizeof(line), "Rx = %.1f \xE2\x84\xA6", rx);
        } else if (rx < 1000000.0f) {
            snprintf(line, sizeof(line), "Rx = %.2f k\xCE\xA9", rx/1000.0f);
        } else {
            snprintf(line, sizeof(line), "Rx = %.2f M\xCE\xA9", rx/1000000.0f);
        }
        GUI_DispStringAt(line, 40, 90);
    }
}

/* ===== Modus: Continuïteit / externe knop ===== */
static void mode_cont_step(void)
{
    if (!s_cont_pin_inited) {
        cy_rslt_t r = cyhal_gpio_init(CONTINUITY_PIN,
                                      CYHAL_GPIO_DIR_INPUT,
                                      CYHAL_GPIO_DRIVE_PULLUP, 1);
        CY_ASSERT(r == CY_RSLT_SUCCESS);
        s_cont_pin_inited = true;
    }

    header("Modus: Continuiteit (P10_6, LOW=contact)", NULL);
    GUI_SetColor(GUI_BLACK);
    GUI_FillRect(0, 30, DISP_W, DISP_H);

    bool contact = (cyhal_gpio_read(CONTINUITY_PIN) == 0); /* LOW = contact */

    GUI_SetFont(GUI_FONT_24_ASCII);
    if (contact) {
        GUI_SetColor(GUI_GREEN);
        GUI_DispStringAt("Contact!", 110, 100);
    } else {
        GUI_SetColor(GUI_RED);
        GUI_DispStringAt("Geen contact", 80, 100);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ===== I2C: probe helper ===== */
static bool i2c_probe_addr(uint8_t addr)
{
    cy_rslt_t r;
    /* 0-byte write: aanwezigheidstest */
    r = cyhal_i2c_master_write(&s_i2c, addr, NULL, 0, I2C_TIMEOUT_MS, true);
    if (r == CY_RSLT_SUCCESS) { return true; }

    /* 1-byte read fallback */
    uint8_t dummy = 0;
    r = cyhal_i2c_master_read(&s_i2c, addr, &dummy, 1, I2C_TIMEOUT_MS, true);
    return (r == CY_RSLT_SUCCESS);
}

/* ===== Modus: I2C Scanner (op TFT) ===== */
static void mode_i2c_step(void)
{
    /* Scan elke ~1000 ms */
    TickType_t now = xTaskGetTickCount();
    if (now >= s_i2c_next_scan) {
        s_i2c_next_scan = now + pdMS_TO_TICKS(1000);

        s_i2c_found_cnt = 0;
        memset(s_i2c_found, 0, sizeof(s_i2c_found));

        for (uint8_t a = I2C_ADDR_MIN; a <= I2C_ADDR_MAX; ++a) {
            bool ok = i2c_probe_addr(a);
            if (ok && (s_i2c_found_cnt < sizeof(s_i2c_found))) {
                s_i2c_found[s_i2c_found_cnt++] = a;
            }
            vTaskDelay(pdMS_TO_TICKS(2)); /* bus ademruimte */
        }
    }

    /* Tekenen */
    header("Modus: I2C Scanner (P6_0=SCL, P6_1=SDA)", "100 kHz");

    GUI_SetColor(GUI_BLACK);
    GUI_FillRect(0, 30, DISP_W, DISP_H);

    GUI_SetColor(GUI_WHITE);
    GUI_SetFont(GUI_FONT_16B_1);
    GUI_DispStringAt("Addr 0x08..0x77 (ACK=adres)", 8, 34);

    int y = 58;
    GUI_SetFont(GUI_FONT_13_1);
    for (uint8_t row = 0; row < 8; ++row) {
        char line[64] = {0};
        int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X: ", row << 4);
        for (uint8_t col = 0; col < 16; ++col) {
            uint8_t a = (row << 4) | col;
            if (a < I2C_ADDR_MIN || a > I2C_ADDR_MAX) {
                pos += snprintf(line + pos, sizeof(line) - pos, "   ");
                continue;
            }
            bool found = false;
            for (uint8_t i = 0; i < s_i2c_found_cnt; ++i) {
                if (s_i2c_found[i] == a) { found = true; break; }
            }
            if (found) {
                pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", a);
            } else {
                pos += snprintf(line + pos, sizeof(line) - pos, ".. ");
            }
        }
        GUI_DispStringAt(line, 8, y);
        y += 18;
        if (y > (DISP_H - 22)) { break; }
    }

    /* Samenvatting */
    GUI_SetFont(GUI_FONT_13_1);
    if (s_i2c_found_cnt == 0) {
        GUI_SetColor(GUI_RED);
        GUI_DispStringAt("Geen devices. Tip: BMP280=0x76/0x77. Check 4.7k pull-ups naar 3V3.", 8, DISP_H - 20);
    } else {
        GUI_SetColor(GUI_GREEN);
        char foundline[128] = {0};
        int p = 0;
        p += snprintf(foundline + p, sizeof(foundline) - p, "Gevonden (%u): ", (unsigned)s_i2c_found_cnt);
        for (uint8_t i = 0; i < s_i2c_found_cnt; ++i) {
            p += snprintf(foundline + p, sizeof(foundline) - p, "0x%02X%s", s_i2c_found[i], (i + 1 < s_i2c_found_cnt) ? " " : "");
        }
        GUI_DispStringAt(foundline, 8, DISP_H - 20);
    }
}

/* ===== Modus: Digital Pin Tester (P13_6) =====
   Toggle P13_6 als OUTPUT; lees hem kort terug als INPUT_PULLUP om status te tonen.
*/
static void mode_dpin_step(void)
{
    static bool init_done = false;
    static bool state = false;
    static TickType_t last_toggle = 0;

    if (!init_done) {
        /* Init P13_6 als output, start LOW */
        cy_rslt_t r = cyhal_gpio_init(DPIN_TEST_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
        CY_ASSERT(r == CY_RSLT_SUCCESS);
        init_done = true;
    }

    header("Modus: Digital Pin Tester (P13_6)", NULL);
    GUI_SetColor(GUI_BLACK);
    GUI_FillRect(0, 30, DISP_W, DISP_H);

    TickType_t now = xTaskGetTickCount();
    if ((now - last_toggle) > pdMS_TO_TICKS(500)) {
        state = !state;
        cyhal_gpio_write(DPIN_TEST_PIN, state);   // toggle pin
        last_toggle = now;
    }

    /* Toon outputstatus */
    GUI_SetFont(GUI_FONT_24_ASCII);
    if (state) {
        GUI_SetColor(GUI_GREEN);
        GUI_DispStringAt("OUTPUT: HIGH", 60, 80);
    } else {
        GUI_SetColor(GUI_RED);
        GUI_DispStringAt("OUTPUT: LOW", 60, 80);
    }

    /* Lees ook inputstatus (zelfde pin teruglezen) */
    cyhal_gpio_init(DPIN_TEST_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);
    bool input_val = cyhal_gpio_read(DPIN_TEST_PIN);

    GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetColor(GUI_WHITE);
    if (input_val) {
        GUI_DispStringAt("INPUT: HIGH (los)", 60, 130);
    } else {
        GUI_DispStringAt("INPUT: LOW  (naar GND)", 60, 130);
    }

    /* Zet terug als output zodat de toggle blijft werken */
    cyhal_gpio_init(DPIN_TEST_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, state);

    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ===== Modus: Digital Pin Monitor (P13_7) =====
   P13_7 als input (pull-up). Sluit extern signaal aan: HIGH≈3V3, LOW=GND.
*/
static void mode_dpin_monitor_step(void)
{
    static bool init_done = false;

    if (!init_done) {
        cy_rslt_t r = cyhal_gpio_init(DPIN_MONITOR_PIN,
                                      CYHAL_GPIO_DIR_INPUT,
                                      CYHAL_GPIO_DRIVE_PULLUP, 1);
        CY_ASSERT(r == CY_RSLT_SUCCESS);
        init_done = true;
    }

    header("Modus: Digital Pin Monitor", "Pin P13_7");

    GUI_SetColor(GUI_BLACK);
    GUI_FillRect(0, 30, DISP_W, DISP_H);

    bool val = cyhal_gpio_read(DPIN_MONITOR_PIN);

    GUI_SetFont(GUI_FONT_24_ASCII);
    if (val) {
        GUI_SetColor(GUI_GREEN);
        GUI_DispStringAt("P13_7 = HIGH", 80, 100);
    } else {
        GUI_SetColor(GUI_RED);
        GUI_DispStringAt("P13_7 = LOW", 80, 100);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ===== Board-knop: kort = mode, lang = T/div (osc) ===== */
static void handle_button(void)
{
    static bool prev = true;                 /* boardknop actief-laag: niet ingedrukt = 1 */
    static TickType_t t_down = 0;
    static bool long_fired = false;

    bool now = cyhal_gpio_read(BTN_PIN);
    TickType_t tnow = xTaskGetTickCount();

    /* press */
    if (!now && prev) {
        t_down = tnow;
        long_fired = false;
    }

    /* hold -> long press */
    if (!now && !prev) {
        if (!long_fired && (tnow - t_down) > pdMS_TO_TICKS(LONGPRESS_MS)) {
            g_tb_index = (uint8_t)((g_tb_index + 1) % TB_COUNT);
            apply_timebase(g_tb_index);
            GUI_Clear();
            long_fired = true;
        }
    }

    /* release -> korte druk = volgende modus (expliciete wrap) */
    if (now && !prev) {
        if (!long_fired && (tnow - t_down) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
            ui_mode_t next = (g_mode == (ui_mode_t)(MODE_COUNT - 1)) ? MODE_OSC
                                                                     : (ui_mode_t)(g_mode + 1);
            g_mode = next;
            GUI_Clear();
        }
    }

    prev = now;
}

/* ===== Task ===== */
void tft_task(void *arg)
{
    (void)arg;
    cy_rslt_t r;

    /* TFT */
    r = mtb_st7789v_init8(&tft_pins); CY_ASSERT(r == CY_RSLT_SUCCESS);
    GUI_Init();
    GUI_SetBkColor(GUI_BLACK); GUI_Clear(); GUI_SetColor(GUI_WHITE);

    /* ADC */
    const cyhal_adc_channel_config_t ch_cfg = { .enable_averaging = false, .min_acquisition_ns = 220, .enabled = true };
    r = cyhal_adc_init(&s_adc, ADC_INPUT_PIN, NULL); CY_ASSERT(r == CY_RSLT_SUCCESS);
    r = cyhal_adc_channel_init_diff(&s_adc_ch, &s_adc, ADC_INPUT_PIN, CYHAL_ADC_VNEG, &ch_cfg); CY_ASSERT(r == CY_RSLT_SUCCESS);

    /* Board-knop */
    r = cyhal_gpio_init(BTN_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1); CY_ASSERT(r == CY_RSLT_SUCCESS);

    /* I2C init */
    const cyhal_i2c_cfg_t i2c_cfg = {
        .is_slave = false,
        .address = 0,
        .frequencyhal_hz = I2C_FREQ_HZ
    };
    r = cyhal_i2c_init(&s_i2c, I2C_SDA, I2C_SCL, NULL);  CY_ASSERT(r == CY_RSLT_SUCCESS);
    r = cyhal_i2c_configure(&s_i2c, &i2c_cfg);           CY_ASSERT(r == CY_RSLT_SUCCESS);

    /* Sample timer + T/div */
    init_sample_timer(SAMPLE_START_HZ);
    apply_timebase(g_tb_index);

    /* I2C scan direct plannen */
    s_i2c_next_scan = xTaskGetTickCount();

    /* Main loop */
    while (1) {
        handle_button();

        if (g_mode >= MODE_COUNT) { g_mode = MODE_OSC; } /* safeguard */

        switch (g_mode) {
            case MODE_OSC:       mode_osc_step();       vTaskDelay(pdMS_TO_TICKS(1));   break;
            case MODE_OSC_PWM:   mode_osc_pwm_step();   vTaskDelay(pdMS_TO_TICKS(1));   break; /* tweede modus */
            case MODE_VOLT:      mode_volt_step();      vTaskDelay(pdMS_TO_TICKS(1));   break;
            case MODE_RES:       mode_res_step();       vTaskDelay(pdMS_TO_TICKS(150)); break;
            case MODE_CONT:      mode_cont_step();                                      break;
            case MODE_I2C:       mode_i2c_step();       vTaskDelay(pdMS_TO_TICKS(20));  break;
            case MODE_DPIN:      mode_dpin_step();                                      break;
            case MODE_DPIN_MON:  mode_dpin_monitor_step();                              break;
            default:             vTaskDelay(pdMS_TO_TICKS(10));                         break;
        }
        taskYIELD();
    }
}
