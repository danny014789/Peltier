/**
  ******************************************************************************
  * @file    cli.c
  * @brief   ASCII CLI over USB CDC. RX is fed from CDC_Receive_FS into a
  *          ring buffer; CLI_Process drains complete lines and dispatches.
  *          TX uses CDC_Transmit_FS with a small busy-wait when the host
  *          hasn't drained the previous packet.
  *
  *          Numeric formatting is hand-rolled because the default
  *          newlib-nano printf in STM32CubeIDE projects strips %f support.
  ******************************************************************************
  */
#include "cli.h"
#include "main.h"
#include "control.h"
#include "peltier.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#define CLI_RX_RING_SZ      256
#define CLI_LINE_MAX        96
#define CLI_TX_BUF_SZ       160
#define CLI_TELEMETRY_HZ    10

/* RX ring buffer (single-producer ISR / single-consumer main) */
static volatile uint8_t  s_rx_ring[CLI_RX_RING_SZ];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static char    s_line[CLI_LINE_MAX];
static uint8_t s_line_len;

static uint8_t  s_stream_on = 1;
static uint32_t s_last_stream_tick;

/* ---------------------- USB TX (busy-tolerant) ---------------------------- */

static uint8_t cli_tx_blocking(const uint8_t *buf, uint16_t len)
{
    uint32_t deadline = HAL_GetTick() + 20;
    uint8_t  rc;
    do {
        rc = CDC_Transmit_FS((uint8_t *)buf, len);
        if (rc != USBD_BUSY) return rc;
    } while ((int32_t)(deadline - HAL_GetTick()) > 0);
    return USBD_BUSY;
}

void CLI_PrintLine(const char *s)
{
    cli_tx_blocking((const uint8_t *)s, (uint16_t)strlen(s));
    cli_tx_blocking((const uint8_t *)"\r\n", 2);
}

/* --------------------------- Tiny formatter ------------------------------- */
/* Append helpers; all return new length-used in buf, never overflow.        */

static int append_str(char *buf, int pos, int cap, const char *s)
{
    while (*s && pos < cap - 1) buf[pos++] = *s++;
    return pos;
}

static int append_int(char *buf, int pos, int cap, int32_t v)
{
    char tmp[12];
    int  n = 0;
    uint8_t neg = 0;
    if (v < 0) { neg = 1; v = -v; }
    if (v == 0) tmp[n++] = '0';
    while (v > 0 && n < (int)sizeof(tmp)) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    if (neg && pos < cap - 1) buf[pos++] = '-';
    while (n-- > 0 && pos < cap - 1) buf[pos++] = tmp[n];
    return pos;
}

static int append_uint(char *buf, int pos, int cap, uint32_t v)
{
    char tmp[12];
    int  n = 0;
    if (v == 0) tmp[n++] = '0';
    while (v > 0 && n < (int)sizeof(tmp)) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n-- > 0 && pos < cap - 1) buf[pos++] = tmp[n];
    return pos;
}

/* Two-decimal float, e.g. -3.14 or 25.00; clamps |v| < 1e6. */
static int append_f2(char *buf, int pos, int cap, float v)
{
    if (isnan(v) || isinf(v)) return append_str(buf, pos, cap, "nan");
    if (v < 0) {
        if (pos < cap - 1) buf[pos++] = '-';
        v = -v;
    }
    if (v > 999999.99f) v = 999999.99f;
    int32_t whole = (int32_t)v;
    int32_t frac  = (int32_t)((v - (float)whole) * 100.0f + 0.5f);
    if (frac >= 100) { whole += 1; frac -= 100; }
    pos = append_uint(buf, pos, cap, (uint32_t)whole);
    if (pos < cap - 1) buf[pos++] = '.';
    if (pos < cap - 1) buf[pos++] = (char)('0' + (frac / 10));
    if (pos < cap - 1) buf[pos++] = (char)('0' + (frac % 10));
    return pos;
}

/* Three-decimal float for gain printout. */
static int append_f3(char *buf, int pos, int cap, float v)
{
    if (isnan(v) || isinf(v)) return append_str(buf, pos, cap, "nan");
    if (v < 0) {
        if (pos < cap - 1) buf[pos++] = '-';
        v = -v;
    }
    if (v > 999999.999f) v = 999999.999f;
    int32_t whole = (int32_t)v;
    int32_t frac  = (int32_t)((v - (float)whole) * 1000.0f + 0.5f);
    if (frac >= 1000) { whole += 1; frac -= 1000; }
    pos = append_uint(buf, pos, cap, (uint32_t)whole);
    if (pos < cap - 1) buf[pos++] = '.';
    if (pos < cap - 1) buf[pos++] = (char)('0' + (frac / 100)); frac %= 100;
    if (pos < cap - 1) buf[pos++] = (char)('0' + (frac / 10));
    if (pos < cap - 1) buf[pos++] = (char)('0' + (frac % 10));
    return pos;
}

static void cli_send(const char *buf, int n)
{
    if (n > 0) cli_tx_blocking((const uint8_t *)buf, (uint16_t)n);
}

/* ------------------------------ RX feed ----------------------------------- */

void CLI_FeedRx(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        uint16_t next = (uint16_t)((s_rx_head + 1) % CLI_RX_RING_SZ);
        if (next == s_rx_tail) break;
        s_rx_ring[s_rx_head] = buf[i];
        s_rx_head = next;
    }
}

static int rx_pop(uint8_t *out)
{
    if (s_rx_tail == s_rx_head) return 0;
    *out = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1) % CLI_RX_RING_SZ);
    return 1;
}

/* --------------------------- Command handling ----------------------------- */

static void to_upper(char *p)
{
    for (; *p; ++p) *p = (char)toupper((unsigned char)*p);
}

static int parse_float_arg(const char *line, float *out)
{
    while (*line && !isspace((unsigned char)*line)) ++line;
    while (*line && isspace((unsigned char)*line)) ++line;
    if (*line == '\0') return 0;
    char *end;
    float v = strtof(line, &end);
    if (end == line) return 0;
    *out = v;
    return 1;
}

static void emit_status(void)
{
    const ControlState_t *s = Control_GetState();
    PID_t *p = Control_GetPID();
    char buf[CLI_TX_BUF_SZ];
    int  n = 0, cap = sizeof(buf);
    n = append_str(buf, n, cap, "STATUS sp=");
    n = append_f2 (buf, n, cap, s->setpoint_c);
    n = append_str(buf, n, cap, " t=");
    n = append_f2 (buf, n, cap, s->temp_c);
    n = append_str(buf, n, cap, " duty=");
    n = append_int(buf, n, cap, s->duty);
    n = append_str(buf, n, cap, " en=");
    n = append_uint(buf, n, cap, s->enabled);
    n = append_str(buf, n, cap, " flt=");
    n = append_uint(buf, n, cap, s->fault);
    n = append_str(buf, n, cap, " sens=");
    n = append_uint(buf, n, cap, s->sensor_ok);
    n = append_str(buf, n, cap, " Kp=");  n = append_f3(buf, n, cap, p->kp);
    n = append_str(buf, n, cap, " Ki=");  n = append_f3(buf, n, cap, p->ki);
    n = append_str(buf, n, cap, " Kd=");  n = append_f3(buf, n, cap, p->kd);
    n = append_str(buf, n, cap, "\r\n");
    cli_send(buf, n);
}

static void emit_get(void)
{
    const ControlState_t *s = Control_GetState();
    char buf[32];
    int  n = 0;
    n = append_str(buf, n, sizeof(buf), "T=");
    n = append_f2 (buf, n, sizeof(buf), s->temp_c);
    n = append_str(buf, n, sizeof(buf), "\r\n");
    cli_send(buf, n);
}

static void emit_gains_ok(void)
{
    PID_t *p = Control_GetPID();
    char buf[CLI_TX_BUF_SZ];
    int  n = 0, cap = sizeof(buf);
    n = append_str(buf, n, cap, "OK Kp=");  n = append_f3(buf, n, cap, p->kp);
    n = append_str(buf, n, cap, " Ki=");    n = append_f3(buf, n, cap, p->ki);
    n = append_str(buf, n, cap, " Kd=");    n = append_f3(buf, n, cap, p->kd);
    n = append_str(buf, n, cap, "\r\n");
    cli_send(buf, n);
}

static void emit_setpoint_ok(void)
{
    const ControlState_t *s = Control_GetState();
    char buf[32];
    int  n = 0;
    n = append_str(buf, n, sizeof(buf), "OK sp=");
    n = append_f2 (buf, n, sizeof(buf), s->setpoint_c);
    n = append_str(buf, n, sizeof(buf), "\r\n");
    cli_send(buf, n);
}

static void emit_help(void)
{
    static const char *help =
        "Commands:\r\n"
        "  SET <degC>     set target temperature (0..25)\r\n"
        "  GET            current temperature\r\n"
        "  KP|KI|KD <v>   tune PID gain\r\n"
        "  START / STOP   enable / disable output\r\n"
        "  STATUS         full status\r\n"
        "  STREAM ON|OFF  10 Hz telemetry\r\n"
        "  CLR            clear latched fault\r\n"
        "  HELP";
    CLI_PrintLine(help);
}

static void dispatch(char *line)
{
    if (s_line_len == 0) return;
    char cmd[16] = {0};
    size_t i = 0;
    while (line[i] && !isspace((unsigned char)line[i]) && i < sizeof(cmd) - 1) {
        cmd[i] = line[i];
        ++i;
    }
    to_upper(cmd);

    if (strcmp(cmd, "SET") == 0) {
        float v;
        if (parse_float_arg(line, &v)) {
            Control_SetSetpoint(v);
            emit_setpoint_ok();
        } else {
            CLI_PrintLine("ERR usage: SET <degC>");
        }
    } else if (strcmp(cmd, "GET") == 0) {
        emit_get();
    } else if (strcmp(cmd, "KP") == 0 || strcmp(cmd, "KI") == 0 || strcmp(cmd, "KD") == 0) {
        float v;
        if (parse_float_arg(line, &v)) {
            PID_t *p = Control_GetPID();
            if      (cmd[1] == 'P') p->kp = v;
            else if (cmd[1] == 'I') p->ki = v;
            else                     p->kd = v;
            emit_gains_ok();
        } else {
            CLI_PrintLine("ERR usage: KP|KI|KD <value>");
        }
    } else if (strcmp(cmd, "START") == 0) {
        Control_SetEnabled(1);
        const ControlState_t *s = Control_GetState();
        char buf[48];
        int n = 0, cap = sizeof(buf);
        n = append_str(buf, n, cap, "OK enabled=");
        n = append_uint(buf, n, cap, s->enabled);
        n = append_str(buf, n, cap, " fault=");
        n = append_uint(buf, n, cap, s->fault);
        n = append_str(buf, n, cap, "\r\n");
        cli_send(buf, n);
    } else if (strcmp(cmd, "STOP") == 0) {
        Control_SetEnabled(0);
        CLI_PrintLine("OK stopped");
    } else if (strcmp(cmd, "CLR") == 0) {
        Control_ClearFault();
        CLI_PrintLine("OK fault cleared");
    } else if (strcmp(cmd, "STREAM") == 0) {
        char *arg = line + i;
        while (*arg && isspace((unsigned char)*arg)) ++arg;
        to_upper(arg);
        if      (strncmp(arg, "ON",  2) == 0) { s_stream_on = 1; CLI_PrintLine("OK stream on"); }
        else if (strncmp(arg, "OFF", 3) == 0) { s_stream_on = 0; CLI_PrintLine("OK stream off"); }
        else CLI_PrintLine("ERR usage: STREAM ON|OFF");
    } else if (strcmp(cmd, "STATUS") == 0) {
        emit_status();
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        emit_help();
    } else {
        CLI_PrintLine("ERR unknown cmd; try HELP");
    }
}

/* ------------------------------- Public ----------------------------------- */

void CLI_Init(void)
{
    s_rx_head = s_rx_tail = 0;
    s_line_len = 0;
    s_stream_on = 1;
    s_last_stream_tick = HAL_GetTick();
    CLI_PrintLine("Peltier PID controller ready. HELP for commands.");
}

void CLI_Process(void)
{
    uint8_t b;
    while (rx_pop(&b)) {
        if (b == '\r' || b == '\n') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                dispatch(s_line);
                s_line_len = 0;
            }
        } else if (b == 0x08 || b == 0x7F) {     /* BS / DEL */
            if (s_line_len > 0) s_line_len--;
        } else if (s_line_len + 1 < sizeof(s_line)) {
            s_line[s_line_len++] = (char)b;
        } else {
            s_line_len = 0;
            CLI_PrintLine("ERR line too long");
        }
    }
}

void CLI_StreamTelemetry(void)
{
    if (!s_stream_on) return;
    uint32_t now = HAL_GetTick();
    if ((now - s_last_stream_tick) < (1000u / CLI_TELEMETRY_HZ)) return;
    s_last_stream_tick = now;

    const ControlState_t *s = Control_GetState();
    char buf[CLI_TX_BUF_SZ];
    int  n = 0, cap = sizeof(buf);
    n = append_str(buf, n, cap, "T=");   n = append_f2 (buf, n, cap, s->temp_c);
    n = append_str(buf, n, cap, " SP="); n = append_f2 (buf, n, cap, s->setpoint_c);
    n = append_str(buf, n, cap, " U=");  n = append_int(buf, n, cap, s->duty);
    n = append_str(buf, n, cap, " EN="); n = append_uint(buf, n, cap, s->enabled);
    n = append_str(buf, n, cap, " FLT=");n = append_uint(buf, n, cap, s->fault);
    n = append_str(buf, n, cap, "\r\n");
    cli_send(buf, n);
}
