/**
  ******************************************************************************
  * @file    cli.h
  * @brief   USB CDC line-based ASCII command interface.
  *          Commands (terminated by '\r' or '\n'):
  *              SET <degC>   set target temperature (clamped 0..25)
  *              GET          report current temperature
  *              KP <v> | KI <v> | KD <v>   tune PID gains
  *              START        enable closed-loop output
  *              STOP         disable output (Peltier shut down)
  *              STATUS       full status dump
  *              STREAM ON|OFF  toggle 10 Hz telemetry stream
  *              CLR          clear latched fault
  *              HELP
  ******************************************************************************
  */
#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void CLI_Init(void);
void CLI_FeedRx(const uint8_t *buf, uint32_t len);   /* from CDC RX cb */
void CLI_Process(void);                               /* main-loop pump  */
void CLI_StreamTelemetry(void);                       /* 10 Hz telemetry */
void CLI_PrintLine(const char *s);

#endif /* CLI_H */
