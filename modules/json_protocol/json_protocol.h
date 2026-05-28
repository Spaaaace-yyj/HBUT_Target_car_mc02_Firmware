//
// Created by spaaaaace on 2026/5/28.
//

#ifndef JSON_PROTOCOL_H
#define JSON_PROTOCOL_H

#include "main.h"

void json_send_status(
    UART_HandleTypeDef *huart,
    float battery,
    const char *fw_version,
    int move_mode
);

void json_send_log(
    UART_HandleTypeDef *huart,
    const char *msg
);

void json_send_ota_state(
    UART_HandleTypeDef *huart,
    const char *state
);

#endif