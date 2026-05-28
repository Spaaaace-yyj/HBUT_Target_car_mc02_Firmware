//
// Created by spaaaaace on 2026/5/28.
//

#include "json_protocol.h"

#include <stdio.h>
#include <string.h>

static void uart_send_string(
    UART_HandleTypeDef *huart,
    const char *str
)
{
    HAL_UART_Transmit(
        huart,
        (uint8_t *)str,
        strlen(str),
        1000
    );
}

void json_send_status(
    UART_HandleTypeDef *huart,
    float battery,
    const char *fw_version,
    int move_mode
)
{
    char buf[256];

    int len = snprintf(
        buf,
        sizeof(buf),
        "{\"type\":\"status\","
        "\"battery\":%.2f,"
        "\"fw_version\":\"%s\","
        "\"move_mode\":%d}\n",
        battery,
        fw_version,
        move_mode
    );

    if (len > 0)
    {
        uart_send_string(huart, buf);
    }
}

void json_send_log(
    UART_HandleTypeDef *huart,
    const char *msg
)
{
    char buf[256];

    int len = snprintf(
        buf,
        sizeof(buf),
        "{\"type\":\"log\","
        "\"msg\":\"%s\"}\n",
        msg
    );

    if (len > 0)
    {
        uart_send_string(huart, buf);
    }
}

void json_send_ota_state(
    UART_HandleTypeDef *huart,
    const char *state
)
{
    char buf[128];

    int len = snprintf(
        buf,
        sizeof(buf),
        "{\"type\":\"ota\","
        "\"state\":\"%s\"}\n",
        state
    );

    if (len > 0)
    {
        uart_send_string(huart, buf);
    }
}