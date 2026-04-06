#ifndef INC_SR595_H_
#define INC_SR595_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    GPIO_TypeDef *data_port;
    uint16_t      data_pin;
    GPIO_TypeDef *clk_port;
    uint16_t      clk_pin;
    GPIO_TypeDef *latch_port;
    uint16_t      latch_pin;
    uint8_t       state;
} SR595_HandleTypeDef;

void SR595_Init(SR595_HandleTypeDef *hsr);
void SR595_Write(SR595_HandleTypeDef *hsr, uint8_t data);
void SR595_SetBit(SR595_HandleTypeDef *hsr, uint8_t bit, bool value);
void SR595_Clear(SR595_HandleTypeDef *hsr);

#endif
