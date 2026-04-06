#include "sr595.h"

static uint8_t SR595_MapBit(uint8_t bit)
{
    /* Logical -> physical mapping for your 5 outputs:
       0 -> 0
       1 -> 4
       2 -> 3
       3 -> 2
       4 -> 1
    */
    static const uint8_t map[5] = {0, 4, 3, 2, 1};

    if (bit < 5)
        return map[bit];

    return bit;   // leave bits 5,6,7 unchanged
}

void SR595_Init(SR595_HandleTypeDef *hsr)
{
    hsr->state = 0x00;
    SR595_Write(hsr, 0x00);
}

void SR595_Write(SR595_HandleTypeDef *hsr, uint8_t data)
{
    HAL_GPIO_WritePin(hsr->latch_port, hsr->latch_pin, GPIO_PIN_RESET);

    for (int8_t i = 7; i >= 0; i--)
    {
        HAL_GPIO_WritePin(hsr->clk_port, hsr->clk_pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(hsr->data_port, hsr->data_pin,
                          ((data >> i) & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(hsr->clk_port, hsr->clk_pin, GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(hsr->latch_port, hsr->latch_pin, GPIO_PIN_SET);

    hsr->state = data;
}

void SR595_SetBit(SR595_HandleTypeDef *hsr, uint8_t bit, bool value)
{
    uint8_t physBit;

    if (bit > 7) return;

    physBit = SR595_MapBit(bit);

    if (value)
        hsr->state |= (1U << physBit);
    else
        hsr->state &= ~(1U << physBit);

    SR595_Write(hsr, hsr->state);
}

void SR595_Clear(SR595_HandleTypeDef *hsr)
{
    SR595_Write(hsr, 0x00);
}
