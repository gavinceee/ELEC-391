#include "sr595.h"

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
                          (data >> i) & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(hsr->clk_port, hsr->clk_pin, GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(hsr->latch_port, hsr->latch_pin, GPIO_PIN_SET);

    hsr->state = data;
}

void SR595_SetBit(SR595_HandleTypeDef *hsr, uint8_t bit, bool value)
{
    if (bit > 7) return;

    if (value)
        hsr->state |= (1 << bit);
    else
        hsr->state &= ~(1 << bit);

    SR595_Write(hsr, hsr->state);
}

void SR595_Clear(SR595_HandleTypeDef *hsr)
{
    SR595_Write(hsr, 0x00);
}
