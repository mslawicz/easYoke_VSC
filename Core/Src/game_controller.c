#include "game_controller.h"
#include "main.h"

void gameController(void)
{
    static const uint16_t BitPattern = 0x0005;
    static const uint8_t NumbOfBits = 10;
    static uint8_t bitNumber = 0;
    static uint32_t loopCounter = 0;
    static const uint8_t HeartBeatPeriod = 1000 / GAME_CTRL_PERIOD / NumbOfBits;

    if(loopCounter % HeartBeatPeriod == 0)
    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, ((BitPattern >> bitNumber) & 1) != 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        bitNumber = (bitNumber + 1) % NumbOfBits;
    }

    loopCounter++;
}

void gameControllerInit(void)
{
    UTIL_SEQ_RegTask(1 << CFG_TASK_GAME_CONTROLLER_ID, UTIL_SEQ_RFU, gameController);
}