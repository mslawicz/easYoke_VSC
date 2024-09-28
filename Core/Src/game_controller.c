#include "game_controller.h"
#include "main.h"
#include "custom_app.h"

uint8_t joyReport[HID_REPORT_SIZE];

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

    /* XXX test of joystick report */
    int16_t axisVal = -32767 + 655 * (loopCounter % 100);
    *(int16_t*)joyReport = axisVal;
    *(int16_t*)(joyReport + 2) = axisVal;
    *(int16_t*)(joyReport + 4) = axisVal;
    *(int16_t*)(joyReport + 6) = 32767 - axisVal;
    joyReport[8] = 1 + (loopCounter >> 4) % 8;  //HAT
    joyReport[9] = 1 << ((loopCounter >> 5) % 8);   //buttons
    if(loopCounter % 90 == 89)
    {
        updateHidReport(joyReport);
    }

    loopCounter++;
}

void gameControllerInit(void)
{
    UTIL_SEQ_RegTask(1 << CFG_TASK_GAME_CONTROLLER_ID, UTIL_SEQ_RFU, gameController);
}