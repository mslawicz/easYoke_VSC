#ifndef GAME_CONTROLLER_H
#define GAME_CONTROLLER_H

#include "stm32_seq.h"

#define GAME_CTRL_PERIOD 10
#define GAME_CTRL_PRIORITY 1

void gameController(void);
void gameControllerInit(void);

#endif /* GAME_CONTROLLER_H */