#ifndef MODBUS_H_
#define MODBUS_H_
//------------------------------------------------
#include "stm32l0xx_hal.h"
#include <math.h>
//------------------------------------------------

void RxModBus(uint8_t *buff_uart);
void EndPacketModBus(uint8_t *buff_uart);

#endif /* MODBUS_H_ */