#include "modbus.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

extern uint8_t str;
uint8_t dataReceived=0; // признак данное получено
uint8_t dataTransmitted=1; // признак данное передано
uint8_t cnt = 0;


uint8_t GenCRC16(uint8_t* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;

	buff[len++] = lo;
	buff[len++] = hi;
	return len;
}

uint8_t CheckCRC16(uint8_t* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len - 2; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;
	if ((buff[len - 2] == lo) &&
		(buff[len - 1] == hi))
	{
		return 1;
	}
	return 0;
}

void RxModBus(uint8_t *buff_uart)
{
    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SetCounter(&htim6, 0);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_UART_Receive_IT (&huart1, &str, 1);
    buff_uart[cnt] = str;
    cnt = cnt + 1;
}


void EndPacketModBus(uint8_t *buff_uart)
{
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    if (CheckCRC16(buff_uart, cnt)) {
        cnt = GenCRC16(buff_uart, cnt);
        HAL_UART_Transmit_IT(&huart1, buff_uart, cnt);
    } else {
        HAL_UART_Transmit_IT(&huart1, buff_uart, 2);
    }
    HAL_UART_Receive_IT(&huart1, &str, 1);
    cnt = 0;
}
