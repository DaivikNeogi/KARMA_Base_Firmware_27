#include "bno_port.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

#define BNO085_I2C_ADDRESS    (0x4A << 1)

static BNO_PortStatus STM32_BNO_Write(const uint8_t *data,
                                      size_t length)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(
        &hi2c3,
        BNO085_I2C_ADDRESS,
        (uint8_t *)data,
        length,
        100
    );

    return (status == HAL_OK)
           ? BNO_PORT_OK
           : BNO_PORT_ERROR;
}


static BNO_PortStatus STM32_BNO_Read(uint8_t *data,
                                     size_t length)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Receive(
        &hi2c3,
        BNO085_I2C_ADDRESS,
        data,
        length,
        100
    );

    return (status == HAL_OK)
           ? BNO_PORT_OK
           : BNO_PORT_ERROR;
}


static void STM32_BNO_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}


static void STM32_BNO_Reset(void)
{
    /*
     * Your actual BNO085 reset GPIO code goes here.
     */
}


void BNO_Port_Init(BNO_Port *port)
{
    port->write    = STM32_BNO_Write;
    port->read     = STM32_BNO_Read;
    port->delay_ms = STM32_BNO_Delay;
    port->reset    = STM32_BNO_Reset;
}