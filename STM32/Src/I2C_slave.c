/*
 * I2C_slave.c
 *
 *  Created on: Jul 11, 2024
 *      Author: polie
 */


#include "main.h"
#include "I2C_slave.h"
#include <stdint.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_Signal_t pi_signal;
extern I2C_Wheel_speed_t  wheel_speed_ref, wheel_speed_est;
uint8_t rxbuffer[sizeof(float)*WHEEL_NUM+1];
uint8_t rxcount = 0;
uint8_t txcount = 0;
uint8_t is_first = 0;

void process_data(void)
{
    if (rxbuffer[0]==1) memcpy(&pi_signal.byte, rxbuffer+1, rxbuffer[0]);
    else if (rxbuffer[0]==sizeof(float)*WHEEL_NUM) memcpy( wheel_speed_ref.bytes, rxbuffer+1, rxbuffer[0]);

}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
    {
        if (is_first == 0)
        {
            rxcount = 0;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxbuffer+rxcount, 1, I2C_FIRST_FRAME);
        }
    }
    else
    {
        txcount = 0;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c,  wheel_speed_est.bytes+txcount, 1, I2C_FIRST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (is_first == 0)
    {
        rxcount++;
        is_first = 1;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxbuffer+rxcount, rxbuffer[0], I2C_LAST_FRAME);
    }
    else
    {
        rxcount = rxcount+rxbuffer[0];
        is_first = 0;
        process_data();
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    txcount++;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c,  wheel_speed_est.bytes+txcount, 1, I2C_NEXT_FRAME);
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t errcode = HAL_I2C_GetError(hi2c);
    if (errcode == 4)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
        if (txcount==0) process_data();
        else txcount--;
    }
    HAL_I2C_EnableListen_IT(hi2c);
}
