/*
 * I2C_slave.h
 *
 *  Created on: Jul 11, 2024
 *      Author: polie
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#ifndef WHEEL_NUM
#define WHEEL_NUM 4
#endif

typedef union
{
    signed char value;
    uint8_t byte;
} I2C_Signal_t;

typedef union
{
    float value[WHEEL_NUM];
    uint8_t bytes[sizeof(float)*WHEEL_NUM];
} I2C_Wheel_speed_t;

void process_data(void);

#endif /* INC_I2C_SLAVE_H_ */
