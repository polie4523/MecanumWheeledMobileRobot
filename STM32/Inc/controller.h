/*
 * controller.h
 *
 *  Created on: Jul 11, 2024
 *      Author: polie
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#define MAX_CCR 255
#define WINDOW_SIZE 5
#define DEADZONE 30

#ifndef WHEEL_NUM
#define WHEEL_NUM 4
#endif

#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.05f
#endif

typedef struct {
	float A;
	float b;
} DSS_MODEL;

typedef struct {
	DSS_MODEL model[4];
    float c;
    float g;
    float q;
    float eta;
    float sat_width;
} DSMC_GAIN;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_GAIN;

float sat(float, const float, const float);
void RLS_step(DSMC_GAIN *, const float *, volatile const float *);
void DSMC_step(float *, volatile const float *, volatile const float *, const DSMC_GAIN);
void setDSMCgain(DSMC_GAIN *, const float, const float, const float, const float, const float, const float, const float);
void PID_step(float *, volatile const float *, volatile const float *, const PID_GAIN);
void MA_filter(float *, volatile const float *);

#endif /* INC_CONTROLLER_H_ */
