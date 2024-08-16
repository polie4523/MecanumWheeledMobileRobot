/*
 * controller.c
 *
 *  Created on: Jul 11, 2024
 *      Author: polie
 */

#include "controller.h"
#include <math.h>

float sat(float s, const float width, const float height) {
    float s_out=0.0f;
    if ( s > width) s_out = height;
    else if ( s < -width) s_out = -height;
    else s_out = s*(height/width);
    return s_out;
}

void DSMC_step(float *const u, volatile const float *const x_d, volatile const float *const x, const DSMC_GAIN gain) {
    static float old_x_d[WHEEL_NUM] = {0.0f}, old_d_est[WHEEL_NUM] = {0.0f}, old_s[WHEEL_NUM];
    float s=0.0f, d_est=0.0f,x_d_next=0.0f;
    for (short i = 0; i < WHEEL_NUM; i++) {
        s = gain.c*(x[i]-x_d[i]);
        d_est = old_d_est[i]+gain.g*(s-gain.q*old_s[i]+gain.eta*sat(old_s[i],gain.sat_width,1))/(gain.c*gain.b);
        x_d_next = 2*x_d[i] - old_x_d[i];
        u[i] = -d_est+(gain.c*x_d_next-gain.c*gain.A*x[i]+gain.q*s-gain.eta*sat(s,gain.sat_width,1))/(gain.c*gain.b);
        u[i] = sat(u[i],MAX_CCR,MAX_CCR);
        old_d_est[i] = d_est;
        old_s[i] = s;
        old_x_d[i] = x_d[i];
    }
}

void PID_step(float *const u, volatile const float *const x_d, volatile const float *const x, const PID_GAIN gain) {
    static float old_error[WHEEL_NUM] = {0.0};
    static float sum_error[WHEEL_NUM] = {0.0};
    float error;
    for (short i = 0; i < WHEEL_NUM; i++) {
        error = x_d[i] - x[i];
        u[i] = gain.Kp*error + gain.Ki*sum_error[i] + gain.Kd*((error-old_error[i]) / SAMPLE_TIME);
        u[i] = sat(u[i], MAX_CCR,MAX_CCR);
        old_error[i] = error;
        sum_error[i] += error*SAMPLE_TIME;
    }
}

void MA_filter(float *const output, volatile const float *const newdata) {
    static float data[WHEEL_NUM][WINDOW_SIZE]={{0}};
    static short now=0;
    float sum=0.0f;
    for (short i = 0; i < WHEEL_NUM; i++) {
        sum=0.0f;
        data[i][now] = newdata[i];
        for (short j = 0; j < WINDOW_SIZE; j++) sum += data[i][j];
        output[i] = sum / WINDOW_SIZE;
    }
    now = (now+1)%WINDOW_SIZE;
}
