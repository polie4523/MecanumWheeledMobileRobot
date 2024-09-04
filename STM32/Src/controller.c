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

void RLS_step(DSMC_GAIN *param, const float *u, volatile const float *y) {
	static float old_y[WHEEL_NUM] = {0.0f}, old_param[WHEEL_NUM][2] = {{0.0f}};
	static float old_P[WHEEL_NUM][3] = {{1.,1.,0.},{1.,1.,0.},{1.,1.,0.},{1.,1.,0.}};
	float den = 0.f, y_hat = 0.f;
	float Q[3] = {0.f}, K[2] = {0.f};
	for (short i = 0; i < WHEEL_NUM; i++) {
		old_param[i][0] = param->model[i].A;
		old_param[i][1] = param->model[i].b;
		den = 1 + old_y[i]*(old_P[i][0]*old_y[i]+old_P[i][2]*u[i]) + u[i]*(old_P[i][2]*old_y[i]+old_P[i][1]*u[i]);
		Q[0] = old_P[i][0] / den;
		Q[1] = old_P[i][1] / den;
		Q[2] = old_P[i][2] / den;
		K[0] = Q[0]*old_y[i]+Q[2]*u[i];
		K[1] = Q[2]*old_y[i]+Q[1]*u[i];
		y_hat = old_y[i]*old_param[i][0] + u[i]*old_param[i][1];
		param->model[i].A = old_param[i][0] + K[0]*(y[i]-y_hat);
		param->model[i].b = old_param[i][1] + K[1]*(y[i]-y_hat);
		old_P[i][0] = old_P[i][0] - pow((old_P[i][0]*old_y[i]+old_P[i][2]*u[i]),2)/den;
		old_P[i][1] = old_P[i][1] - pow((old_P[i][2]*old_y[i]+old_P[i][1]*u[i]),2)/den;
		old_P[i][2] = old_P[i][2] - (old_P[i][0]*old_y[i]+old_P[i][2]*u[i])*(old_P[i][2]*old_y[i]+old_P[i][1]*u[i])/den;
		old_y[i] = y[i];
	}
}

void DSMC_step(float *const u, volatile const float *const x_d, volatile const float *const x, const DSMC_GAIN gain) {
    static float old_x_d[WHEEL_NUM] = {0.0f}, old_d_est[WHEEL_NUM] = {0.0f}, old_s[WHEEL_NUM]= {0.0f};
    float s=0.0f, d_est=0.0f,x_d_next=0.0f;
    for (short i = 0; i < WHEEL_NUM; i++) {
        s = gain.c*(x[i]-x_d[i]);
        d_est = old_d_est[i]+gain.g*(s-gain.q*old_s[i]+gain.eta*sat(old_s[i],gain.sat_width,1))/(gain.c*gain.model[i].b);
        x_d_next = 2*x_d[i] - old_x_d[i];
        u[i] = -d_est+(gain.c*x_d_next-gain.c*gain.model[i].A*x[i]+gain.q*s-gain.eta*sat(s,gain.sat_width,1))/(gain.c*gain.model[i].b);
        u[i] = sat(u[i],MAX_CCR,MAX_CCR);
        old_d_est[i] = d_est;
        old_s[i] = s;
        old_x_d[i] = x_d[i];
    }
}

void setDSMCgain(DSMC_GAIN *gain, const float A, const float b, const float c, const float g, const float q, const float eta, const float sat_width) {
	for (short i = 0; i < WHEEL_NUM; i++) {
		gain->model[i].A = A;
		gain->model[i].b = b;
	}
	gain->c = c;
	gain->g = g;
	gain->q = q;
	gain->eta = eta;
	gain->sat_width = sat_width;
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
