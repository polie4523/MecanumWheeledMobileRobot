#ifndef __MWMR_H
#define __MWMR_H

#include <csignal>
#include <sys/time.h>
#include <array>
#include "i2c_master.h"
#include "MPU9250.h"
#include "TrajectoryGenerator.h"

#define WHEEL_NUM 4
#define SAMPLE_TIME 0.05f
#define WHEEL_R 0.03f
#define L 0.358f // track width+base
#define MPU9250_ADDRESS 0x68

typedef union {
    float value[WHEEL_NUM] {0.0};
    char bytes[sizeof(float)*WHEEL_NUM];
} Wheel_speed_t;

typedef struct {
    float kp[3] {1,1,1};
    float ki[3] {0.25,0.25,0.25};
    float kb[3] {0.25,0.25,0.25};
    float max_vel[3] {0.6,0.6,0.8};
} ControllerParameter;


class MecanumWMR
{
    public:
        MecanumWMR(): isDone(false), angle(0.0f), p_est({0.0}), starting_point({0.0})
        {
            ControllerParam = ControllerParameter();
            wheel_speed_ref = Wheel_speed_t();
            wheel_speed_est = Wheel_speed_t();
            instance = this;
            struct sigaction sa;
            sa.sa_flags = SA_RESTART;
            sa.sa_handler = &MecanumWMR::TimerHandler;
            sigaction(SIGALRM, &sa, nullptr);
        }
        
        bool init(void);
        void shutdown(void);
        void setInitCond(const float x_0, const float y_0, const float theta_0);
        void setPIDgain_translation(const float kp, const float ki, const float kb);
        void setPIDgain_rotation(const float kp, const float ki, const float kb);
        void setVelocityLimit(const float linear_v_max, const float angular_v_max);
        float getGyroZ(void) {return IMU.getGyroZ();}
        std::array<float, 3> getMag(void) {
            std::array<float, 3> mag;
            mag[0] = IMU.getMagX();
            mag[1] = IMU.getMagY();
            mag[2] = IMU.getMagZ();
            return mag;
        }
        void Log(bool islog) {log = islog;}
        std::array<float, 3> getPosition(void) const;
        bool Point2PointMove(const std::array<float, 3> target_point);
    private:
        static MecanumWMR* instance;
        ControllerParameter ControllerParam;
        Wheel_speed_t wheel_speed_ref, wheel_speed_est;
        bool isDone;
        float angle;
        std::array<float, 3> p_est;
        std::array<float, 3> starting_point;
        I2C i2c;
        MPU9250 IMU;
        TrajectoryGenerator tg;
        bool log {true};
        FILE *p_fptr {nullptr};
        FILE *v_fptr {nullptr};
        void localization(void);
        void pos_controller(const std::array<float, 3> p_ref, const std::array<float, 3> v_ref);
        float sat(float s,const float width, const float height);
        void ForwardKinematicTrans(float *const linear_v, const float *const wheel_v);
        void InverseKinematicTrans(const float *const linear_v, float *const wheel_v);
        void VelRobot2World(const float *const v_local, float *const v_global);
        void VelWorld2Robot(float *const v_local, const float *const v_global);
        void Log2File(const std::array<float, 3> p_ref, float g, const std::array<float, 3> m);
        void timer_start(void);
        void timer_stop(void);
        static void TimerHandler(int signum) {
            if (instance) {
                instance->timer_callback();
            }
        }
        void timer_callback(void);
};

#endif // __MWMR_H
