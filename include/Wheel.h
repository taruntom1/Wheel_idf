#ifndef WHEEL_H
#define WHEEL_H

#include "MyStructs.h"
#include "MotorDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "EncoderPulseReader.h"

struct LoopDelays

{
    uint32_t anglePID = 0;
    uint32_t speedPID = 0;
    uint32_t PWM = 0;
    uint32_t encoder = 0;
};

class Wheel
{
private:
    const char *TAG = "Wheel";

    inline static uint8_t wheel_instance_count = 0;
    uint8_t wheel_id;

    LoopDelays loop_delays;
    MotorData *motor_data;

    MotorDriverConfig motor_config;
    MotorDriver *motorDriver;

    pcnt_config_t encoder_config;
    EncoderPulseReader *encoder;

    WheelTaskHandles *task_handles;

    void PWMDirectControl();
    void OdoBroadcast();

    void Run();

public:
    Wheel(ControllerData *controller_data, TaskHandles *task_handles);
    ~Wheel();

    void Start();
    void Stop();
};

#endif // WHEEL_H