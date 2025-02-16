#ifndef WHEEL_H
#define WHEEL_H

#include "MyStructs.h"
#include "MotorDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct LoopDelays

{
    uint32_t anglePID = 0;
    uint32_t speedPID = 0;
    uint32_t PWM = 0;
};

class Wheel
{
private:
    static uint8_t wheel_instance_count;
    uint8_t wheel_id;

    LoopDelays loop_delays;
    MotorData* motor_data;

    MotorDriverConfig motor_config;
    MotorDriver *motorDriver;

    WheelTaskHandles *task_handles;

    void PWMDirectControl();

    #define CONTROL_MODE_UPDATE (1 << 0)
    void Run();

public:
    Wheel(ControllerData* controller_data, TaskHandles *task_handles);

    ~Wheel();

    void Start();
    void Stop();

    
};

#endif // WHEEL_H