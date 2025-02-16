#include "Wheel.h"

Wheel::Wheel(ControllerData *controller_data, TaskHandles *task_handles)
{
    wheel_id = wheel_instance_count;
    wheel_instance_count++;

    this->motor_data = &controller_data->motorData[wheel_id];
    this->loop_delays.anglePID = 1000 / controller_data->controllerProperties.anglePIDFrequency;
    this->loop_delays.speedPID = 1000 / controller_data->controllerProperties.speedPIDFrequency;
    this->loop_delays.PWM = 1000 / controller_data->controllerProperties.pwmUpdateFrequency;

    motor_config = {
        .directionPin = (gpio_num_t)motor_data->motorConnections.dirPin,
        .pwmPin = (gpio_num_t)motor_data->motorConnections.pwmPin,
        .clockFrequencyHz = 1000000,
        .pwmResolution = 1000};

    motorDriver = new MotorDriver(motor_config);

    motorDriver->init();

    this->task_handles = &task_handles->wheel_task_handles[wheel_id];

    this->task_handles->wheel_run_task_handle = nullptr;
    this->task_handles->PWMDirectControlTaskHandle = nullptr;
}

Wheel::~Wheel()
{
    delete motorDriver;
    if (task_handles->PWMDirectControlTaskHandle != nullptr)
    {
        vTaskDelete(task_handles->PWMDirectControlTaskHandle);
        task_handles->PWMDirectControlTaskHandle = nullptr;
    }

    if (task_handles->wheel_run_task_handle != nullptr)
    {
        vTaskDelete(task_handles->wheel_run_task_handle);
        task_handles->wheel_run_task_handle = nullptr;
    }
}

void Wheel::Start()
{
    if (task_handles->wheel_run_task_handle == nullptr)
    { // Prevent duplicate tasks
        xTaskCreate([](void *param)
                    { static_cast<Wheel *>(param)->Run(); },
                    "Wheel run task", 1024, this, 5, &task_handles->wheel_run_task_handle);
    }
}

void Wheel::Stop()
{
    if (task_handles->wheel_run_task_handle != nullptr)
    {
        vTaskDelete(task_handles->wheel_run_task_handle);
        task_handles->wheel_run_task_handle = nullptr;
    }
}

void Wheel::Run()
{
    uint32_t receivedFlags;

    while (true)
    {
        xTaskNotifyWait(0, ULONG_MAX, &receivedFlags, portMAX_DELAY);
        if (receivedFlags & CONTROL_MODE_UPDATE)
        {
            if (task_handles->PWMDirectControlTaskHandle != nullptr)
            {
                vTaskDelete(task_handles->PWMDirectControlTaskHandle);
            }

            switch (motor_data->controlMode)
            {
            case PWM_DIRECT_CONTROL:
                xTaskCreate([](void *param)
                            { static_cast<Wheel *>(param)->PWMDirectControl(); }, "PWMDirectControl", 256, this, 5, &task_handles->PWMDirectControlTaskHandle);
                break;

            default:
                break;
            }
        }
    }
}

void Wheel::PWMDirectControl()
{
    while (true)
    {
        motorDriver->setSpeed(static_cast<float>(motor_data->pwmValue) / 1000.0f);
        vTaskDelay(pdMS_TO_TICKS(loop_delays.PWM));
    }
}
