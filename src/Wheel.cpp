#include "Wheel.h"

Wheel::Wheel(ControllerData *controller_data)
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
}

Wheel::~Wheel()
{
    delete motorDriver;
}


void Wheel::Run(void *pvParameter)
{
    uint32_t receivedFlags;

    TaskHandle_t PWMDirectControlTaskHandle = NULL;

    while (true)
    {
        xTaskNotifyWait(0, ULONG_MAX, &receivedFlags, portMAX_DELAY);
        if (receivedFlags & CONTROL_MODE_UPDATE)
        {
            if (PWMDirectControlTaskHandle != NULL)
            {
                vTaskDelete(PWMDirectControlTaskHandle);
            }

            switch (motor_data->controlMode)
            {
            case PWM_DIRECT_CONTROL:
                xTaskCreate([](void *param)
                            { static_cast<Wheel *>(param)->PWMDirectControl(); }, "PWMDirectControl", 256, this, 5, &PWMDirectControlTaskHandle);
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
