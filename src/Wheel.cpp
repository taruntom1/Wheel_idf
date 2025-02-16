#include "Wheel.h"
#include "esp_log.h"

static const char *TAG = "Wheel";

Wheel::Wheel(ControllerData *controller_data, TaskHandles *task_handles)
{
    ESP_LOGI(TAG, "Initializing Wheel instance %d", wheel_instance_count);

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
    ESP_LOGI(TAG, "MotorDriver initialized for Wheel %d", wheel_id);

    this->task_handles = &task_handles->wheel_task_handles[wheel_id];
    this->task_handles->wheel_run_task_handle = nullptr;
    this->task_handles->PWMDirectControlTaskHandle = nullptr;
}

Wheel::~Wheel()
{
    ESP_LOGI(TAG, "Destroying Wheel instance %d", wheel_id);
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
    ESP_LOGI(TAG, "Wheel instance %d destroyed", wheel_id);
}

void Wheel::Start()
{
    if (task_handles->wheel_run_task_handle == nullptr)
    { // Prevent duplicate tasks
        ESP_LOGI(TAG, "Starting Wheel run task for instance %d", wheel_id);
        xTaskCreate([](void *param)
                    { static_cast<Wheel *>(param)->Run(); },
                    "Wheel run task", 3000, this, 5, &task_handles->wheel_run_task_handle);
    }
}

void Wheel::Stop()
{
    if (task_handles->wheel_run_task_handle != nullptr)
    {
        ESP_LOGI(TAG, "Stopping Wheel run task for instance %d", wheel_id);
        vTaskDelete(task_handles->wheel_run_task_handle);
        task_handles->wheel_run_task_handle = nullptr;
    }
}

void Wheel::Run()
{
    ESP_LOGI(TAG, "Wheel instance %d Run task started", wheel_id);
    uint32_t receivedFlags;

    while (true)
    {
        xTaskNotifyWait(0, ULONG_MAX, &receivedFlags, portMAX_DELAY);


        if (receivedFlags & CONTROL_MODE_UPDATE)
        {
            ESP_LOGI(TAG, "Wheel %d updating control mode", wheel_id);
            if (task_handles->PWMDirectControlTaskHandle != nullptr)
            {
                vTaskDelete(task_handles->PWMDirectControlTaskHandle);
                task_handles->PWMDirectControlTaskHandle = nullptr;
            }

            switch (motor_data->controlMode)
            {
            case PWM_DIRECT_CONTROL:
                ESP_LOGI(TAG, "Wheel %d entering PWM Direct Control mode", wheel_id);
                xTaskCreate([](void *param)
                            { static_cast<Wheel *>(param)->PWMDirectControl(); }, "PWMDirectControl", 2500, this, 5, &task_handles->PWMDirectControlTaskHandle);
                break;

            default:
                ESP_LOGW(TAG, "Wheel %d unknown control mode", wheel_id);
                break;
            }
        }
    }
}

void Wheel::PWMDirectControl()
{
    ESP_LOGI(TAG, "Wheel %d PWM Direct Control task started", wheel_id);
    while (true)
    {
        float speed = static_cast<float>(motor_data->pwmValue) / 1000.0f;
        ESP_LOGV(TAG, "Wheel %d setting speed: %.3f", wheel_id, speed);
        motorDriver->setSpeed(speed);
        vTaskDelay(pdMS_TO_TICKS(loop_delays.PWM));
    }
}
