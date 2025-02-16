#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Wheel.h"
#include "MyStructs.h" // Make sure this defines ControllerData, TaskHandles, etc.

/* // For this test, we assume something like:
enum ControlMode {
    DEFAULT_MODE = 0,
    PWM_DIRECT_CONTROL = 1,
};



// Dummy definitions (adjust to your actual definitions)
struct MotorConnections {
    int dirPin;
    int pwmPin;
};

struct MotorData {
    ControlMode controlMode;
    int pwmValue;
    MotorConnections motorConnections;
};

struct ControllerProperties {
    uint32_t anglePIDFrequency;
    uint32_t speedPIDFrequency;
    uint32_t pwmUpdateFrequency;
};

struct ControllerData {
    ControllerProperties controllerProperties;
    // Assume we have at least one wheel (index 0)
    MotorData motorData[1];
};

struct WheelTaskHandles {
    TaskHandle_t wheel_run_task_handle;
    TaskHandle_t PWMDirectControlTaskHandle;
};

struct TaskHandles {
    // For our test, we only need one wheel task handle.
    WheelTaskHandles wheel_task_handles[1];
}; */
static const char *TAG = "WheelTest";
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wheel Test");

    // Setup dummy ControllerData with frequencies and initial motor settings.
    ControllerData controller_data = {};
    controller_data.controllerProperties.anglePIDFrequency = 50;   // 50 Hz (20ms period)
    controller_data.controllerProperties.speedPIDFrequency = 50;   // 50 Hz
    controller_data.controllerProperties.pwmUpdateFrequency = 100; // 100 Hz (10ms period)

    // Initialize motor data for wheel 0.
    controller_data.motorData = new MotorData[1];

    controller_data.motorData[0].controlMode = PWM_DIRECT_CONTROL;
    controller_data.motorData[0].pwmValue = 500; // Example PWM value (range assumed 0-1000)
    controller_data.motorData[0].motorConnections.dirPin = 18;
    controller_data.motorData[0].motorConnections.pwmPin = 19;

    // Setup dummy TaskHandles.
    TaskHandles task_handles = {};
    task_handles.wheel_task_handles = new WheelTaskHandles[1];
    task_handles.wheel_task_handles[0].wheel_run_task_handle = nullptr;
    task_handles.wheel_task_handles[0].PWMDirectControlTaskHandle = nullptr;
    ESP_LOGI(TAG, "All values set up correctly. Starting tasks now...");
    // Create the Wheel instance.
    Wheel *wheel = new Wheel(&controller_data, &task_handles);
    ESP_LOGI(TAG, "Object created successfully. Initializing wheel now...");
    // Start the wheel's main task.
    wheel->Start();
    ESP_LOGI(TAG, "Wheel task started");

    // Wait briefly to ensure the Run task is created.
    vTaskDelay(pdMS_TO_TICKS(1000));

    // --- Phase 1: PWM_DIRECT_CONTROL ---
    // Set the control mode to PWM_DIRECT_CONTROL.
    controller_data.motorData[0].controlMode = PWM_DIRECT_CONTROL;
    ESP_LOGI(TAG, "Sending notification to start PWM_DIRECT_CONTROL");
    // Notify the Wheel::Run task.
    xTaskNotify(task_handles.wheel_task_handles[0].wheel_run_task_handle,
                CONTROL_MODE_UPDATE, eSetBits);

    // Let it run in PWM_DIRECT_CONTROL mode for 10 seconds.
    vTaskDelay(pdMS_TO_TICKS(10000));

    // --- Phase 2: Default control (i.e. no PWM direct control task) ---
    // Change the control mode to default.
    controller_data.motorData[0].controlMode = OFF;
    ESP_LOGI(TAG, "Sending notification to revert to default control mode");
    xTaskNotify(task_handles.wheel_task_handles[0].wheel_run_task_handle,
                CONTROL_MODE_UPDATE, eSetBits);

    // Wait for 5 seconds.
    vTaskDelay(pdMS_TO_TICKS(5000));

    // --- Cleanup ---
    ESP_LOGI(TAG, "Stopping wheel tasks");
    wheel->Stop();
    delete wheel;
    ESP_LOGI(TAG, "Wheel instance deleted, test complete");

    // Optionally keep app_main running (or exit if appropriate).
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
