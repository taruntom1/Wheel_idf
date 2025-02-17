#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Wheel.h"
#include "MyStructs.h" // Make sure this defines ControllerData, TaskHandles, etc.

void printTaskInfo()
{
    char buffer[512]; // Buffer to hold task list output
    vTaskList(buffer);
    ESP_LOGI("TASKS", "\nTask Name\tState\tPrio\tStack Left\tTask Num\n%s", buffer);
}

void monitorTask(void *pvParameters)
{
    while (1)
    {
        printTaskInfo();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Print every 5 seconds
    }
}

static const char *TAG = "WheelTest";
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wheel Test");

    // Setup dummy ControllerData with frequencies and initial motor settings.
    ControllerData controller_data = {};


    // Initialize motor data for wheel 0.
    controller_data.motorData = new MotorData[1];

    controller_data.motorData[0].updateFrequenciesWheel.anglePID = 50;   // 50 Hz (20ms period)
    controller_data.motorData[0].updateFrequenciesWheel.speedPID = 50;   // 50 Hz
    controller_data.motorData[0].updateFrequenciesWheel.pwm = 100; // 100 Hz (10ms period)
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

    // Create a monitor task to check the status of all tasks.
    xTaskCreate(monitorTask, "MonitorTask", 4096, NULL, 1, NULL);

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
