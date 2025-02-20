#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Wheel.h"
#include "MyStructs.h" // Ensure it defines ControllerData, TaskHandles, etc.
#include <iostream>

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
        vTaskDelay(pdMS_TO_TICKS(2000)); // Print every 2 seconds
    }
}

static const char *TAG = "WheelTest";
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wheel Test");

    // Setup ControllerData with motor and odometry settings.
    ControllerData controller_data = {};
    controller_data.motorData = new MotorData[1];
    
    controller_data.motorData[0].updateFrequenciesWheel.anglePID = 50;
    controller_data.motorData[0].updateFrequenciesWheel.speedPID = 50;
    controller_data.motorData[0].updateFrequenciesWheel.pwm = 100;
    controller_data.motorData[0].controlMode = PWM_DIRECT_CONTROL;
    controller_data.motorData[0].pwmValue = 500;
    controller_data.motorData[0].motorConnections.dirPin = 18;
    controller_data.motorData[0].motorConnections.pwmPin = 19;
    controller_data.motorData[0].motorConnections.encPinA = 20;
    controller_data.motorData[0].motorConnections.encPinB = 21;
    
    // Enable odometry broadcasting
    controller_data.motorData[0].odoBroadcastStatus.angleBroadcast = true;
    controller_data.motorData[0].odoBroadcastStatus.speedBroadcast = true;
    controller_data.controllerProperties.odoBroadcastFrequency = 10; // 10 Hz
    
    // Setup TaskHandles
    TaskHandles task_handles = {};
    task_handles.wheel_task_handles = new WheelTaskHandles[1];
    task_handles.wheel_task_handles[0].wheel_run_task_handle = nullptr;
    task_handles.wheel_task_handles[0].PWMDirectControlTaskHandle = nullptr;
    task_handles.wheel_task_handles[0].OdoBroadcast = nullptr;
    
    ESP_LOGI(TAG, "All values set up correctly. Starting tasks now...");

    // Create the Wheel instance
    Wheel *wheel = new Wheel(&controller_data, &task_handles, 0);
    ESP_LOGI(TAG, "Object created successfully. Initializing wheel now...");
    
    wheel->Start();
    ESP_LOGI(TAG, "Wheel task started");

    // Wait to ensure the Run task is created.
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Start monitoring task
    xTaskCreate(monitorTask, "MonitorTask", 4096, NULL, 1, NULL);

    // --- Phase 1: PWM_DIRECT_CONTROL ---
    controller_data.motorData[0].controlMode = PWM_DIRECT_CONTROL;
    ESP_LOGI(TAG, "Sending notification to start PWM_DIRECT_CONTROL");
    xTaskNotify(task_handles.wheel_task_handles[0].wheel_run_task_handle, CONTROL_MODE_UPDATE, eSetBits);
    vTaskDelay(pdMS_TO_TICKS(10000));

    // --- Phase 2: ODO_BROADCAST_STATUS_UPDATE ---
    ESP_LOGI(TAG, "Sending notification to start OdoBroadcast");
    xTaskNotify(task_handles.wheel_task_handles[0].wheel_run_task_handle, ODO_BROADCAST_STATUS_UPDATE, eSetBits);
    for(int i = 0; i < 100; i++){
        std::cout << "Angle : " << controller_data.motorData[0].odometryData.angle << "\t";
        std::cout << "speed : " << controller_data.motorData[0].odometryData.rpm << "\t";
        std::cout << "pwm : " << controller_data.motorData[0].pwmValue << std::endl;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    

    // --- Phase 3: Default control mode ---
    controller_data.motorData[0].controlMode = OFF;
    ESP_LOGI(TAG, "Sending notification to revert to default control mode");
    xTaskNotify(task_handles.wheel_task_handles[0].wheel_run_task_handle, CONTROL_MODE_UPDATE, eSetBits);
    vTaskDelay(pdMS_TO_TICKS(5000));

    // --- Cleanup ---
    ESP_LOGI(TAG, "Stopping wheel tasks");
    wheel->Stop();
    delete wheel;
    ESP_LOGI(TAG, "Wheel instance deleted, test complete");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
