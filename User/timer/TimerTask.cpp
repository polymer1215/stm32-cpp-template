#include "TimerTask.hpp"

TimerCallback TimerTask::currentCallback = nullptr;
volatile int32_t TimerTask::currentRemainingTimeMs = 0;

TimerTask::Task TimerTask::taskQueue[MAX_TASKS];
volatile int TimerTask::queueHead = 0;
volatile int TimerTask::queueTail = 0;
volatile int TimerTask::queueCount = 0;

void TimerTask::RunForDuration(TimerCallback callback, int32_t durationMs) {
    ClearTasks();
    AddTask(callback, durationMs);
}

bool TimerTask::AddTask(TimerCallback callback, int32_t durationMs) {
    if (queueCount >= MAX_TASKS) {
        return false;
    }
    
    taskQueue[queueTail].callback = callback;
    taskQueue[queueTail].durationMs = durationMs;
    
    queueTail = (queueTail + 1) % MAX_TASKS;
    queueCount++;
    
    return true;
}

void TimerTask::ClearTasks() {
    queueHead = 0;
    queueTail = 0;
    queueCount = 0;
    currentCallback = nullptr;
    currentRemainingTimeMs = 0;
}

void TimerTask::Update() {
    // Check if current task is running
    if (currentRemainingTimeMs > 0) {
        if (currentCallback != nullptr) {
            currentCallback();
        }
        currentRemainingTimeMs -= 10;
    } 
    // If current task finished (or never started), try to get next task
    else {
        if (queueCount > 0) {
            // Load next task from queue
            currentCallback = taskQueue[queueHead].callback;
            currentRemainingTimeMs = taskQueue[queueHead].durationMs;
            
            queueHead = (queueHead + 1) % MAX_TASKS;
            queueCount--;
            
            // Execute the new task immediately for this tick
            if (currentCallback != nullptr) {
                currentCallback();
            }
            currentRemainingTimeMs -= 10;
        } else {
            // No more tasks
            currentCallback = nullptr;
        }
    }
}

bool TimerTask::IsFinished() {
    return (currentRemainingTimeMs <= 0) && (queueCount == 0);
}
