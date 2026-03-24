#ifndef TIMER_TASK_HPP
#define TIMER_TASK_HPP

#include <stdint.h>

typedef void (*TimerCallback)(void);

class TimerTask {
public:
    struct Task {
        TimerCallback callback;
        int32_t durationMs;
    };

    /**
     * @brief Run a single task immediately, clearing any pending tasks.
     * @param callback Function to call
     * @param durationMs Duration in milliseconds
     */
    static void RunForDuration(TimerCallback callback, int32_t durationMs);

    /**
     * @brief Add a task to the execution queue.
     * @param callback Function to call
     * @param durationMs Duration in milliseconds
     * @return true if added, false if queue is full
     */
    static bool AddTask(TimerCallback callback, int32_t durationMs);

    /**
     * @brief Clear all pending tasks.
     */
    static void ClearTasks();

    /**
     * @brief Update the timer task state. Should be called from a timer interrupt (e.g. every 10ms).
     */
    static void Update();
    
    /**
     * @brief Check if all tasks are finished
     */
    static bool IsFinished();

private:
    static const int MAX_TASKS = 10;
    static Task taskQueue[MAX_TASKS];
    static volatile int queueHead;
    static volatile int queueTail;
    static volatile int queueCount;
    
    static TimerCallback currentCallback;
    static volatile int32_t currentRemainingTimeMs;
};

#endif // TIMER_TASK_HPP
