#pragma once
#include <stdint.h>

class StepCounter {
public:
    void begin();
    void update(); // Call this frequently from your main loop
    uint32_t steps() const { return stepCount_; }
    // Simple calibration to remove bias, call while still.
    void calibrate();

private:
    // Algorithm state
    enum State { S_IDLE, S_ASCEND, S_DESCEND };
    State state_ = S_IDLE;
    uint32_t lastStepMs_ = 0;
    uint32_t peakTimeMs_ = 0;
    float peakAccel_ = 0.0f;
    float baseline_ = 0.0f; // Dynamic baseline

    // Tuning parameters (start with these, adjust after testing)
    const float STEP_THRESHOLD = 1.2f; // in g (1g = 9.81 m/s²)
    const uint32_t MIN_STEP_MS = 250;  // debounce time between steps
    const uint32_t MAX_STEP_MS = 2000; // a step shouldn't take longer than this
    
    volatile uint32_t stepCount_ = 0;
    
    bool readAccel(float &ax, float &ay, float &az);
};