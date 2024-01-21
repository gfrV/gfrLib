#pragma once

class PID {
    public:
        float kp;
        float ki;
        float kd;

        PID(float kp, float ki, float kd, const float iMax, const float settleError,
            const float settleTime, const float maxSettleError, const float maxSettleTime, const float maxTime);

        float update(float target, float actual);

        float updateerror(float error);

        void reset();

        bool isSettled();
    private:
        const float iMax;
        const float settleError;
        const float settleTime;
        const float maxSettleError;
        const float maxSettleTime;
        const float maxTime;

        float lastError;
        float errorSum;
        float settleTimer;
        float maxSettleTimer;
        float maxTimer;
};