#pragma once
#include <functional>

class PID {
    public:

        float kp;
        float ki;
        float kd;
        /**
         * @brief sets up PID
         * 
         * @param kp proportional value
         * @param ki integral value
         * @param kd derivative value
         * @param iMax max integral windup
         * @param settleError default settle error
         * @param settleTime default settle time
         * @param maxSettleError max settle error
         * @param maxSettleTime max settle time
         * @param maxTime how long the entire movement should take
         * 
         */
        PID(float kp, float ki, float kd, const float iMax, const float settleError, const float settleTime,
            const float maxSettleError, const float maxSettleTime, const float maxTime);

        /**
         * @brief updates the PID values in timestep
         *
         * @param target target position for the PID
         * @param actual the position it is currently at
         */
        float update(float target, float actual);

        /**
         * @brief calculates error in timestep
         *
         * @param error current error between the target and actual position
         */
        float updateerror(float error);

        /**
         * @brief resets the PID.
         *
         */
        void reset();

        /**
         * @return is the robot settled or not
         *
         */
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