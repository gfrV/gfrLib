#include "pid.hpp"
#include <cmath>

namespace gfrLib {

class gainScheduler {
    public:
        /**
         * @brief Construct a new gain Scheduler object
         * 
         * @param min min speed the robot could travel at
         * @param max max speed the robot could travel at
         * @param roundness @note how smooth the movement should be(greater than 0)
         * @param thickness 
         */
        gainScheduler(float min, float max, float roundness, float thickness);

        /**
         * @brief 
         * 
         * @param selectedPID 
         * @param error 
         * @return float 
         */
        float update(PID selectedPID, float error);

    private:
        //VALUES
        float min;
        float max;
        float roundness;
        float thickness;
        
        //gain func
        float gainFunction(float error, float min, float max, float roundness, float thickness);
};
}; // namespace gfrLib
