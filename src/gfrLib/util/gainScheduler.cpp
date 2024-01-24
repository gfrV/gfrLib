#include "gfrLib/util/gainScheduler.hpp"

using namespace gfrLib;

/**
 * @brief Construct a new gain Scheduler::gain Scheduler object
 * 
 * @param min 
 * @param max 
 * @param roundness 
 * @param thickness 
 */
gainScheduler::gainScheduler(float min, float max, float roundness, float thickness):
    min(min),
    max(max),
    roundness(roundness),
    thickness(thickness) {}

/**
 * @brief 
 * 
 * @param error 
 * @param min 
 * @param max 
 * @param roundness 
 * @param thickness 
 * @return float 
 * 
 */
float gainScheduler::gainFunction(float error, float min, float max, float roundness, float thickness){
    return (min - max) * std::pow(std::abs(error), roundness) /
                       (std::pow(std::abs(error), roundness) + std::pow(thickness, roundness)) +
                   max;
}
/**
 * @brief 
 * 
 * @param selectedPID 
 * @param error 
 * @return * float 
 */
float gainScheduler::update(PID selectedPID, float error){
    selectedPID.kp = gainFunction(error, min, max, roundness, thickness);
    return selectedPID.updateerror(error);
}