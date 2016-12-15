#pragma once

#include <algorithm>
#include <chrono>
#include <vector>
#include "glm/glm.hpp"

namespace kinematic
{

class RoboticArmController
{
public:
    RoboticArmController();
    ~RoboticArmController();

    void update(const std::chrono::high_resolution_clock::duration& deltaTime);

    float getVisualThickness() const;

    const std::vector<std::pair<float, float>>& getSolutions() const;

    float getFirstArmLength() const;
    float getSecondArmLength() const;

    void setTarget(const glm::vec2& position);
    glm::vec2 getTarget() const { return _ikTarget; }

    std::pair<glm::vec2, glm::vec2> buildConfiguration(float alpha, float beta);

private:
    bool solveInverseKinematics();

private:
    glm::vec2 _ikTarget;
    glm::vec2 _ikSecondTarget;

    float _firstArmLength, _secondArmLength;
    std::vector<std::pair<float, float>> _solutions;

    float _thickness;
    bool _lastSolveResult;
};

}
