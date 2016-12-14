#pragma once

#include <chrono>
#include <algorithm>
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
    float getArmAlphaAngle() const;
    float getArmBetaAngle() const;
    float getFirstArmLength() const;
    float getSecondArmLenght() const;

    glm::vec2 getFirstArmEndPoint() const;
    glm::vec2 getSecondArmEndPoint() const;

    std::pair<glm::vec2, glm::vec2> buildConfiguration(float alpha, float beta);

private:
    bool solveInverseKinematics();

private:
    glm::vec2 _ikTarget;
    float _firstArmLength, _secondArmLength;
    float _thickness;
    float _alphaAngle;
    float _betaAngle;
    bool _lastSolveResult;
};

}
