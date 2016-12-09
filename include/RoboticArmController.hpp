#pragma once

#include <chrono>

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

private:
    float _thickness;
    float _alphaAngle;
    float _betaAngle;
};

}
