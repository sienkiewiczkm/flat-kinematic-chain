#pragma once

#include <memory>
#include <vector>
#include "fw/GeometryChunk.hpp"
#include "fw/Mesh.hpp"
#include "fw/Vertices.hpp"

namespace kinematic
{

class RoboticArmRendering
{
public:
    RoboticArmRendering();
    ~RoboticArmRendering();

    void setArmsThickness(float thickness);
    void setFirstArmLength(float length);
    void setSecondArmLength(float length);

    void setAlphaAngle(float alphaAngle);
    void setBetaAngle(float betaAngle);

    std::vector<fw::GeometryChunk> render();

private:
    float _armsThickness;
    float _firstArmLength, _secondArmLength;
    float _alphaAngle, _betaAngle;
    std::shared_ptr<fw::Mesh<fw::StandardVertex2D>> _quad;
};

}
