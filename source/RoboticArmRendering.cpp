#include "RoboticArmRendering.hpp"
#include <cmath>
#include "glm/gtc/matrix_transform.hpp"
#include "fw/DebugShapes.hpp"

namespace kinematic
{

RoboticArmRendering::RoboticArmRendering():
    _armsThickness{0.05f},
    _firstArmLength{0.5f},
    _secondArmLength{0.5f},
    _alphaAngle{},
    _betaAngle{}
{
    _quad = fw::createQuad2D({1.0f, 1.0f});
}

RoboticArmRendering::~RoboticArmRendering()
{
}

void RoboticArmRendering::setArmsThickness(float thickness)
{
    _armsThickness = thickness;
}

void RoboticArmRendering::setFirstArmLength(float length)
{
    _firstArmLength = length;
}

void RoboticArmRendering::setSecondArmLength(float length)
{
    _secondArmLength = length;
}

void RoboticArmRendering::setAlphaAngle(float alphaAngle)
{
    _alphaAngle = alphaAngle;
}

void RoboticArmRendering::setBetaAngle(float betaAngle)
{
    _betaAngle = betaAngle;
}

std::vector<fw::GeometryChunk> RoboticArmRendering::render()
{
    glm::vec2 p0{0.0f, 0.0f};

    glm::vec2 p1 = p0 + glm::vec2{
        _firstArmLength * cosf(_alphaAngle),
        _firstArmLength * sinf(_alphaAngle)
    };

    glm::vec2 p2 = p1 + glm::vec2{
        _secondArmLength * cosf(_alphaAngle + _betaAngle),
        _secondArmLength * sinf(_alphaAngle + _betaAngle)
    };

    glm::mat4 firstTransform = glm::translate(
        glm::mat4{},
        glm::vec3{(p0 + p1) / 2.0f, 0.0f}
    );

    firstTransform = glm::rotate(
        firstTransform,
        _alphaAngle,
        {0.0f, 0.0f, 1.0f}
    );

    firstTransform = glm::scale(
        firstTransform,
        {_firstArmLength, _armsThickness, 1.0f}
    );

    glm::mat4 secondTransform = glm::translate(
        glm::mat4{},
        glm::vec3{(p1 + p2) / 2.0f, 0.0f}
    );

    secondTransform = glm::rotate(
        secondTransform,
        _alphaAngle + _betaAngle,
        {0.0f, 0.0f, 1.0f}
    );

    secondTransform = glm::scale(
        secondTransform,
        {_secondArmLength, _armsThickness, 1.0f}
    );

    return {
        { _quad, nullptr, glm::mat4{firstTransform} },
        { _quad, nullptr, glm::mat4{secondTransform} }
    };
}

}
