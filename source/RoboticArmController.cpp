#include "RoboticArmController.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "imgui.h"
#include "fw/GeometricIntersections.hpp"

namespace kinematic
{

RoboticArmController::RoboticArmController():
    _firstArmLength{0.3f},
    _secondArmLength{0.3f},
    _thickness{0.01f},
    _alphaAngle{},
    _betaAngle{},
    _lastSolveResult{true}
{
}

RoboticArmController::~RoboticArmController()
{
}

void RoboticArmController::update(
    const std::chrono::high_resolution_clock::duration& deltaTime
)
{
    if (!ImGui::Begin("Robotic Arm Controller"))
    {
        ImGui::End();
        return;
    }

    ImGui::DragFloat(
        "First arm length",
        &_firstArmLength,
        0.01f,
        0.0f,
        100.0f
    );

    ImGui::DragFloat(
        "Second arm length",
        &_secondArmLength,
        0.01f,
        0.0f,
        100.0f
    );

    if (ImGui::CollapsingHeader("Forward kinematics"))
    {
        ImGui::DragFloat("Alpha (rad)", &_alphaAngle, 0.01f);
        ImGui::DragFloat("Beta (rad)", &_betaAngle, 0.01f);
    }

    if (ImGui::CollapsingHeader("Inverse kinematics"))
    {
        ImGui::DragFloat2("Target", glm::value_ptr(_ikTarget), 0.1f);
        if (ImGui::Button("Reach target"))
        {
            _lastSolveResult = solveInverseKinematics();
        }

        if (!_lastSolveResult)
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                "Solution not found."
            );
        }
    }

    if (ImGui::CollapsingHeader("Visuals"))
    {
        ImGui::DragFloat("Visual thickness", &_thickness, 0.01f, 0.0f, 1.0f);
    }

    ImGui::End();
}

float RoboticArmController::getVisualThickness() const
{
    return _thickness;
}

float RoboticArmController::getArmAlphaAngle() const
{
    return _alphaAngle;
}

float RoboticArmController::getArmBetaAngle() const
{
    return _betaAngle;
}

bool RoboticArmController::solveInverseKinematics()
{
    auto intersections = fw::intersectCircles<glm::vec2, float>(
        {0, 0},
        _firstArmLength,
        _ikTarget,
        _secondArmLength
    );

    if (intersections.size() == 0)
    {
        return false;
    }

    _alphaAngle = atan2f(intersections[0].y, intersections[0].x);
    _betaAngle = atan2f(
        _ikTarget.y - intersections[0].y,
        _ikTarget.x - intersections[0].x
    ) - _alphaAngle;

    return true;
}

float RoboticArmController::getFirstArmLength() const
{
    return _firstArmLength;
}

float RoboticArmController::getSecondArmLenght() const
{
    return _secondArmLength;
}

glm::vec2 RoboticArmController::getFirstArmEndPoint() const
{
    glm::vec2 p0{0.0f, 0.0f};

    glm::vec2 p1 = p0 + glm::vec2{
        _firstArmLength * cosf(_alphaAngle),
        _firstArmLength * sinf(_alphaAngle)
    };

    return p1;
}

glm::vec2 RoboticArmController::getSecondArmEndPoint() const
{
    auto p1 = getFirstArmEndPoint();

    glm::vec2 p2 = p1 + glm::vec2{
        _secondArmLength * cosf(_alphaAngle + _betaAngle),
        _secondArmLength * sinf(_alphaAngle + _betaAngle)
    };

    return p2;
}

}
