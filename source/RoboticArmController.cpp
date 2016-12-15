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
    _lastSolveResult{true}
{
    _solutions.push_back({0, 0});
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
        ImGui::DragFloat("Alpha (rad)", &_solutions[0].first, 0.01f);
        ImGui::DragFloat("Beta (rad)", &_solutions[0].second, 0.01f);
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

const std::vector<std::pair<float, float>>& RoboticArmController::getSolutions(
) const
{
    return _solutions;
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

    _solutions.clear();
    for (auto i = 0; i < intersections.size(); ++i)
    {
        auto alphaAngle = atan2f(intersections[i].y, intersections[i].x);
        auto betaAngle = atan2f(
            _ikTarget.y - intersections[i].y,
            _ikTarget.x - intersections[i].x
        ) - alphaAngle;

        _solutions.push_back({alphaAngle, betaAngle});
    }

    return true;
}

float RoboticArmController::getFirstArmLength() const
{
    return _firstArmLength;
}

float RoboticArmController::getSecondArmLength() const
{
    return _secondArmLength;
}

void RoboticArmController::setTarget(const glm::vec2& position)
{
    _ikTarget = position;
}

std::pair<glm::vec2, glm::vec2> RoboticArmController::buildConfiguration(
    float alpha,
    float beta
)
{
    glm::vec2 p0{0.0f, 0.0f};

    glm::vec2 p1 = p0 + glm::vec2{
        _firstArmLength * cosf(alpha),
        _firstArmLength * sinf(alpha)
    };

    glm::vec2 p2 = p1 + glm::vec2{
        _secondArmLength * cosf(alpha + beta),
        _secondArmLength * sinf(alpha + beta)
    };

    return {p1, p2};
}

}
