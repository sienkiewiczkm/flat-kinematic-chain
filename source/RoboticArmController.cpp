#include "RoboticArmController.hpp"
#include "imgui.h"

namespace kinematic
{

RoboticArmController::RoboticArmController():
    _alphaAngle{},
    _betaAngle{}
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

    ImGui::DragFloat("Visual thickness", &_thickness, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Alpha (rad)", &_alphaAngle, 0.01f);
    ImGui::DragFloat("Beta (rad)", &_betaAngle, 0.01f);

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

}
