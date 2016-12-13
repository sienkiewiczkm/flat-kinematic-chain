#include "KinematicChainApplication.hpp"

#include <iostream>

#include "glm/gtc/matrix_transform.hpp"
#include "imgui.h"

#include "fw/Common.hpp"
#include "fw/DebugShapes.hpp"
#include "fw/Resources.hpp"

namespace kinematic
{

KinematicChainApplication::KinematicChainApplication()
{
}

KinematicChainApplication::~KinematicChainApplication()
{
}

void KinematicChainApplication::onCreate()
{
    ImGuiApplication::onCreate();

    _standard2DEffect = std::make_shared<fw::Standard2DEffect>();

    _quadGeometry = fw::createQuad2D({1.0f, 1.0f});

    _armController = std::make_shared<RoboticArmController>();
    _armRendering = std::make_shared<RoboticArmRendering>();

    _testTexture = std::make_shared<fw::Texture>(
        fw::getFrameworkResourcePath("textures/checker-base.png")
    );
}

void KinematicChainApplication::onDestroy()
{
    ImGuiApplication::onDestroy();
}

void KinematicChainApplication::onUpdate(
    const std::chrono::high_resolution_clock::duration& deltaTime
)
{
    ImGuiApplication::onUpdate(deltaTime);
    _armController->update(deltaTime);
    ImGui::ShowTestWindow();
}

void KinematicChainApplication::onRender()
{
    glClearColor(0.4f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    auto framebufferSize = getFramebufferSize();
    auto aspectRatio =
        static_cast<float>(framebufferSize.x) / framebufferSize.y;
    auto projection = glm::ortho(-aspectRatio, aspectRatio, -1.0f, 1.0f);

    _armRendering->setFirstArmLength(_armController->getFirstArmLength());
    _armRendering->setSecondArmLength(_armController->getSecondArmLenght());
    _armRendering->setArmsThickness(_armController->getVisualThickness());
    _armRendering->setAlphaAngle(_armController->getArmAlphaAngle());
    _armRendering->setBetaAngle(_armController->getArmBetaAngle());

    for (const auto& chunk: _armRendering->render())
    {
        _standard2DEffect->setModelMatrix(chunk.getModelMatrix());
        _standard2DEffect->setViewMatrix({});
        _standard2DEffect->setProjectionMatrix(projection);
        _standard2DEffect->setDiffuseTexture(_testTexture->getTextureId());
        _standard2DEffect->begin();
        chunk.getMesh()->render();
        _standard2DEffect->end();
    }

    ImGuiApplication::onRender();
}

bool KinematicChainApplication::onMouseButton(int button, int action, int mods)
{
    if (ImGuiApplication::onMouseButton(button, action, mods)) { return true; }
    return false;
}

bool KinematicChainApplication::onMouseMove(glm::dvec2 newPosition)
{
    if (ImGuiApplication::onMouseMove(newPosition)) { return true; }
    return false;
}

bool KinematicChainApplication::onScroll(double xoffset, double yoffset)
{
    if (fw::ImGuiApplication::onScroll(xoffset, yoffset))
        return true;

    return false;
}

bool KinematicChainApplication::onResize()
{
    return false;
}

}
