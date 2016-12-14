#include "KinematicChainApplication.hpp"

#include <iostream>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/string_cast.hpp"
#include "imgui.h"

#include "fw/Common.hpp"
#include "fw/DebugShapes.hpp"
#include "fw/Resources.hpp"
#include "fw/GeometricIntersections.hpp"

namespace kinematic
{

KinematicChainApplication::KinematicChainApplication():
    _selectedConstraint{-1},
    _lmbDown{false},
    _isConstraintGrabbed{false}
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

    _constraints.push_back({{-1.0, -1.0},{-0.5, -0.5}});
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

    glm::vec2 p0{0,0};
    glm::vec2 p1 = _armController->getFirstArmEndPoint();
    glm::vec2 p2 = _armController->getSecondArmEndPoint();

    if (checkArmConstraintCollision(p0, p1)
        || checkArmConstraintCollision(p1, p2))
    {
        ImGui::TextColored(ImVec4{1.0f, 0, 0, 1.0f}, "Collision!");
    }

    if (ImGui::CollapsingHeader("Constraints"))
    {
        ImGui::Text("Select constraints by clicking, move by dragging");

        if (ImGui::Button("New"))
        {
            _constraints.push_back({
                {-0.5f, -0.5f},
                {0.5f, 0.5f}
            });
        }

        if (_selectedConstraint >= 0)
        {
            ImGui::SameLine();
            if (ImGui::Button("Delete"))
            {
                std::swap(
                    _constraints.back(),
                    _constraints[_selectedConstraint]
                );
                _constraints.pop_back();
                _selectedConstraint = -1;
            }
            else
            {
                auto& selected = _constraints[_selectedConstraint];
                auto size = selected.max - selected.min;
                ImGui::SliderFloat2("Size", glm::value_ptr(size), 0.01f, 10.0f);
                selected.max = selected.min + size;
            }
        }
    }

    if (ImGui::CollapsingHeader("Configuration space"))
    {
        if (ImGui::Button("Calculate"))
        {
            createAvailabilityMap();
            _availabilityMapCreated = true;
        }

        if (_availabilityMapCreated)
        {
            int w, h;
            int miplevel = 0;

            glBindTexture(GL_TEXTURE_2D, _availabilityMapTexture);

            glGetTexLevelParameteriv(
                GL_TEXTURE_2D,
                miplevel,
                GL_TEXTURE_WIDTH,
                &w
            );

            glGetTexLevelParameteriv(
                GL_TEXTURE_2D,
                miplevel,
                GL_TEXTURE_HEIGHT,
                &h
            );

            glBindTexture(GL_TEXTURE_2D, 0);

            auto availX = ImGui::GetContentRegionAvailWidth();
            float scale = std::min(1.0f, availX / w);

            void* imTex = (void*)_availabilityMapTexture;

            ImVec2 tex_screen_pos = ImGui::GetCursorScreenPos();
            ImGui::Text("%dx%d", w, h);
            ImGui::Image(
                imTex,
                ImVec2(w*scale, h*scale),
                ImVec2(0,0),
                ImVec2(1,1),
                ImColor(255,255,255,255),
                ImColor(255,255,255,128)
            );

            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();

                float focus_sz = 32.0f;
                float focus_x = ImGui::GetMousePos().x - tex_screen_pos.x - focus_sz * 0.5f;
                float focus_y = ImGui::GetMousePos().y - tex_screen_pos.y - focus_sz * 0.5f;

                if (focus_x < 0.0f) focus_x = 0.0f;
                else if (focus_x > w - focus_sz)
                {
                    focus_x = w - focus_sz;
                }

                if (focus_y < 0.0f) focus_y = 0.0f;
                else if (focus_y > h - focus_sz)
                {
                    focus_y = h - focus_sz;
                }

                ImGui::Text("Min: alpha=%.2f, beta=%.2f", focus_y, focus_x);
                ImGui::Text("Max: alpha=%.2f, beta=%.2f", focus_y + focus_sz, focus_x + focus_sz);
                ImVec2 uv0 = ImVec2((focus_x) / w, (focus_y) / h);
                ImVec2 uv1 = ImVec2((focus_x + focus_sz) / w, (focus_y + focus_sz) / h);
                ImGui::Image(imTex, ImVec2(128,128), uv0, uv1, ImColor(255,255,255,255), ImColor(255,255,255,128));
                ImGui::EndTooltip();
            }
        }
    }
}

void KinematicChainApplication::onRender()
{
    glClearColor(0.4f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    auto projection = getProjection();

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

    for (const auto& constraint: _constraints)
    {
        glm::vec2 position = (constraint.min + constraint.max) / 2.0f;
        glm::vec2 size = constraint.max - constraint.min;

        auto translation = glm::translate(
            glm::mat4{},
            glm::vec3{position, 0.0f}
        );

        auto scaling = glm::scale(
            glm::mat4{},
            glm::vec3{size, 1.0f}
        );

        _standard2DEffect->setModelMatrix(translation * scaling);
        _standard2DEffect->setViewMatrix({});
        _standard2DEffect->setProjectionMatrix(projection);
        _standard2DEffect->setDiffuseTexture(_testTexture->getTextureId());
        _standard2DEffect->begin();
        _quadGeometry->render();
        _standard2DEffect->end();
    }

    ImGuiApplication::onRender();
}

bool KinematicChainApplication::onMouseButton(int button, int action, int mods)
{
    if (ImGuiApplication::onMouseButton(button, action, mods)) { return true; }

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            if (!_lmbDown)
            {
                grabConstraint();
            }

            _lmbDown = true;
        }
        else
        {
            _isConstraintGrabbed = false;
            _lmbDown = false;
        }
    }

    return false;
}

bool KinematicChainApplication::onMouseMove(glm::dvec2 newPosition)
{
    if (ImGuiApplication::onMouseMove(newPosition)) { return true; }

    if (_isConstraintGrabbed)
    {
        auto newWorldPosition = getWorldCursorPos(newPosition);
        auto delta = newWorldPosition - _previousGrabWorldPosition;
        _constraints[_selectedConstraint].min += delta;
        _constraints[_selectedConstraint].max += delta;
        _previousGrabWorldPosition = newWorldPosition;
    }

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

glm::mat4 KinematicChainApplication::getProjection() const
{
    auto framebufferSize = getFramebufferSize();
    auto aspectRatio =
        static_cast<float>(framebufferSize.x) / framebufferSize.y;
    return glm::ortho(-aspectRatio, aspectRatio, -1.0f, 1.0f);
}

glm::vec2 KinematicChainApplication::getWorldCursorPos(
    glm::vec2 screenMousePos
) const
{
    auto screenSize = getWindowSize();

    glm::vec2 projMousePos{
        2.0f * (screenMousePos.x / screenSize.x) - 1.0f,
        -(2.0f * (screenMousePos.y / screenSize.y) - 1.0f)
    };

    glm::vec2 worldPos = glm::inverse(
        getProjection()
    ) * glm::vec4{projMousePos, 0.0, 1.0};

    return worldPos;
}

void KinematicChainApplication::grabConstraint()
{
    auto worldCursorPos = getWorldCursorPos(getCurrentMousePosition());
    for (auto i = 0; i < _constraints.size(); ++i)
    {
        if (_constraints[i].contains(worldCursorPos))
        {
            _selectedConstraint = i;
            _isConstraintGrabbed = true;
            _previousGrabWorldPosition = worldCursorPos;
            break;
        }
    }
}

bool KinematicChainApplication::checkArmConstraintCollision(
    glm::vec2 start,
    glm::vec2 end
) const
{
    for (const auto& constraint: _constraints)
    {
        if (checkSegmentAABBCollision(start, end, constraint))
        {
            return true;
        }
    }

    return false;
}

bool KinematicChainApplication::checkSegmentAABBCollision(
    const glm::vec2& start,
    const glm::vec2& end,
    const fw::AABB<glm::vec2>& aabb
) const
{
    if (aabb.contains(start) || aabb.contains(end))
    {
        return true;
    }

    auto pmin = aabb.min;
    auto pmax = aabb.max;
    glm::vec2 pminmax{pmin.x, pmax.y};
    glm::vec2 pmaxmin{pmax.x, pmin.y};

    return
        fw::intersectSegments<glm::vec2, float>(
            start,
            end,
            pmin,
            pmaxmin
        ).kind != fw::GeometricIntersectionKind::None
        || fw::intersectSegments<glm::vec2, float>(
            start,
            end,
            pmin,
            pminmax
        ).kind != fw::GeometricIntersectionKind::None
        || fw::intersectSegments<glm::vec2, float>(
            start,
            end,
            pmax,
            pminmax
        ).kind != fw::GeometricIntersectionKind::None
        || fw::intersectSegments<glm::vec2, float>(
            start,
            end,
            pmin,
            pmaxmin
        ).kind != fw::GeometricIntersectionKind::None;
}

void KinematicChainApplication::createAvailabilityMap()
{
    _availabilityMap.clear();
    for (auto alphaStep = 0; alphaStep < 360; ++alphaStep)
    {
        float alpha = glm::radians(static_cast<float>(alphaStep));
        for (auto betaStep = 0; betaStep < 360; ++betaStep)
        {
            float beta = glm::radians(static_cast<float>(betaStep));
            _availabilityMap.push_back(checkConfiguration(alpha, beta));
        }
    }

    createAvailabilityMapTexture();
}

void KinematicChainApplication::createAvailabilityMapTexture()
{
    std::vector<unsigned char> image;
    for (const auto& state: _availabilityMap)
    {
        if (state)
        {
            image.push_back(0);
            image.push_back(255);
            image.push_back(0);
        }
        else
        {
            image.push_back(255);
            image.push_back(0);
            image.push_back(0);
        }
    }

    glGenTextures(1, &_availabilityMapTexture);
    glBindTexture(GL_TEXTURE_2D, _availabilityMapTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 360, 360, 0,
        GL_RGB, GL_UNSIGNED_BYTE, image.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

bool KinematicChainApplication::checkConfiguration(float alpha, float beta)
{
    auto config = _armController->buildConfiguration(alpha, beta);
    return !(checkArmConstraintCollision({0, 0}, config.first)
        || checkArmConstraintCollision(config.first, config.second));
}

}
