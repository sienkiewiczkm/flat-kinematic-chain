#include "KinematicChainApplication.hpp"

#include <iostream>
#include <queue>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/string_cast.hpp"
#include "glm/gtx/color_space.hpp"
#include "imgui.h"

#include "fw/Common.hpp"
#include "fw/DebugShapes.hpp"
#include "fw/Resources.hpp"
#include "fw/GeometricIntersections.hpp"

namespace kinematic
{

KinematicChainApplication::KinematicChainApplication():
    _searchMapAvailable{false},
    _selectedConstraint{-1},
    _isConstraintGrabbed{false},
    _frameTime{0.25f},
    _animationEnabled{false},
    _frameAnimationPassed{0.0f},
    _currentAnimationStep{0}
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

    if (ImGui::CollapsingHeader("Constraints"))
    {
        ImGui::Text("Select constraints by clicking, move by dragging");

        if (ImGui::Button("New"))
        {
            _constraints.push_back({
                {-0.5f, -0.5f},
                {0.5f, 0.5f}
            });

            _selectedConstraint = _constraints.size() - 1;
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
                ImGui::DragFloat2(
                    "Size",
                    glm::value_ptr(size),
                    0.05f,
                    0.01f,
                    10.0f
                );

                auto center = (selected.min + selected.max) / 2.0f;
                selected.min = center - size * 0.5f;
                selected.max = center + size * 0.5f;
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

            showTexturePreview(_availabilityMapTexture, w, h);
        }
    }

    if (ImGui::CollapsingHeader("Path finding"))
    {
        ImGui::DragFloat2(
            "Start conf",
            glm::value_ptr(_startConfiguration),
            0.02f
        );

        auto solutions = getValidSolutions();
        if (solutions.size() > 0)
        {
            if (ImGui::Button("Store current##start"))
            {
                _startConfiguration = glm::vec2{
                    solutions[0].first,
                    solutions[0].second
                };
            }
        }

        ImGui::DragFloat2(
            "End conf",
            glm::value_ptr(_endConfiguration),
            0.02f
        );

        if (solutions.size() > 0)
        {
            if (ImGui::Button("Store current##end"))
            {
                _endConfiguration = glm::vec2{
                    solutions[0].first,
                    solutions[0].second
                };
            }
        }

        if (_availabilityMapCreated)
        {
            if (ImGui::Button("Find path"))
            {
                findPath();
            }
        }
        else
        {
            ImGui::TextColored(
                {1.0f, 0.0f, 0.0f, 1.0f},
                "Path cannot be found without configuration space generated."
            );
        }

        if (_searchMapAvailable)
        {
            int w, h;
            int miplevel = 0;

            glBindTexture(GL_TEXTURE_2D, _searchMapTexture);

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

            showTexturePreview(_searchMapTexture, w, h);
        }
    }

    if (_animationEnabled)
    {
        _frameAnimationPassed +=
            std::chrono::duration<float>(deltaTime).count();

        while (_frameAnimationPassed > _frameTime)
        {
            ++_currentAnimationStep;
            _frameAnimationPassed -= _frameTime;
        }

        if (_currentAnimationStep + 1 >= _configurationPath.size())
        {
            _animationEnabled = false;
            _currentAnimationStep = 0;
            _frameAnimationPassed = 0.0f;
        }
    }

    if (ImGui::CollapsingHeader("Animation"))
    {
        ImGui::SliderFloat("Time per frame", &_frameTime, 0.05f, 2.0f);

        if (_currentAnimationStep > 0 && ImGui::Button("Restart"))
        {
            _currentAnimationStep = 0;
            _frameAnimationPassed = 0.0f;
        }

        if (!_animationEnabled && ImGui::Button("Play"))
        {
            if (_currentAnimationStep >= _configurationPath.size())
                _currentAnimationStep = 0;
            _animationEnabled = true;
        }
        else if (_animationEnabled && ImGui::Button("Stop"))
        {
            _animationEnabled = false;
        }
    }
}

void KinematicChainApplication::onRender()
{
    glClearColor(0.4f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    auto projection = getProjection();

    glm::vec3 primaryColor{0.0f, 1.0f, 1.0f};
    glm::vec3 secondaryColor{0.3f, 0.3f, 0.3f};


    auto solutions = getValidSolutions();

    if (solutions.size() > 1)
    {
        _standard2DEffect->setEmissionColor(secondaryColor);
    }
    else
    {
        _standard2DEffect->setEmissionColor(primaryColor);
    }


    for (auto it = solutions.rbegin(); it != solutions.rend(); ++it)
    {
        const auto& solution = *it;

        _armRendering->setFirstArmLength(_armController->getFirstArmLength());
        _armRendering->setSecondArmLength(_armController->getSecondArmLength());
        _armRendering->setArmsThickness(_armController->getVisualThickness());
        _armRendering->setAlphaAngle(solution.first);
        _armRendering->setBetaAngle(solution.second);

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

        _standard2DEffect->setEmissionColor(primaryColor);
    }

    for (const auto& constraint: _constraints)
    {
        glm::vec2 position = (constraint.min + constraint.max) / 2.0f;
        glm::vec2 size = constraint.max - constraint.min;
        drawQuad(position, size, {1.0f, 0.3f, 0.3f});
    }

    drawQuad(_armController->getTarget(), {0.01, 0.01}, {0.0f, 1.0f, 0.0f});

    if (_line != nullptr)
    {
        _standard2DEffect->setEmissionColor({0.3f, 1.0f, 1.0f});
        _standard2DEffect->setModelMatrix({});
        _standard2DEffect->setViewMatrix({});
        _standard2DEffect->setProjectionMatrix(projection);
        _standard2DEffect->begin();
        _line->render();
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
            if (!grabConstraint())
            {
                _selectedConstraint = -1;

                glm::vec2 worldPos = getWorldCursorPos(
                    getCurrentMousePosition()
                );

                _armController->setTarget(worldPos);
                _armController->solveInverseKinematics();
            }
        }
        else
        {
            _isConstraintGrabbed = false;
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

bool KinematicChainApplication::grabConstraint()
{
    auto worldCursorPos = getWorldCursorPos(getCurrentMousePosition());

    for (auto i = 0; i < _constraints.size(); ++i)
    {
        if (_constraints[i].contains(worldCursorPos))
        {
            _selectedConstraint = i;
            _isConstraintGrabbed = true;
            _previousGrabWorldPosition = worldCursorPos;
            return true;
        }
    }

    return false;
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

std::vector<std::pair<float, float>>
    KinematicChainApplication::getValidSolutions()
{
    if (_animationEnabled)
    {
        auto from = glm::vec2{_configurationPath[_currentAnimationStep]};
        auto to = glm::vec2{_configurationPath[_currentAnimationStep + 1]};

        float t = std::min(1.0f, _frameAnimationPassed / _frameTime);
        auto mixed = glm::vec2{
            mixDegrees(from.x, to.x, t),
            mixDegrees(from.y, to.y, t),
        };

        mixed = glm::radians(mixed);
        return {{mixed.x, mixed.y}};
    }

    std::vector<std::pair<float, float>> output;
    for (const auto& solution: _armController->getSolutions())
    {
        if (checkConfiguration(solution.first, solution.second))
        {
            output.push_back(solution);
        }
    }
    return output;
}

float KinematicChainApplication::mixDegrees(float a, float b, float m)
{
    auto lengthDirect = std::abs(b - a);
    auto lengthAround = std::min(a, b) + (360.0f - std::max(a, b));

    if (lengthDirect < lengthAround)
    {
        return glm::mix(a, b, m);
    }

    if (a < b)
    {
        return glm::mix(a, b - 360.0f, m);
    }
    else
    {
        return glm::mix(a, b + 360.0f, m);
    }
}

bool KinematicChainApplication::checkConfiguration(float alpha, float beta)
{
    auto config = _armController->buildConfiguration(alpha, beta);
    return !(checkArmConstraintCollision({0, 0}, config.first)
        || checkArmConstraintCollision(config.first, config.second));
}

void KinematicChainApplication::drawQuad(
    const glm::vec2& position,
    const glm::vec2& size,
    const glm::vec3& color
)
{
    _standard2DEffect->setEmissionColor(color);
    _standard2DEffect->setViewMatrix({});
    _standard2DEffect->setModelMatrix(glm::scale(
        glm::translate(
            glm::mat4{},
            glm::vec3{position, 0.0}
        ),
        glm::vec3{size, 1.0}
    ));
    _standard2DEffect->setProjectionMatrix(getProjection());
    _standard2DEffect->setDiffuseTexture(_testTexture->getTextureId());
    _standard2DEffect->begin();
    _quadGeometry->render();
    _standard2DEffect->end();
}

void KinematicChainApplication::showTexturePreview(
    GLuint texture,
    int w,
    int h
)
{
    auto availX = ImGui::GetContentRegionAvailWidth();
    float scale = std::min(1.0f, availX / w);
    void* imTex = (void*)texture;
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
        float focus_x =
            ImGui::GetMousePos().x - tex_screen_pos.x - focus_sz * 0.5f;
        float focus_y =
            ImGui::GetMousePos().y - tex_screen_pos.y - focus_sz * 0.5f;

        if (focus_x < 0.0f)
        {
            focus_x = 0.0f;
        }
        else if (focus_x > w - focus_sz)
        {
            focus_x = w - focus_sz;
        }

        if (focus_y < 0.0f)
        {
            focus_y = 0.0f;
        }
        else if (focus_y > h - focus_sz)
        {
            focus_y = h - focus_sz;
        }

        ImGui::Text("Min: alpha=%.2f, beta=%.2f", focus_y, focus_x);
        ImGui::Text(
            "Max: alpha=%.2f, beta=%.2f",
            focus_y + focus_sz,
            focus_x + focus_sz
        );

        ImVec2 uv0{(focus_x) / w, (focus_y) / h};
        ImVec2 uv1{(focus_x + focus_sz) / w, (focus_y + focus_sz) / h};
        ImGui::Image(
            imTex,
            ImVec2(128,128),
            uv0,
            uv1,
            ImColor(255,255,255,255),
            ImColor(255,255,255,128)
        );

        ImGui::EndTooltip();
    }
}

glm::ivec2 KinematicChainApplication::getClosestInConfiguration(
    glm::vec2 coord
)
{
    auto degrees = glm::degrees(coord);
    glm::vec2 clamped{
        static_cast<int>(std::round(degrees.x)) % 360,
        static_cast<int>(std::round(degrees.y)) % 360
    };

    if (clamped.x < 0) { clamped.x += 360; }
    if (clamped.y < 0) { clamped.y += 360; }

    return clamped;
}

void KinematicChainApplication::findPath()
{
    auto startDeg = getClosestInConfiguration(_startConfiguration);
    auto endDeg = getClosestInConfiguration(_endConfiguration);

    const int cSearchMapMax = 360 * 360 + 1;
    _searchMap.resize(360*360);
    std::fill(std::begin(_searchMap), std::end(_searchMap), cSearchMapMax);

    _searchMapTraceback.resize(360*360);
    for (auto& el: _searchMapTraceback)
    {
        el = glm::ivec2{-1, -1};
    }

    std::queue<glm::ivec2> coordQueue;
    coordQueue.push(startDeg);
    markSearchMap(startDeg, 0);

    const int dirx[] = {-1, 0, +1, 0};
    const int diry[] = {0, -1, 0, +1};

    bool found = false;
    int maxDist = 0;
    while (!coordQueue.empty())
    {
        auto current = coordQueue.front();
        coordQueue.pop();

        if (current == endDeg)
        {
            found = true;
            break;
        }

        int nextDist = getSearchMapValue(current) + 1;
        maxDist = std::max(maxDist, nextDist);

        for (auto i = 0; i < 4; ++i)
        {
            glm::ivec2 next{
                (current.x + dirx[i]) % 360,
                (current.y + diry[i]) % 360
            };

            if (next.x < 0) { next.x += 360; }
            if (next.y < 0) { next.y += 360; }

            if (!verifyAvailability(next)) { continue; }

            auto nextCurrentValue = getSearchMapValue(next);

            if (nextCurrentValue > nextDist)
            {
                auto index = 360 * next.x + next.y;
                _searchMapTraceback[index] = current;
                markSearchMap(next, nextDist);
                coordQueue.push(next);
            }
        }
    }

    _configurationPath.clear();

    if (found)
    {
        trackbackAndStorePath(endDeg);
        _animationEnabled = false;
        _currentAnimationStep = 0;
        _frameAnimationPassed = 0.0f;
    }

    for (auto &pathStep: _configurationPath)
    {
        markSearchMap(pathStep, -2);
    }

    markSearchMap(startDeg, -1);
    markSearchMap(endDeg, -1);

    std::vector<unsigned char> image;
    for (const auto& state: _searchMap)
    {
        if (state == cSearchMapMax)
        {
            image.push_back(0);
            image.push_back(0);
            image.push_back(0);
            continue;
        }

        if (state == -1)
        {
            image.push_back(255);
            image.push_back(0);
            image.push_back(255);
            continue;
        }

        if (state == -2)
        {
            image.push_back(255);
            image.push_back(255);
            image.push_back(255);
            continue;
        }

        auto gradient = std::min(1.0f, static_cast<float>(state) / maxDist);
        int hue = static_cast<int>(260 * gradient);
        auto color = glm::rgbColor(glm::vec3{hue, 1.0f, 1.0f});

        image.push_back(
            static_cast<unsigned char>(std::min(255.0f, 255 * color.x))
        );

        image.push_back(
            static_cast<unsigned char>(std::min(255.0f, 255 * color.y))
        );

        image.push_back(
            static_cast<unsigned char>(std::min(255.0f, 255 * color.z))
        );
    }

    _searchMapAvailable = true;
    glGenTextures(1, &_searchMapTexture);
    glBindTexture(GL_TEXTURE_2D, _searchMapTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 360, 360, 0,
        GL_RGB, GL_UNSIGNED_BYTE, image.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

void KinematicChainApplication::markSearchMap(glm::ivec2 coord, int value)
{
    auto index = 360 * coord.x + coord.y;
    _searchMap[index] = value;
}

int KinematicChainApplication::getSearchMapValue(glm::ivec2 coord)
{
    auto index = 360 * coord.x + coord.y;
    return _searchMap[index];
}

bool KinematicChainApplication::verifyAvailability(glm::ivec2 coord)
{
    auto index = 360 * coord.x + coord.y;
    return _availabilityMap[index];
}

void KinematicChainApplication::trackbackAndStorePath(glm::ivec2 end)
{
    std::vector<glm::ivec2> newPath;
    auto current = end;
    while (current != glm::ivec2{-1, -1})
    {
        newPath.push_back(current);
        auto index = 360 * current.x + current.y;
        current = _searchMapTraceback[index];
    }

    _configurationPath.clear();
    std::reverse_copy(
        std::begin(newPath),
        std::end(newPath),
        std::back_inserter(_configurationPath)
    );

    updatePolygonalLine();
}

void KinematicChainApplication::updatePolygonalLine()
{
    std::vector<fw::VertexColor> vertices;
    for (const auto& pos: _configurationPath)
    {
        auto config = _armController->buildConfiguration(
            glm::radians(static_cast<float>(pos.x)),
            glm::radians(static_cast<float>(pos.y))
        );

        vertices.push_back({
            {config.second, 0.0f},
            {1.0f, 1.0f, 1.0f}
        });
    }

    _line = std::make_shared<fw::PolygonalLine>(vertices);
}

}
