#pragma once

#include <memory>

#include "glm/glm.hpp"

#include "fw/AABB.hpp"
#include "fw/FrameMarker.hpp"
#include "fw/Grid.hpp"
#include "fw/ImGuiApplication.hpp"
#include "fw/Mesh.hpp"
#include "fw/OrbitingCamera.hpp"
#include "fw/Texture.hpp"
#include "fw/TexturedPhongEffect.hpp"
#include "fw/UniversalPhongEffect.hpp"
#include "fw/Vertices.hpp"
#include "fw/effects/Standard2DEffect.hpp"

#include "RoboticArmController.hpp"
#include "RoboticArmRendering.hpp"

namespace kinematic
{

class KinematicChainApplication:
    public fw::ImGuiApplication
{
public:
    KinematicChainApplication();
    virtual ~KinematicChainApplication();

protected:
    virtual void onCreate() override;
    virtual void onDestroy() override;
    virtual void onUpdate(
        const std::chrono::high_resolution_clock::duration& deltaTime
    ) override;
    virtual void onRender() override;

    virtual bool onMouseButton(int button, int action, int mods) override;
    virtual bool onMouseMove(glm::dvec2 newPosition) override;
    virtual bool onScroll(double xoffset, double yoffset) override;
    virtual bool onResize() override;

    glm::vec2 getWorldCursorPos(glm::vec2 screenMousePos) const;
    glm::mat4 getProjection() const;
    bool grabConstraint();

    bool checkArmConstraintCollision(glm::vec2 start, glm::vec2 end) const;

    bool checkSegmentAABBCollision(
        const glm::vec2& start,
        const glm::vec2& end,
        const fw::AABB<glm::vec2>& aabb
    ) const;

    void createAvailabilityMap();
    void createAvailabilityMapTexture();
    bool checkConfiguration(float alpha, float beta);

private:
    void drawQuad(
        const glm::vec2& position,
        const glm::vec2& size,
        const glm::vec3& color
    );

    void showTexturePreview(int w, int h);

    std::shared_ptr<fw::Standard2DEffect> _standard2DEffect;
    std::shared_ptr<fw::Mesh<fw::StandardVertex2D>> _quadGeometry;
    std::shared_ptr<fw::Texture> _testTexture;

    std::shared_ptr<RoboticArmController> _armController;
    std::shared_ptr<RoboticArmRendering> _armRendering;

    GLuint _texturePreview;

    bool _availabilityMapCreated;
    GLuint _availabilityMapTexture;

    bool _isConstraintGrabbed;
    glm::vec2 _previousGrabWorldPosition;

    int _selectedConstraint;
    std::vector<fw::AABB<glm::vec2>> _constraints;

    std::vector<bool> _availabilityMap;
};

}
