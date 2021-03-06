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
#include "fw/PolygonalLine.hpp"
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

    std::vector<std::pair<float, float>> getValidSolutions();

private:
    void drawQuad(
        const glm::vec2& position,
        const glm::vec2& size,
        const glm::vec3& color
    );

    void showTexturePreview(GLuint texture, int w, int h);

    glm::ivec2 getClosestInConfiguration(glm::vec2 coord);
    void markSearchMap(glm::ivec2 coord, int value);
    int getSearchMapValue(glm::ivec2 coord);
    bool verifyAvailability(glm::ivec2 coord);
    void findPath();
    void trackbackAndStorePath(glm::ivec2 end);
    void updatePolygonalLine();

    float mixDegrees(float a, float b, float m);

    std::shared_ptr<fw::PolygonalLine> _line;

    std::shared_ptr<fw::Standard2DEffect> _standard2DEffect;
    std::shared_ptr<fw::Mesh<fw::StandardVertex2D>> _quadGeometry;
    std::shared_ptr<fw::Texture> _testTexture;

    std::shared_ptr<RoboticArmController> _armController;
    std::shared_ptr<RoboticArmRendering> _armRendering;

    GLuint _texturePreview;

    bool _animationEnabled;
    float _frameAnimationPassed;
    float _frameTime;
    int _currentAnimationStep;

    bool _availabilityMapCreated;
    GLuint _availabilityMapTexture;

    bool _searchMapAvailable;
    GLuint _searchMapTexture;
    std::vector<glm::ivec2> _configurationPath;

    bool _isConstraintGrabbed;
    glm::vec2 _previousGrabWorldPosition;

    glm::vec2 _startConfiguration;
    glm::vec2 _endConfiguration;

    int _selectedConstraint;
    std::vector<fw::AABB<glm::vec2>> _constraints;

    std::vector<bool> _availabilityMap;
    std::vector<int> _searchMap;
    std::vector<glm::ivec2> _searchMapTraceback;
};

}
