#pragma once

#include <memory>

#include "glm/glm.hpp"

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

private:
    std::shared_ptr<fw::Standard2DEffect> _standard2DEffect;
    std::shared_ptr<fw::Mesh<fw::StandardVertex2D>> _quadGeometry;
    std::shared_ptr<fw::Texture> _testTexture;
};

}
