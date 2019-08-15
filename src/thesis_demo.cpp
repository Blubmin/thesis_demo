#define _USE_MATH_DEFINES
#include <cmath>

#include "aesthetic_camera_component.h"
#include "circular_movement_component.h"

#include <GL/gl3w.h>
#include <glog/logging.h>
#include <imgui/imgui.h>
#include <pixel_engine/camera.h>
#include <pixel_engine/empty.h>
#include <pixel_engine/free_camera_component.h>
#include <pixel_engine/game.h>
#include <pixel_engine/mesh_loader.h>
#include <pixel_engine/ogl_framebuffer.h>
#include <pixel_engine/ogl_fxaa_renderer.h>
#include <pixel_engine/ogl_gamma_renderer.h>
#include <pixel_engine/ogl_mesh.h>
#include <pixel_engine/ogl_texture_renderer.h>
#include <pixel_engine/point_light.h>
#include <pixel_engine/program.h>
#include <pixel_engine/scene.h>
#include <pixel_engine/scene_renderer.h>
#include <Eigen/Geometry>
#include <boost/format.hpp>

namespace {
boost::filesystem::path GetResourcePath() {
  return boost::filesystem::path(__FILE__).parent_path() / "resources";
}
boost::filesystem::path GetMeshPath(const std::string& mesh_file) {
  return GetResourcePath() / "meshes" / mesh_file;
}
boost::filesystem::path GetShaderPath(const std::string& shader_file) {
  return GetResourcePath() / "shaders" / shader_file;
}
}  // namespace

class HelloGame : public pxl::Game {
 public:
  HelloGame() : pxl::Game("Hello Game") {}
  void Init() override {
    empty = std::make_shared<pxl::Empty>();
    empty->AddComponent(std::make_shared<CircularMovementComponent>());

    mesh = pxl::MeshLoader::LoadMesh<pxl::OglMesh>(GetMeshPath("bunny.obj"));
    mesh->position = Eigen::Vector3f(-5.0, 0, 0);
    mesh->rotation.y() = 90;
    mesh->Bind();
    empty->AddChild(mesh);

    ground = pxl::MeshLoader::LoadMesh<pxl::OglMesh>(GetMeshPath("plane.obj"));
    ground->Bind();
    ground->position -= Eigen::Vector3f(0.f, 0.01f, 0.f);
    ground->scale = Eigen::Vector3f(10.f, 10.f, 10.f);
    ground->rotation.x() = -90;

    prog = std::shared_ptr<pxl::Program>(new pxl::Program(
        GetShaderPath("mesh.vert"), GetShaderPath("mesh.frag")));

    camera = std::make_shared<pxl::Camera>();
    camera->position += Eigen::Vector3f(0, 1, 10);
    camera->AddComponent(std::make_shared<pxl::FreeCameraComponent>());

    aesthetic_camera = std::make_shared<pxl::Camera>();
    aesthetic_camera->fov = 45;
    aesthetic_camera->position = camera->position;
    auto aesthetic_camera_component =
        std::make_shared<AestheticCameraComponent>();
    aesthetic_camera_component->SetTarget(mesh);
    aesthetic_camera->AddComponent(aesthetic_camera_component);

    framebuffers =
        std::make_pair(std::make_shared<pxl::OglFramebuffer>(1920, 1080),
                       std::make_shared<pxl::OglFramebuffer>(1920, 1080));
    framebuffers.first->Bind();
    framebuffers.second->Bind();
    viewport_framebuffer = std::make_shared<pxl::OglFramebuffer>(1920, 1080);
    viewport_framebuffer->Bind();

    point_lights.emplace_back(new pxl::PointLight(pxl::Color(1.f, .0f, .0f)));
    point_lights.back()->position += Eigen::Vector3f(4.f, 2.f, 0.f);
    point_lights.emplace_back(new pxl::PointLight(pxl::Color(.0f, .0f, 1.f)));
    point_lights.back()->position += Eigen::Vector3f(-4.f, 2.f, 0.f);
    point_lights.emplace_back(new pxl::PointLight(pxl::Color(.0f, 1.f, .0f)));
    point_lights.back()->position += Eigen::Vector3f(-8.f, 2.f, 0.f);
    point_lights.emplace_back(new pxl::PointLight(pxl::Color(.5f, .0f, 1.f)));
    point_lights.back()->position += Eigen::Vector3f(8.f, 2.f, 0.f);

    auto light = std::make_shared<pxl::PointLight>(pxl::Color(1.f, 1.f, 1.f));
    light->position += Eigen::Vector3f(1.f, 1.f, 1.f);

    auto light2 = std::make_shared<pxl::PointLight>(pxl::Color(1.f, .5f, 0.f));
    light2->position += Eigen::Vector3f(1.f, 1.f, 1.f);
    light->AddChild(light2);

    scene = std::make_shared<pxl::Scene>();
    scene->camera = camera;
    scene->entities.push_back(empty);
    scene->entities.push_back(camera);
    scene->entities.push_back(mesh);
    scene->entities.push_back(ground);
    scene->entities.push_back(light);
    scene->entities.push_back(light2);
    // scene->entities.push_back(
    //    std::make_shared<pxl::PointLight>(pxl::Color(1.f, 1.f, 1.f)));
    // scene->entities.back()->position += Eigen::Vector3f(0.f, 3.f, 0.f);
    scene->entities.insert(scene->entities.end(), point_lights.begin(),
                           point_lights.end());
    scene->Bind();
  }

  void Update(float time_elapsed) override {
    static float gamma = 2.2f;
    scene->Update(time_elapsed);
    framebuffers.first->Start();
    // prog->Bind();

    if (ImGui::Begin("Property")) {
      ImGui::DragFloat3("Position", mesh->position.data());
      ImGui::DragFloat3("Rotation", mesh->rotation.data());
      ImGui::DragFloat3("Scale", mesh->scale.data());
      ImGui::DragFloat("Gamma", &gamma, .01f);
      ImGui::End();
    }

    // Draw scene from primary camera
    scene->camera = camera;
    pxl::SceneRenderer::RenderScene(*scene);
    framebuffers.first->End();

    framebuffers.second->Start();
    pxl::OglGammaRenderer::GetInstance()->RenderTexture(
        framebuffers.first->GetColorAttachment(0), gamma);
    framebuffers.second->End();

    pxl::OglFxaaRenderer::GetInstance()->RenderTexture(
        *framebuffers.second->GetColorAttachment(0));

    // Draw scene from aesthetic camera
    framebuffers.first->Start();
    aesthetic_camera->Update(time_elapsed);
    scene->camera = aesthetic_camera;
    pxl::SceneRenderer::RenderScene(*scene);
    framebuffers.first->End();

    viewport_framebuffer->Start();
    pxl::OglGammaRenderer::GetInstance()->RenderTexture(
        framebuffers.first->GetColorAttachment(0), gamma);
    viewport_framebuffer->End();

    framebuffers.first->Start();
    pxl::OglFxaaRenderer::GetInstance()->RenderTexture(
        *viewport_framebuffer->GetColorAttachment(0));
    framebuffers.first->End();

    pxl::OglTextureRenderer::GetInstance()->RenderTexture(
        *framebuffers.first->GetColorAttachment(0),
        Eigen::Rectf(Eigen::Vector2f(.73, .035), Eigen::Vector2f(.98, .285)));
  }

  std::pair<std::shared_ptr<pxl::OglFramebuffer>,
            std::shared_ptr<pxl::OglFramebuffer>>
      framebuffers;
  std::shared_ptr<pxl::OglFramebuffer> viewport_framebuffer;
  std::shared_ptr<pxl::Camera> camera;
  std::shared_ptr<pxl::Camera> aesthetic_camera;
  std::shared_ptr<pxl::Scene> scene;
  std::shared_ptr<pxl::Empty> empty;
  std::shared_ptr<pxl::OglMesh> mesh;
  std::shared_ptr<pxl::OglMesh> ground;
  std::shared_ptr<pxl::Program> prog;
  std::vector<std::shared_ptr<pxl::PointLight>> point_lights;
};

int main(int argc, char* argv[]) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  HelloGame game;
  game.Run();
  return 0;
}
