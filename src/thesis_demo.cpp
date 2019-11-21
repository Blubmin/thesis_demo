#define _USE_MATH_DEFINES
#include <cmath>

#include "aesthetic_camera_component.h"
#include "ai_manager.h"
#include "circular_movement_component.h"
#include "planar_movement_component.h"
#include "shooting_component.h"

#include <GL/gl3w.h>
#include <glog/logging.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <pixel_engine/camera.h>
#include <pixel_engine/collider_component.h>
#include <pixel_engine/convex_hull.h>
#include <pixel_engine/directional_light.h>
#include <pixel_engine/empty.h>
#include <pixel_engine/fps_controller.h>
#include <pixel_engine/free_camera_component.h>
#include <pixel_engine/game.h>
#include <pixel_engine/mesh_loader.h>
#include <pixel_engine/ogl_framebuffer.h>
#include <pixel_engine/ogl_fxaa_renderer.h>
#include <pixel_engine/ogl_gamma_renderer.h>
#include <pixel_engine/ogl_mesh.h>
#include <pixel_engine/ogl_texture_renderer.h>
#include <pixel_engine/physics_component.h>
#include <pixel_engine/point_light.h>
#include <pixel_engine/program.h>
#include <pixel_engine/scene.h>
#include <pixel_engine/scene_renderer.h>
#include <pixel_engine/skybox.h>
#include <pixel_engine/sphere.h>
#include <Eigen/Geometry>
#include <boost/format.hpp>

namespace {
boost::filesystem::path GetResourcePath() {
  return boost::filesystem::path(__FILE__).parent_path() / "resources";
}
boost::filesystem::path GetMeshPath(const std::string& mesh_file) {
  return GetResourcePath() / "meshes" / mesh_file;
}
boost::filesystem::path GetImagePath(const std::string& mesh_file) {
  return GetResourcePath() / "images" / mesh_file;
}
boost::filesystem::path GetShaderPath(const std::string& shader_file) {
  return GetResourcePath() / "shaders" / shader_file;
}
}  // namespace

class ThesisDemo : public pxl::Game {
 public:
  ThesisDemo() : pxl::Game("Aesthetic Cam Demo") {}
  void Init() override {
    glfwSetInputMode(pxl::Game::State.window, GLFW_CURSOR,
                     GLFW_CURSOR_DISABLED);

    paused = true;
    main_camera = true;

    reticle_tex =
        std::make_shared<pxl::OglTexture2d>(GetImagePath("reticle.png"));
    reticle_tex->Bind();

    empty = std::make_shared<pxl::Empty>();
    empty->AddComponent(std::make_shared<CircularMovementComponent>());

    mesh =
        pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(GetMeshPath("bunny.obj"));
    mesh->position = Eigen::Vector3f(-5.0, 0, 0);
    mesh->rotation.y() = 90;
    mesh->Bind();
    empty->AddChild(mesh);

    ground = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("wood/wood.obj"));
    ground->Bind();
    ground->position -= Eigen::Vector3f(0.f, 0.01f, 0.f);
    // ground->scale = Eigen::Vector3f(50.f, 50.f, 1.f);
    // ground->rotation.x() = -90;

    prog = std::shared_ptr<pxl::Program>(new pxl::Program(
        GetShaderPath("mesh.vert"), GetShaderPath("mesh.frag")));

    camera = std::make_shared<pxl::Camera>();
    camera->position += Eigen::Vector3f(0, 1, 10);

    aesthetic_camera = std::make_shared<pxl::Camera>();
    aesthetic_camera->fov = 45;
    aesthetic_camera->position = camera->position;
    auto aesthetic_camera_component =
        std::make_shared<AestheticCameraComponent>();
    aesthetic_camera_component->SetPlayer(mesh);
    aesthetic_camera->AddComponent(aesthetic_camera_component);

    framebuffers.push_back(std::make_shared<pxl::OglFramebuffer>(1920, 1080));
    framebuffers.push_back(std::make_shared<pxl::OglFramebuffer>(1920, 1080));
    framebuffers.push_back(std::make_shared<pxl::OglFramebuffer>(1920, 1080));
    framebuffers.push_back(std::make_shared<pxl::OglFramebuffer>(1920, 1080));
    for (auto fb : framebuffers) {
      fb->Bind();
    }
    viewport_framebuffer = std::make_shared<pxl::OglFramebuffer>(1920, 1080);
    viewport_framebuffer->Bind();

    // auto light = std::make_shared<pxl::PointLight>(pxl::Color(1.f,
    // 1.f, 1.f)); light->position += Eigen::Vector3f(1.f, 1.f, 1.f);

    // auto light2 = std::make_shared<pxl::PointLight>(pxl::Color(1.f, .5f,
    // 0.f)); light2->position += Eigen::Vector3f(1.f, 1.f, 1.f);
    // light->AddChild(light2);

    auto dir_light =
        std::make_shared<pxl::DirectionalLight>(Eigen::Vector3f(1, -1, 1));

    player = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("army_man_standing_scaled.obj"));
    player->AddChild(camera);
    player->AddComponent(std::make_shared<pxl::CapsuleCollider>(
        .35f, 1.8f, pxl::ColliderComponent::kDynamic));
    player->AddComponent(std::make_shared<pxl::PhysicsComponent>());

    camera->position = Eigen::Vector3f(0, 1.65, .2);
    camera->rotation.y() = 180;

    // std::shared_ptr<pxl::Mesh> sphere = std::make_shared<pxl::OglMesh>(
    //    std::make_shared<pxl::UvSphere>(.5f, 10, 20));
    player->Bind();
    player->position = Eigen::Vector3f(3, 0, 2);
    player->AddComponent(std::make_shared<pxl::FpsController>());
    aesthetic_camera_component->SetTarget(player);

    block = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("block_A/block_A.obj"));
    // block = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
    //    GetMeshPath("urchin.obj"));
    block->Bind();
    block->position = Eigen::Vector3f(-5, 1, -5);
    block->rotation.y() = 45;
    block->AddComponent(std::make_shared<pxl::BoxCollider>(
        Eigen::Vector3f(1, 1, 1), pxl::ColliderComponent::kStatic));

    auto block_H = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("block_A/block_H.obj"));
    block_H->Bind();
    block_H->position = Eigen::Vector3f(-5, 3, -5);
    block_H->rotation.z() = 90;
    block_H->AddComponent(std::make_shared<pxl::BoxCollider>(
        Eigen::Vector3f(1, 1, 1), pxl::ColliderComponent::kStatic));

    auto ramp =
        pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(GetMeshPath("ramp.obj"));
    ramp->Bind();
    ramp->position = Eigen::Vector3f(5, 1, -5);
    ramp->rotation.y() = -45;
    ramp->AddComponent(std::make_shared<pxl::HullCollider>(
        *ramp->mesh, pxl::ColliderComponent::kStatic));

    auto slight_ramp = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("slight_ramp.obj"));
    slight_ramp->Bind();
    slight_ramp->position = Eigen::Vector3f(8, .5, -5);
    slight_ramp->AddComponent(std::make_shared<pxl::HullCollider>(
        *slight_ramp->mesh, pxl::ColliderComponent::kStatic));

    auto cards = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("playing_cards.obj"));
    cards->Bind();
    cards->position = Eigen::Vector3f(5, 0, -8);
    cards->AddComponent(std::make_shared<pxl::CombinedHullCollider>(
        *cards->mesh, pxl::ColliderComponent::kStatic));

    auto wall1 = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("block_wall_big.obj"));
    wall1->Bind();
    wall1->position = Eigen::Vector3f(0, 0, -27);
    wall1->AddComponent(std::make_shared<pxl::CombinedHullCollider>(
        *wall1->mesh, pxl::ColliderComponent::kStatic));

    auto wall2 = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("block_wall_big.obj"));
    wall2->Bind();
    wall2->position = Eigen::Vector3f(0, 0, 27);
    wall2->rotation = Eigen::Vector3f(0, 180, 0);
    wall2->AddComponent(std::make_shared<pxl::CombinedHullCollider>(
        *wall2->mesh, pxl::ColliderComponent::kStatic));

    auto wall3 = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("block_wall_big.obj"));
    wall3->Bind();
    wall3->position = Eigen::Vector3f(25, 0, 0);
    wall3->rotation = Eigen::Vector3f(0, -90, 0);
    wall3->AddComponent(std::make_shared<pxl::CombinedHullCollider>(
        *wall3->mesh, pxl::ColliderComponent::kStatic));

    auto wall4 = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("block_wall_big.obj"));
    wall4->Bind();
    wall4->position = Eigen::Vector3f(-25, 0, 0);
    wall4->rotation = Eigen::Vector3f(0, 90, 0);
    wall4->AddComponent(std::make_shared<pxl::CombinedHullCollider>(
        *wall4->mesh, pxl::ColliderComponent::kStatic));

    auto tree = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("tree_standee.obj"));
    tree->Bind();
    tree->position = Eigen::Vector3f(-6, 0, 6);
    tree->rotation = Eigen::Vector3f(0, 150, 0);
    tree->AddComponent(std::make_shared<pxl::HullCollider>(
        *tree->mesh, pxl::ColliderComponent::kStatic));

    auto car = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("toon_car.obj"));
    car->Bind();
    car->position = Eigen::Vector3f(6.2, 0, 1);
    car->rotation.y() = -25;
    car->AddComponent(std::make_shared<pxl::CombinedHullCollider>(
        *car->mesh, pxl::ColliderComponent::kStatic));

    auto books = pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
        GetMeshPath("book_track.obj"));
    books->Bind();
    books->position = Eigen::Vector3f(10, 0, -5);
    books->rotation.y() = 12;
    books->AddComponent(std::make_shared<pxl::HullCollider>(
        *books->mesh, pxl::ColliderComponent::kStatic));

    behind_camera = std::make_shared<pxl::Camera>();
    behind_camera->position = Eigen::Vector3f(0, 2.5, -2);
    behind_camera->rotation.y() = 180;
    behind_camera->rotation.x() = -19;
    player->AddChild(behind_camera);

    overhead_camera = std::make_shared<pxl::Camera>();
    overhead_camera->position = Eigen::Vector3f(0, 10, 0);
    overhead_camera->fov = 100;
    overhead_camera->rotation.y() = 180;
    overhead_camera->rotation.x() = -90;
    player->AddChild(overhead_camera);

    scene = std::make_shared<pxl::Scene>();
    scene->camera = camera;
    scene->AddEntity(wall1);
    scene->AddEntity(wall2);
    scene->AddEntity(wall3);
    scene->AddEntity(wall4);
    scene->AddEntity(tree);
    // scene->AddEntity(ramp);
    scene->AddEntity(car);
    // scene->AddEntity(slight_ramp);
    scene->AddEntity(player);
    scene->AddEntity(empty);
    scene->AddEntity(books);
    // scene->AddEntity(camera);
    // scene->AddEntity(aesthetic_camera);
    // scene->AddEntity(mesh);
    scene->AddEntity(ground);
    // scene->AddEntity(light);
    // scene->AddEntity(light2);
    scene->AddEntity(dir_light);
    scene->AddEntity(block);
    scene->AddEntity(block_H);
    scene->AddEntity(cards);

    auto skybox_mesh = pxl::MeshLoader::LoadMesh<pxl::OglMesh>(
        GetMeshPath("skybox/skybox.obj"));
    skybox_mesh->Bind();
    scene->skybox = std::make_shared<pxl::Skybox>(skybox_mesh);
    scene->Bind();

    ai_manager = std::make_shared<AiManager>(scene, player);
    aesthetic_camera_component->SetPlayer(player);
    aesthetic_camera_component->SetTarget(ai_manager->red_leader_);
    pxl::Game::BackgroundThreadPool.Post(
        aesthetic_camera_component->RunSolver());

    player->AddComponent(std::make_shared<ShootingComponent>(scene));
  }

  void Update(float time_elapsed) override {
    static float gamma = 2.2f;
    if (!paused) {
      ai_manager->Update(time_elapsed);
      scene->Update(time_elapsed);
    }
    framebuffers[0]->Begin();
    // prog->Bind();
    if (ImGui::Begin("Property")) {
      ImGui::DragFloat3("Position", mesh->position.data());
      ImGui::DragFloat3("Rotation", mesh->rotation.data());
      ImGui::DragFloat3("Scale", mesh->scale.data());
      ImGui::DragFloat("Gamma", &gamma, .01f);

      ImGui::Separator();
      auto dir_light = scene->GetEntity<pxl::DirectionalLight>();
      ImGui::TextUnformatted("Directional Light");
      ImGui::DragFloat3("Direction", dir_light->direction.data());
      ImGui::DragFloat("Strength", &dir_light->strength, .01f, 0.f, 100.f);
      ImGui::ColorEdit3("Color", dir_light->color.data());
      ImGui::Separator();

      // Draw checkboxes to disable residuals
      auto aesthetic_camera_component =
          aesthetic_camera->GetComponent<AestheticCameraComponent>();
      if (aesthetic_camera_component != nullptr) {
        for (int i = 0;
             i < aesthetic_camera_component->constant_residuals.size(); ++i) {
          boost::format label("Residual %d");
          label % i;
          bool val = aesthetic_camera_component->constant_residuals[i];
          if (ImGui::Checkbox(label.str().c_str(), &val)) {
            aesthetic_camera_component->constant_residuals[i] = val;
          }
        }

        // Draw checkboxes to disable parameters
        for (int i = 0;
             i < aesthetic_camera_component->constant_parameters.size(); ++i) {
          boost::format label("Parameter %d");
          label % i;
          bool val = aesthetic_camera_component->constant_parameters[i];
          if (ImGui::Checkbox(label.str().c_str(), &val)) {
            aesthetic_camera_component->constant_parameters[i] = val;
          }
        }
      }
    }

    ImGui::DragFloat("Separation Distance", &ai_manager->separation_distance);
    ImGui::DragFloat("Separation", &ai_manager->separation);
    ImGui::DragFloat("Clustering Distance", &ai_manager->clustering_distance);
    ImGui::DragFloat("Clustering", &ai_manager->clustering);
    ImGui::DragFloat("Max Speed", &ai_manager->max_speed);

    ImGui::End();

    if (ImGui::IsKeyPressed(GLFW_KEY_SLASH)) {
      main_camera = !main_camera;
    }

    if (ImGui::IsKeyPressed(GLFW_KEY_TAB)) {
      glfwSetInputMode(pxl::Game::State.window, GLFW_CURSOR,
                       glfwGetInputMode(pxl::Game::State.window, GLFW_CURSOR) ==
                               GLFW_CURSOR_DISABLED
                           ? GLFW_CURSOR_NORMAL
                           : GLFW_CURSOR_DISABLED);
      auto fps = player->GetComponent<pxl::FpsController>();
      fps->disable = !fps->disable;
    }

    if (ImGui::IsKeyDown(GLFW_KEY_LEFT_CONTROL) &&
        ImGui::IsKeyPressed(GLFW_KEY_P)) {
      paused = !paused;
    }

    // Draw scene from primary camera
    scene->camera = camera;
    pxl::SceneRenderer::RenderScene(*scene, framebuffers[0]);
    glEnable(GL_BLEND);
    pxl::OglTextureRenderer::GetInstance()->RenderTexture(
        *reticle_tex,
        Eigen::Rectf(
            Eigen::Vector2f(.5f - .015 / pxl::Game::State.GetAspectRatio(),
                            .485f),
            Eigen::Vector2f(.5f + .015 / pxl::Game::State.GetAspectRatio(),
                            .515f)));
    glDisable(GL_BLEND);
    framebuffers[0]->End();

    // Draw scene from aesthetic camera
    framebuffers[1]->Begin();
    aesthetic_camera->Update(time_elapsed);
    scene->camera = aesthetic_camera;
    pxl::SceneRenderer::RenderScene(*scene, framebuffers[1]);
    framebuffers[1]->End();

    // Draw scene from 3rd person camera
    framebuffers[2]->Begin();
    aesthetic_camera->Update(time_elapsed);
    scene->camera = behind_camera;
    pxl::SceneRenderer::RenderScene(*scene, framebuffers[2]);
    framebuffers[2]->End();

    // Draw scene from overhead camera
    framebuffers[3]->Begin();
    aesthetic_camera->Update(time_elapsed);
    scene->camera = overhead_camera;
    pxl::SceneRenderer::RenderScene(*scene, framebuffers[3]);
    framebuffers[3]->End();

    // Draw final results to back buffer
    std::shared_ptr<pxl::OglFramebuffer> main_viewport = framebuffers[0];
    std::shared_ptr<pxl::OglFramebuffer> sub_viewport = framebuffers[1];
    std::shared_ptr<pxl::OglFramebuffer> sub_sub_viewport = framebuffers[2];
    std::shared_ptr<pxl::OglFramebuffer> sub_sub_sub_viewport = framebuffers[3];

    if (!main_camera) {
      std::swap(main_viewport, sub_viewport);
    }

    // auto mouse_pos =
    //     ImGui::GetMousePos() *
    //     ImVec2(pxl::SceneRenderer::shadow_buffer_->GetColorAttachment(0)
    //                ->GetWidth(),
    //            pxl::SceneRenderer::shadow_buffer_->GetColorAttachment(0)
    //                ->GetHeight()) /
    //     ImVec2(pxl::Game::State.window_width,
    //     pxl::Game::State.window_height);
    // mouse_pos.y =
    //     pxl::SceneRenderer::shadow_buffer_->GetColorAttachment(0)->GetHeight()
    //     - mouse_pos.y;
    //
    // auto pixel =
    //     pxl::SceneRenderer::shadow_buffer_->ReadPixel(mouse_pos.x,
    //     mouse_pos.y);
    // ImGui::BeginTooltip();
    // ImGui::Text("(%f, %f, %f, %f)", pixel.x(), pixel.y(), pixel.z(),
    // pixel.w()); ImGui::EndTooltip();

    pxl::OglTextureRenderer::GetInstance()->RenderTexture(
        *main_viewport->GetColorAttachment(0),
        Eigen::Rectf(Eigen::Vector2f(0, .5), Eigen::Vector2f(.5, 1)));
    // pxl::OglTextureRenderer::GetInstance()->RenderTexture(
    //    *pxl::SceneRenderer::shadow_buffer_->GetColorAttachment(0));
    // pxl::OglTextureRenderer::GetInstance()->RenderTexture(
    //    *std::dynamic_pointer_cast<pxl::OglMaterial>(
    //         block->GetComponent<pxl::OglMesh>()->materials[0])
    //         ->diffuse_texture);
    pxl::OglTextureRenderer::GetInstance()->RenderTexture(
        *sub_viewport->GetColorAttachment(0),
        Eigen::Rectf(Eigen::Vector2f(.5, .5), Eigen::Vector2f(1, 1)));
    pxl::OglTextureRenderer::GetInstance()->RenderTexture(
        *sub_sub_viewport->GetColorAttachment(0),
        Eigen::Rectf(Eigen::Vector2f(0, 0), Eigen::Vector2f(.5, .5)));
    pxl::OglTextureRenderer::GetInstance()->RenderTexture(
        *sub_sub_sub_viewport->GetColorAttachment(0),
        Eigen::Rectf(Eigen::Vector2f(.5, 0), Eigen::Vector2f(1, .5)));

    if (paused) {
      auto paused_size = ImGui::CalcTextSize("PAUSED") / 2;
      auto window_size =
          ImVec2(pxl::Game::State.window_width, pxl::Game::State.window_height);
      auto window_fourth = window_size / 4;
      auto drawlist = ImGui::GetOverlayDrawList();
      drawlist->AddText(window_fourth - paused_size, IM_COL32_WHITE, "PAUSED");
      drawlist->AddText(window_size - window_fourth - paused_size,
                        IM_COL32_WHITE, "PAUSED");
      drawlist->AddText(ImVec2(window_size.x, window_fourth.y) -
                            ImVec2(window_fourth.x, 0) - paused_size,
                        IM_COL32_WHITE, "PAUSED");
      drawlist->AddText(ImVec2(window_fourth.x, window_size.y) -
                            ImVec2(0, window_fourth.y) - paused_size,
                        IM_COL32_WHITE, "PAUSED");
    }
  }

  bool paused;
  bool main_camera;
  std::vector<std::shared_ptr<pxl::OglFramebuffer>> framebuffers;
  std::shared_ptr<pxl::OglFramebuffer> viewport_framebuffer;
  std::shared_ptr<pxl::Camera> camera;
  std::shared_ptr<pxl::Camera> aesthetic_camera;
  std::shared_ptr<pxl::Camera> behind_camera;
  std::shared_ptr<pxl::Camera> overhead_camera;
  std::shared_ptr<pxl::Scene> scene;
  std::shared_ptr<pxl::Empty> empty;
  std::shared_ptr<pxl::MeshEntity> mesh;
  std::shared_ptr<pxl::MeshEntity> ground;
  std::shared_ptr<pxl::MeshEntity> block;
  std::shared_ptr<pxl::MeshEntity> player;
  std::shared_ptr<pxl::Program> prog;
  std::shared_ptr<pxl::OglTexture2d> reticle_tex;

  std::shared_ptr<AiManager> ai_manager;
};

int main(int argc, char* argv[]) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  ThesisDemo game;
  game.Run();
  return 0;
}
