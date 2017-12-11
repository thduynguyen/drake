#pragma once

// Define this macro so glad.h will be included in shader_gles3.h
#define GLES3_INCLUDE_H "glad/glad.h"

#include "core/error_list.h"
#include "core/register_core_types.h"
#include "os/file_access.h"
#include "drivers/gles3/rasterizer_gles3.h"
#include "drivers/register_driver_types.h"
#include "drivers/unix/dir_access_unix.h"
#include "drivers/unix/file_access_unix.h"
#include "os/thread_dummy.h"
#include "os_dummy.h"
#include "project_settings.h"
#include "scene/main/scene_tree.h"
#include "scene/register_scene_types.h"
#include "servers/register_server_types.h"
#include "servers/visual/visual_server_raster.h"

// TL;DR: GLFW has to be included here, after rasterizer_gles3.
// glfw needs to be included *after* glad.h, which is
// included by shader_gles3.h in rasterizer.gles3.h
#include <GLFW/glfw3.h>

class GodotRenderer {
  int window_width_, window_height_;
  OS_Dummy os_;
  GLFWwindow *window_ = nullptr;
  VisualServer *visual_server_ = nullptr;
  ARVRServer *arvr_server_ = nullptr;
  ProjectSettings *globals_ = nullptr;
  Physics2DServer* physics_2d_server_ = nullptr; //!< Unfortunately this is needed in Viewport's ctor
  PhysicsServer* physics_server_ = nullptr; //!< Unfortunately this is needed in Viewport's ctor

public:
  GodotRenderer(int window_width, int window_height)
    : window_width_(window_width), window_height_(window_height), os_(window_width, window_height) {}

  void Initialize() {
    InitGLContext();
    InitGodot();
  }

  void Draw() {
    glClear(GL_COLOR_BUFFER_BIT);
    VSG::scene->update_dirty_instances(); //update scene stuff
    VSG::viewport->draw_viewports();
    VSG::scene->render_probes();
  }

  void MainLoop() {
    do {
      Draw();
      //Ref<Image> image = viewport_->get_texture()->get_data();
      //image->save_png("/home/duynguyen/Downloads/texture.png");

      // TODO: modify this for render_to_texture
      glfwSwapBuffers(window_);
      glfwPollEvents();
    } while (glfwGetKey(window_, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
         glfwWindowShouldClose(window_) == 0);
  }

  void Cleanup() {
    CleanupGodot();
    CleanupGL();
  }

  GLFWwindow *const get_glfw_window() const { return window_; }

  int width() const { return window_width_; }
  int height() const { return window_height_; }

private:
  // TODO: modify this for render_to_texture
  Error InitGLContext() {
    if (!glfwInit()) {
      fprintf(stderr, "Failed to initialize GLFW\n");
      return FAILED;
    }
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL
    //glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

    // Open a window_ and create its OpenGL context
    window_ = glfwCreateWindow(window_width_,
        window_height_, "", NULL, NULL);
    if (window_ == NULL) {
      fprintf(stderr, "Failed to open GLFW window_.\n");
      glfwTerminate();
      return FAILED;
    }
    glfwMakeContextCurrent(window_); // Initialize GLEW
    glfwSetInputMode(window_, GLFW_STICKY_KEYS, GL_TRUE);
    return OK;
  }

  void InitGodot() {
    // Main::setup()
    RID_OwnerBase::init_rid();

    /// THESE are needed for ObjectDB operations
    ThreadDummy::make_default();
    SemaphoreDummy::make_default();
    MutexDummy::make_default();
    RWLockDummy::make_default();
    IPDummy::make_default();

    // Do we need this? Can we just pass resources directly from Drake?
    FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_RESOURCES);
    FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_USERDATA);
    FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_FILESYSTEM);
    ////FileAccessBufferedFA<FileAccessUnix>::make_default();
    ////TODO: Change to DirAccessOSX for osx platform!!!
    DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_RESOURCES);
    DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_USERDATA);
    DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_FILESYSTEM);
    // ============================

    ClassDB::init();

    register_core_types();
    register_core_driver_types();

    // Needed for GLOBAL_DEF in VisualServer and Rasterizer
    globals_ = memnew(ProjectSettings);
    register_core_settings(); //here globals_ is present

    //////////////////////////////////////
    // Main setup2
    ////////////////////////////////////////////////////////////////

    RasterizerGLES3::register_config();
    RasterizerGLES3::make_current();

    visual_server_ = memnew(VisualServerRaster);
    visual_server_->init();

    //=====================================
    register_server_types();
    Color clear = GLOBAL_DEF("rendering/environment/default_clear_color", Color(0.3, 0.3, 0.3));
    VisualServer::get_singleton()->set_default_clear_color(clear);

    register_scene_types();

    arvr_server_ = memnew(ARVRServer); // Needed for VisualServer::draw(), in VisualServerViewport::draw_viewports()

    physics_2d_server_ = Physics2DServerManager::new_default_server();
    /// 3D Physics Server
		physics_server_ = PhysicsServerManager::new_default_server();
  }

  void CleanupGodot() {
    if (physics_2d_server_)
      memdelete(physics_2d_server_);

    if (physics_server_)
      memdelete(physics_server_);

    // Cleanup
    unregister_scene_types();
    unregister_server_types();

    visual_server_->finish();
    memdelete(visual_server_);

    if (arvr_server_)
      memdelete(arvr_server_);

    if (globals_)
      memdelete(globals_);

    unregister_core_driver_types();
    unregister_core_types();
  }

  void CleanupGL() {
    glfwTerminate();
  }
};
