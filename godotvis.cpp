/*************************************************************************/
/*  godot_x11.cpp                                                        */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2017 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2017 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/
#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

// Define this macro so glad.h will be included in shader_gles3.h
#define GLES3_INCLUDE_H "glad/glad.h"

#include "core/error_list.h"
#include "core/register_core_types.h"
#include "drivers/gles3/rasterizer_gles3.h"
#include "drivers/register_driver_types.h"
#include "os/thread_dummy.h"
#include "os_dummy.h"
#include "project_settings.h"
#include "quick_hull.h"
#include "scene/main/scene_tree.h"
#include "scene/register_scene_types.h"
#include "servers/register_server_types.h"
#include "servers/visual/visual_server_raster.h"

// TL;DR: GLFW has to be included here, after rasterizer_gles3.
// glfw needs to be included *after* glad.h, which is
// included by shader_gles3.h in rasterizer.gles3.h
#include <GLFW/glfw3.h>


#define OBJECT_COUNT 50

struct InstanceInfo {

	RID instance;
	Transform base;
	Vector3 rot_axis;
};

void init() {

	print_line("INITIALIZING TEST RENDER");
	VisualServer *vs = VisualServer::get_singleton();
	RID test_cube = vs->get_test_cube();
	RID scenario = vs->scenario_create();

	Vector<Vector3> vts;

	vts.push_back(Vector3(1, 1, 1));
	vts.push_back(Vector3(1, -1, 1));
	vts.push_back(Vector3(-1, 1, 1));
	vts.push_back(Vector3(-1, -1, 1));
	vts.push_back(Vector3(1, 1, -1));
	vts.push_back(Vector3(1, -1, -1));
	vts.push_back(Vector3(-1, 1, -1));
	vts.push_back(Vector3(-1, -1, -1));

	Geometry::MeshData md;
	Error err = QuickHull::build(vts, md);
	print_line("ERR: " + itos(err));
	test_cube = vs->mesh_create();
	vs->mesh_add_surface_from_mesh_data(test_cube, md);

	List<String> cmdline = OS::get_singleton()->get_cmdline_args();
	int object_count = OBJECT_COUNT;
	if (cmdline.size() > 0 && cmdline[cmdline.size() - 1].to_int()) {
		object_count = cmdline[cmdline.size() - 1].to_int();
	};

	for (int i = 0; i < object_count; i++) {

		InstanceInfo ii;

		ii.instance = vs->instance_create2(test_cube, scenario);

		ii.base.translate(Math::random(-20, 20), Math::random(-20, 20), Math::random(-20, 18));
		ii.base.rotate(Vector3(0, 1, 0), Math::randf() * Math_PI);
		ii.base.rotate(Vector3(1, 0, 0), Math::randf() * Math_PI);
		vs->instance_set_transform(ii.instance, ii.base);

		ii.rot_axis = Vector3(Math::random(-1, 1), Math::random(-1, 1), Math::random(-1, 1)).normalized();
	}

	RID camera = vs->camera_create();

	// 		vs->camera_set_perspective( camera, 60.0,0.1, 100.0 );

	RID viewport = vs->viewport_create();
	Size2i screen_size = OS::get_singleton()->get_window_size();
	vs->viewport_set_size(viewport, screen_size.x, screen_size.y);
	vs->viewport_attach_to_screen(viewport, Rect2(Vector2(), screen_size));
	vs->viewport_set_active(viewport, true);
	vs->viewport_attach_camera(viewport, camera);
	vs->viewport_set_scenario(viewport, scenario);
	vs->camera_set_transform(camera, Transform(Basis(), Vector3(0, 3, 30)));
	vs->camera_set_perspective(camera, 60, 0.1, 1000);

	/*
		RID lightaux = vs->light_create( VisualServer::LIGHT_OMNI );
		vs->light_set_var( lightaux, VisualServer::LIGHT_VAR_RADIUS, 80 );
		vs->light_set_var( lightaux, VisualServer::LIGHT_VAR_ATTENUATION, 1 );
		vs->light_set_var( lightaux, VisualServer::LIGHT_VAR_ENERGY, 1.5 );
		light = vs->instance_create( lightaux );
		*/
	RID lightaux;

	lightaux = vs->light_create(VisualServer::LIGHT_DIRECTIONAL);
	//vs->light_set_color( lightaux, VisualServer::LIGHT_COLOR_AMBIENT, Color(0.0,0.0,0.0) );
	vs->light_set_color(lightaux, Color(1.0, 1.0, 1.0));
	//vs->light_set_shadow( lightaux, true );
	RID light = vs->instance_create2(lightaux, scenario);
	Transform lla;
	//lla.set_look_at(Vector3(),Vector3(1,-1,1),Vector3(0,1,0));
	lla.set_look_at(Vector3(), Vector3(-0.000000, -0.836026, -0.548690), Vector3(0, 1, 0));

	vs->instance_set_transform(light, lla);

	lightaux = vs->light_create(VisualServer::LIGHT_OMNI);
	//vs->light_set_color( lightaux, VisualServer::LIGHT_COLOR_AMBIENT, Color(0.0,0.0,1.0) );
	vs->light_set_color(lightaux, Color(1.0, 1.0, 0.0));
	vs->light_set_param(lightaux, VisualServer::LIGHT_PARAM_RANGE, 4);
	vs->light_set_param(lightaux, VisualServer::LIGHT_PARAM_ENERGY, 8);
	//vs->light_set_shadow( lightaux, true );
	//light = vs->instance_create( lightaux );
}

class GodotRenderer {
  int window_width, window_height;
  //TODO: Use nullptr when C++11 is enabled
  OS_Dummy os;
  GLFWwindow* window = NULL;
  VisualServer* visual_server = NULL;
  ARVRServer* arvr_server = NULL;
  ProjectSettings* globals = NULL;

public:
	GodotRenderer(int window_width, int window_height)
		: window_width(window_width), window_height(window_height) {}

	void Initialize() {
		Init_gl_context();
		Init_godot();
  }

  void Draw() {
		VSG::scene->update_dirty_instances(); //update scene stuff
		VSG::viewport->draw_viewports();
		VSG::scene->render_probes();
  }

  void MainLoop() {
    do {
      glClear(GL_COLOR_BUFFER_BIT);
      Draw();
      // TODO: modify this for render_to_texture
      glfwSwapBuffers(window);
      glfwPollEvents();
    } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
         glfwWindowShouldClose(window) == 0);
  }

  void Cleanup() {
    Cleanup_godot();
    Cleanup_gl();
  }

  GLFWwindow* const get_glfw_window() const { return window; }

private:
  // TODO: modify this for render_to_texture
  Error Init_gl_context() {
	  if (!glfwInit()) {
		  fprintf(stderr, "Failed to initialize GLFW\n");
		  return FAILED;
	  }
	  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
	  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

	  // Open a window and create its OpenGL context
	  window = glfwCreateWindow(window_width,
			  window_height, "Godot Render Window", NULL, NULL);
	  if (window == NULL) {
		  fprintf(stderr, "Failed to open GLFW window.\n");
		  glfwTerminate();
		  return FAILED;
	  }
	  glfwMakeContextCurrent(window); // Initialize GLEW
	  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
  }

  void Init_godot() {
    // Main::setup()
    RID_OwnerBase::init_rid();

    /// THESE are needed for ObjectDB operations
    ThreadDummy::make_default();
    SemaphoreDummy::make_default();
    MutexDummy::make_default();
    RWLockDummy::make_default();
    IPDummy::make_default();
    // ============================

    ClassDB::init();

    register_core_types();
    register_core_driver_types();

    // Needed for GLOBAL_DEF in VisualServer and Rasterizer
    globals = memnew(ProjectSettings);
    register_core_settings(); //here globals is present

    //////////////////////////////////////
    // Main setup2
    ////////////////////////////////////////////////////////////////

    RasterizerGLES3::register_config();
    RasterizerGLES3::make_current();

    visual_server = memnew(VisualServerRaster);
    visual_server->init();

    //=====================================
    register_server_types();
    Color clear = GLOBAL_DEF("rendering/environment/default_clear_color", Color(0.3, 0.3, 0.3));
    VisualServer::get_singleton()->set_default_clear_color(clear);

    register_scene_types();

    arvr_server = memnew(ARVRServer); // Needed for VisualServer::draw(), in VisualServerViewport::draw_viewports()
  }

  void Cleanup_godot() {
    // Cleanup
    unregister_scene_types();
    unregister_server_types();

    //memdelete(main_loop);
    visual_server->finish();
    memdelete(visual_server);

    if (arvr_server)
      memdelete(arvr_server);

    if (globals)
      memdelete(globals);

    unregister_core_driver_types();
    unregister_core_types();
  }

  void Cleanup_gl() {
    glfwTerminate();
  }
};

int main(int argc, char *argv[]) {

  GodotRenderer renderer(1024, 600);
  renderer.Initialize();
	init(); // This create the cubes and add them to the visual server
	std::cout << "before main loop..." << std::endl;

  renderer.MainLoop();

  renderer.Cleanup();

	return 0; //os.get_exit_code();
}
