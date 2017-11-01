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

#include "godot_renderer.h"

#define OBJECT_COUNT 50

struct InstanceInfo {

	RID instance;
	Transform base;
	Vector3 rot_axis;
};

void SetupScene() {
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

	int object_count = OBJECT_COUNT;

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

int main(int argc, char *argv[]) {
  GodotRenderer renderer(640, 480);
  renderer.Initialize();
	SetupScene(); // This create the cubes and add them to the visual server

  renderer.MainLoop();

  renderer.Cleanup();

	return 0; //os_.get_exit_code();
}
