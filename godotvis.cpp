#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include "godot_renderer.h"
#include "scene/3d/mesh_instance.h"

#define DEGREE M_PI/180.0

String path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";

struct InstanceInfo {
	RID instance;
	Transform base;
	Vector3 rot_axis;
};

void SetupScene() {
	VisualServer *vs = VisualServer::get_singleton();
	RID scenario = vs->scenario_create();

  String filename = path + "godot_ball.mesh";
  // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(), which initialize VAO
  // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <-- ArrayMesh::_setv() <-- res->set() <-- ResourceInteractiveLoaderBinary::poll()
  Ref<Mesh> mesh = ResourceLoader::load(filename);

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
  //RID test_cube = vs->get_test_cube();
  RID test_cube = vs->mesh_create();
  vs->mesh_add_surface_from_mesh_data(test_cube, md);

  RID instance = vs->instance_create2(test_cube, scenario);

  //RID mesh_id = mesh->get_rid();
  SpatialMaterial* material = memnew(SpatialMaterial);
  material->set_albedo(Color(1.0, 0., 0.));
  vs->mesh_surface_set_material(test_cube, 0, material->get_rid());
  std::cout << "NUM SURFACES: " << vs->mesh_get_surface_count(test_cube) << std::endl;

  // This calls vs->create_instance() in VisualInstance constructor
  // Using instance_create2() doesn't work, probably because the object_id is not attached to the instance
  // (VS::instance_attach_object_instance_id call in VisualInstance's ctor)
  // The instance is created and linked with the scenario, however, its data is missing.
  // Doing this is not ideal, however, as nobody manages this pointer, it's not freed at the end
  // hence the program doesn't exit naturally.
  MeshInstance *mesh_instance = memnew(MeshInstance);
  mesh_instance->set_mesh(mesh);
  // put this instance into the scenario. This is important as viewport's render_scene
  // render_camera function checks these instances and culls them out
  vs->instance_set_scenario(mesh_instance->get_instance(), scenario);
  //RID mesh_instance = vs->instance_create2(mesh->get_rid(), scenario);
  vs->mesh_surface_set_material(mesh->get_rid(), 0, material->get_rid());
  Rect3 aabb = mesh->get_aabb();
  Vector3 position = aabb.position;

  //vs->instance_set_transform(instance, Transform(Basis(), Vector3()));

	RID camera = vs->camera_create();

  //Transform Tc_in = Transform(Basis(Vector3(0.0, -90.0 * DEGREE, 0.0)).transposed(), Vector3(20.0, 0.0, 0.0));
  Transform Tc;
  Tc.set_look_at(Vector3(20., 0., 0.), position, Vector3(0, 1, 0));
  vs->camera_set_transform(camera, Tc);
	vs->camera_set_perspective(camera, 65, 0.1, 100);


	RID viewport = vs->viewport_create();
	Size2i screen_size = OS::get_singleton()->get_window_size();
	vs->viewport_set_size(viewport, screen_size.x, screen_size.y);
	vs->viewport_attach_to_screen(viewport, Rect2(Vector2(), screen_size));
	vs->viewport_set_active(viewport, true);
	vs->viewport_attach_camera(viewport, camera);
	vs->viewport_set_scenario(viewport, scenario);


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
	lla.set_look_at(Vector3(20., 0, 0), position, Vector3(0, 1, 0));

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
