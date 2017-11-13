#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include "godot_renderer.h"
#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/camera.h"

#define DEGREE M_PI / 180.0

String path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";

class GodotScene {
  SceneTree* tree_ = NULL;
  Spatial* scene_root_ = NULL; //!< This will be freed by scene tree. Keep the pointer to add intances.
  Camera* camera_ = NULL; //!< This will be freed by scene tree. Keep the pointer to update.
  Environment* env;
  MeshInstance* mesh_instance_ = NULL;
  Ref<ShaderMaterial> shader_material_;
  SpatialMaterial* material = NULL;

public:
  GodotScene() {}

  void Initialize() {
    // Construct and initialize a SceneTree
    // This creates a default Viewport as the root
    // and a World attached to that Viewport
    // The World creates and contains a VisualServer's scenario
    // We need to attach an environment to the World and other
    // visual instance (camera, meshes etc) to the tree_.
    tree_ = memnew(SceneTree);
    tree_->init();

    // Load skybox resource
    String skyfilename = path + "night.hdr";
    Ref<Texture> sky_texture = ResourceLoader::load(skyfilename);
    // TODO: This will be freed by Environment. Also, what's the correct way to create a Ref<Sky>?
    PanoramaSky* sky = memnew(PanoramaSky);
    sky->set_panorama(sky_texture);
    sky->set_radiance_size(Sky::RADIANCE_SIZE_64);

    //// Set Environment
    env = memnew(Environment);
    env->set_background(Environment::BG_SKY);
    env->set_sky(sky);
    env->set_bg_energy(1.0);
    tree_->get_root()->get_world()->set_environment(env);

    // Dummy Spatial as the top root of the scene
    scene_root_ = memnew(Spatial);
    tree_->add_current_scene(scene_root_); // need to do this here so all subsequent children knows about the tree_, espcially the camera

    // Add camera
    camera_ = memnew(Camera);
    scene_root_->add_child(camera_);
    camera_->set_perspective(65.0, 0.1, 100.0);
    Transform Tc(Basis(), Vector3(0, 2.5, 15.0));
    camera_->set_transform(Tc);
    camera_->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED); // SceneTree::_flush_transform_notifications() is private, so we have to do this :(
  }

  void SetupScene() {
    // Load a mesh
    // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(), which initializes VAO
    // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <-- ArrayMesh::_setv() <-- res->set() <-- ResourceInteractiveLoaderBinary::poll()
    String filename = path + "godot_ball.mesh";
    Ref<ArrayMesh> mesh = ResourceLoader::load(filename);
    // Load its material
    material = memnew(SpatialMaterial);
    material->set_albedo(Color(1.0, 1.0, 1.0, 1.0));
    Ref<Texture> texture = ResourceLoader::load(path + "aluminium_albedo.png");
    material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
    material->set_metallic(0.59);
    material->set_specular(0.5);
    material->set_roughness(0.4);
    Ref<Texture> normal_tex = ResourceLoader::load(path + "aluminium_normal.png");
    material->set_feature(SpatialMaterial::FEATURE_NORMAL_MAPPING, true);
    material->set_texture(SpatialMaterial::TEXTURE_NORMAL, normal_tex);
    material->set_normal_scale(0.21);
    material->set_feature(SpatialMaterial::FEATURE_ANISOTROPY, true);
    material->set_anisotropy(0.99);
    // IMPORTANT: This calls to SpatialMaterial::_update_shader(). Without this materials wont' work
    // Godot sets up so that this is called in SceneTree::call_idle_call_backs()
    // Not sure if we need to do this as an idle callback...
    SpatialMaterial::flush_changes();
    mesh->surface_set_material(0, material);
    mesh->surface_set_material(1, material);
    mesh->surface_set_material(2, material);
    mesh->surface_set_material(3, material);

    shader_material_.instance();
    Shader* shader = memnew(Shader);
    String code = "shader_type spatial; \nrender_mode unshaded;\n";
    code += "void fragment() {\n";
    code += "\tfloat near = 2.0, far = 3.0;\n";
    code += "\tfloat dis = -VERTEX.z; // TODO can easily switch to real distance instead of just z\n";
    code += "\tfloat depth_color = (dis-near)/(far-near)*1.0;\n";
    code += "\tvec3 viewing_ray = normalize(VERTEX);\n";
    code += "\tif (abs(dot(NORMAL, viewing_ray)) < 0.3) {\n";
    code += "\t\t	depth_color = 0.0;\n";
    code += "\t}\n";
    code += "\tALBEDO = vec3(depth_color, depth_color, depth_color);\n";
    code += "\tALPHA = 1.0;\n";
    code += "}\n";
    shader->set_code(code);
    shader_material_->set_shader(shader);

    // Instantiate the mesh from its mesh and material resources
    mesh_instance_ = memnew(MeshInstance);
    scene_root_->add_child(mesh_instance_); // Must add before doing any settings
    mesh_instance_->set_mesh(mesh);
    Basis R(Vector3(0.0, 1.0, 0.0), 90.0 * DEGREE);
    R = Basis(Vector3(0.0, 0.0, 1.0), -10.0 * DEGREE) * R;
    mesh_instance_->set_transform(Transform(Basis(), Vector3(0, 0.0, 0.)));
    mesh_instance_->set_scale(Vector3(0.8, 0.8, 0.8));
    mesh_instance_->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED);

    // Add lights
    Light* light = memnew(OmniLight());
    scene_root_->add_child(light);
    light->set_color(Color(1.0, 1.0, 1.0));
    light->set_param(Light::PARAM_ENERGY, 2.0);
    light->set_param(Light::PARAM_RANGE, 50.0);
    light->set_transform(Transform(Basis(), Vector3(1.0, 5., 10.)));
    light->notification(Spatial::NOTIFICATION_TRANSFORM_CHANGED);
  }

  Ref<Image> Capture() {
    return tree_->get_root()->get_texture()->get_data();
  }

  void ApplyDepthShader() {
    Ref<Mesh> mesh = mesh_instance_->get_mesh();
    for (int i = 0; i < mesh->get_surface_count(); ++i)
      mesh_instance_->set_surface_material(i, shader_material_);
    SpatialMaterial::flush_changes();
    env->set_background(Environment::BG_CLEAR_COLOR);
  }

  void ApplyMaterialShader() {
    Ref<Mesh> mesh = mesh_instance_->get_mesh();
    for (int i = 0; i < mesh->get_surface_count(); ++i)
      mesh_instance_->set_surface_material(i, mesh->surface_get_material(i));
    SpatialMaterial::flush_changes();
    env->set_background(Environment::BG_SKY);
  }

  void Finish() {
    shader_material_.unref();
    if (tree_) {
      // This will free all the resources/instances that have been new-ed
      // and add to the tree
      tree_->finish();
      memdelete(tree_);
    }
  }

};


int main(int argc, char *argv[]) {
  GodotRenderer renderer(1280, 960);
  renderer.Initialize();
  GodotScene scene;
  scene.Initialize();
  scene.SetupScene();

  renderer.Draw();
  // TODO: Why can't I use Ref<Image> variable?
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb1.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth1.png");

  scene.ApplyMaterialShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb2.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth2.png");

  scene.ApplyMaterialShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb3.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth3.png");

  scene.Finish();
  renderer.Cleanup();

  return 0; //os_.get_exit_code();
}
