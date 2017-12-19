#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <iostream>

#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "drake/systems/sensors/godot_renderer/godot_scene.h"

using namespace godotvis;

std::string path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";

int main(int argc, char *argv[]) {
  GodotRenderer renderer(640, 480);

  GodotScene scene;
  scene.Initialize();
  scene.SetupEnvironment(path + "night.hdr");
  scene.AddCamera(65.0, 0.1, 100.0);

  Eigen::Isometry3d camera_pose{Eigen::Isometry3d::Identity()};
  camera_pose.translation() = Eigen::Vector3d(0., 2.5, 15.);
  scene.SetCameraPose(camera_pose);

  int id = scene.AddMeshInstance(path + "godot_ball.mesh");
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  //scene.SetInstancePose(id, pose);
  scene.SetInstanceScale(id, 0.8, 0.8, 0.8);

  Transform godot_T = ConvertToGodotTransform(pose);
  Transform expected_T{Basis{1., 0., 0., 0., 1., 0., 0., 0., 1.},
                       Vector3{0., 2.5, 15.0}};
  if (godot_T != expected_T)
    std::cout << "FAIL!" << std::endl;
  else
    std::cout << "Pass!" << std::endl;

  int cube_id = scene.AddCubeInstance(1., 1., 1.);
  int sphere_id = scene.AddSphereInstance(1.);
  int cylinder_id = scene.AddCylinderInstance(0.5, 1.0);
  pose.translation() = Eigen::Vector3d(1.0, 0., 0.);
  scene.SetInstancePose(sphere_id, pose);
  pose.translation() = Eigen::Vector3d(-1.0, 0., 0.);
  scene.SetInstancePose(cylinder_id, pose);

  scene.set_viewport_size(640, 480);
  renderer.Draw();
  // TODO: Why can't I use Ref<Image> variable?
  Ref<Image> image = scene.Capture();
  std::cout << "Image format: " << image->get_format() << std::endl;
  std::cout << "Expected format: " << Image::FORMAT_RGBA8 << std::endl;
  image->save_png("/home/duynguyen/Downloads/rgb1.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  image = scene.Capture();
  std::cout << "Depth Image format: " << image->get_format() << std::endl;
  std::cout << "Depth Expected format: " << Image::FORMAT_RGBA8 << std::endl;
  image->save_png("/home/duynguyen/Downloads/depth1.png");

  scene.set_viewport_size(160, 120);
  scene.ApplyMaterialShader();
  renderer.Draw();
  image = scene.Capture();
  image->save_png("/home/duynguyen/Downloads/rgb2.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  image = scene.Capture();
  image->save_png("/home/duynguyen/Downloads/depth2.png");

  scene.set_viewport_size(320, 240);
  scene.ApplyMaterialShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/rgb3.png");

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png("/home/duynguyen/Downloads/depth3.png");

  scene.Finish();
  image.unref();

  return 0; //os_.get_exit_code();
}
