#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/camera.h"
#include "scene/resources/primitive_meshes.h"

namespace godotvis {

class GodotScene {
  SceneTree* tree_ = NULL;
  Spatial* scene_root_ = NULL; //!< This will be freed by scene tree. Keep the pointer to add intances.
  Camera* camera_ = NULL; //!< This will be freed by scene tree. Keep the pointer to update.
  Environment* env = NULL;
  MeshInstance* mesh_instance_ = NULL;
  Ref<ShaderMaterial> shader_material_;
  SpatialMaterial* material = NULL;
  CubeMesh* cube_ = nullptr;
  SphereMesh* sphere_ = nullptr;
  CylinderMesh* cylinder_ = nullptr;
  std::vector<int> mesh_instance_ids_;

public:
  GodotScene() {}

  void Initialize();
  void SetupEnvironment(const std::string& env_filename);

  /// Add a camera to the scene. Only support one camera for now.
  /// TODO: handle distortion
  void AddCamera(double fov_y, double z_near, double z_far);

  void SetCameraPose(const Eigen::Isometry3d& X_WI);

  Ref<Image> Capture();
  void ApplyDepthShader();
  void ApplyMaterialShader();
  void Finish();
  int AddMeshInstance(const std::string& filename);
  int AddCubeInstance(double x_length, double y_length, double z_length);
  int AddSphereInstance(double radius);
  int AddCylinderInstance(double radius, double height);

  void SetInstancePose(int id, const Eigen::Isometry3d& X_WI);

  void SetInstancePose(Spatial* instance, const Eigen::Isometry3d& X_WI);

  void SetInstanceScale(int id, double sx, double sy, double sz);

private:
  void InitDepthShader();
  Spatial* get_spatial_instance(int id);
  Ref<Mesh> LoadMesh(const std::string& filename);
  int AddInstance(const Ref<Mesh>& mesh);
};

/// Utility functon to convert Eigen's transform to Godot's transform
Transform ConvertToGodotTransform(const Eigen::Isometry3d& transform);

} // namespace godotvis

