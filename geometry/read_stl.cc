#include "drake/geometry/read_obj.h"

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

std::shared_ptr<std::vector<Eigen::Vector3d>> ReadSTLFileVerticesArmstrong(const std::string& filename, double scale, double padding) {
  std::stringstream padding_str;
  padding_str << std::fixed << std::setprecision(3) << padding;

  boost::filesystem::path unpadded_mesh_path(filename);
  std::string padded_mesh_path = unpadded_mesh_path.parent_path().parent_path().string() + "/collision/padded_meshes/" + unpadded_mesh_path.stem().string() + "_padded_"+padding_str.str()+"m.stl";
  
  return ReadSTLFileVertices(padded_mesh_path, scale);
}

std::shared_ptr<std::vector<Eigen::Vector3d>> ReadSTLFileVertices(const std::string& filename, double scale) {
  shapes::Mesh* padded_mesh = shapes::createMeshFromResource(filename);

  // The position for each vertex vertex k has values at index (3k, 3k+1, 3k+2) = (x,y,z)
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(padded_mesh->vertex_count);
  for (size_t pti=0; pti<padded_mesh->vertex_count; pti+=3) {
    double x = padded_mesh->vertices[pti] * scale;
    double y = padded_mesh->vertices[pti+1] * scale;
    double z = padded_mesh->vertices[pti+2] * scale;
    vertices.emplace_back(x, y, z);
  }

  return vertices;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
