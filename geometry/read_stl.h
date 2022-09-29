#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

namespace drake {
namespace geometry {
namespace internal {

std::shared_ptr<std::vector<Eigen::Vector3d>> ReadSTLFileVerticesArmstrong(const std::string& filename, double scale, double padding=0.0);

std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadSTLFileVertices(const std::string& filename, double scale, double padding=0.0);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
