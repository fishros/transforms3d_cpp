#ifndef TRANSFORMS3D_PCLTOOL_H_
#define TRANSFORMS3D_PCLTOOL_H_

#include <transforms3d/nanoflann.hpp>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;

struct PointCloud {
  std::vector<Vector3d> pts;
  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx].x();
    else if (dim == 1)
      return pts[idx].y();
    else
      return pts[idx].z();
  }
  template <class BBOX> bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};


void RadiusFilter(PointCloud& cloud, PointCloud& cloud_filtered, double radius, int min_pts);

#endif // TRANSFORMS3D_PCLTOOL_H_
