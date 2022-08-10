#include <iostream>
#include <transforms3d/pcdtool.h>
using namespace std;
using namespace Eigen;

void RadiusFilter(PointCloud &cloud, PointCloud &cloud_filtered, double radius,
                  int min_pts) {
  using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3>;
  my_kd_tree_t tree(3, cloud, {10});
  const double search_radius = static_cast<double>(radius);
  std::vector<std::pair<uint32_t, double>> ret_matches;
  nanoflann::SearchParams params;
  // params.sorted = false;
  double query_pt[3];

  uint32_t point_size = cloud.pts.size();
  cloud_filtered.pts.clear();
  for (uint32_t i = 0; i < point_size; i++) {
    query_pt[0] = cloud.pts[i].x();
    query_pt[1] = cloud.pts[i].y();
    query_pt[2] = cloud.pts[i].z();
    size_t nMatches =
        tree.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
    printf("nMatches:%ld\n", nMatches);
    if (nMatches > min_pts) {
      cloud_filtered.pts.push_back(cloud.pts[i]);
    }
  }
}

// const size_t nMatches =
//     tree.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
// cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches
//      << " matches\n";
// for (size_t i = 0; i < nMatches; i++)
// {
//     cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i
//          << "]=" << ret_matches[i].second << endl;
//     cout << "\n";
// }