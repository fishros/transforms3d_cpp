/*
 * @Descripttion:
 * @Author: 小鱼
 * @Date: 2021-08-09 21:04:19
 * @LastEditTime: 2021-08-09 23:39:20
 */
#include <cmath>
#include <iostream>
#include <transforms3d/nanoflann.hpp>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;

#include "gtest/gtest.h"

struct PointCloud
{
    std::vector<Vector3d> pts;
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x();
        else if (dim == 1)
            return pts[idx].y();
        else
            return pts[idx].z();
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const
    {
        return false;
    }
};

TEST(TestKDTREE, TestSearch)
{
    /* base@llaser1  */
    Matrix4d base2llaser1 = TransForms::ComposeEuler(0, 0, 1, 0, 0, 0);

    TransFormsGroup tfg;
    tfg.pushTransForm("base_link", "laser", base2llaser1);
    std::vector<Vector3d> points;
    PointCloud pcd;
    int size = 10;
    for (int i = 0; i < size; i++)
    {
        points.push_back({0, 0, (float)(i + 1) / 10.0});
        pcd.pts.push_back({0, 0, (float)(i + 1) / 10.0});
        // printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
    }
    pcd.pts.push_back({0, 2.0, 1 / 1000.0});

    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3>;
    my_kd_tree_t tree(3, pcd, {10});
    const double search_radius = static_cast<double>(0.2);
    std::vector<std::pair<uint32_t, double>> ret_matches;

    nanoflann::SearchParams params;
    // params.sorted = false;
    const double query_pt[3] = {0.0, 0.0, 0.0};

    const size_t nMatches =
        tree.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
    cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches
         << " matches\n";
    for (size_t i = 0; i < nMatches; i++)
    {
        cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i
             << "]=" << ret_matches[i].second << endl;
        cout << "\n";
    }
}