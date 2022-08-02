/*
 * @Descripttion:
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 23:39:20
 */
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;

Matrix4d TransFormsGroup::getTransForm(const std::string &start,
                                       const std::string &end) {
  Matrix4d matrix;
  matrix.setIdentity(); //单位矩阵
  std::vector<std::string> paths = findPath(start, end);
  if (paths.size() <= 1) {
    return matrix;
  }
  for (int i = 0; i < paths.size() - 1; i++) {
    for (Child c : tfg[paths[i]]) {
      if (c.name == paths[i + 1]) {
        matrix *= c.t;
      }
    }
  }
  return matrix;
}
Matrix4d TransFormsGroup::pushTransForm(const std::string &parent,
                                        const std::string &child, double &x,
                                        double &y, double &z, double &rx,
                                        double &ry, double &rz) {
  return pushTransForm(parent, child,
                       TransForms::ComposeEuler(x, y, z, rx, ry, rz));
}
Matrix4d TransFormsGroup::pushTransForm(const std::string &parent,
                                        const std::string &child, double &x,
                                        double &y, double &z, double &rx,
                                        double &ry, double &rz, double &rw) {
  Vector3d positon(x, y, z);
  Quaterniond quat(rx, ry, rz, rw);
  return pushTransForm(parent, child, TransForms::Compose(positon, quat));
}
Matrix4d TransFormsGroup::pushTransForm(const std::string &parent,
                                        const std::string &child,
                                        Matrix4d matrix) {
  bool has_parent_node = true, has_child_node = true;
  if (tfg.find(parent) == tfg.end())
    has_parent_node = false;
  if (tfg.find(child) == tfg.end())
    has_child_node = false;

  // 分情况判断
  if (has_parent_node) {
    std::vector<Child> childs = tfg[parent];
    bool has_child = false;
    for (int i = 0; i < childs.size(); i++) {
      if (childs[i].name == child) {
        has_child = true;
        // 更新已有的矩阵
        tfg[parent][i].t = matrix;
      }
    }
    if (!has_child) {
      // 没有这个孩子，添加孩子
      Child new_child;
      new_child.name = child;
      new_child.t = matrix;
      tfg[parent].push_back(new_child);
    }
  } else {
    // 添加一个节点
    Child new_child;
    new_child.name = child;
    new_child.t = matrix;
    std::vector<Child> new_childs;
    new_childs.push_back(new_child);
    //添加一个父节点
    tfg[parent] = new_childs;
  }

  // 分情况判断
  if (has_child_node) {
    std::vector<Child> childs = tfg[child];
    bool has_child = false;
    for (int i = 0; i < childs.size(); i++) {
      if (childs[i].name == child) {
        has_child = true;
        // 更新已有的矩阵
        tfg[child][i].t = matrix;
      }
    }
    if (!has_child) {
      // 没有这个孩子，添加孩子
      Child new_child;
      new_child.name = parent;
      new_child.t = matrix.inverse();
      tfg[child].push_back(new_child);
    }
  } else {
    // 添加一个节点
    Child new_child;
    new_child.name = parent;
    new_child.t = matrix.inverse();
    std::vector<Child> new_childs;
    new_childs.push_back(new_child);
    tfg[child] = new_childs;
  }
  return matrix;
}

std::string TransFormsGroup::toString() {
  std::string str;
  std::map<std::string, std::vector<Child>>::iterator iter;
  for (iter = tfg.begin(); iter != tfg.end(); iter++) {
    // cout << "node:" << iter->first << endl;
    str += "node:" + iter->first + "\n";
    for (int i = 0; i < iter->second.size(); i++) {
      // cout <<"\tchild" << i << " : " << iter->second[i].name << endl;
      // cout <<"\t\tmatrix : ";
      VectorXd vxd = TransForms::H2EulerAngle(iter->second[i].t);
      // cout<<fixed<<setprecision(5)<<left << setw(14)<<vxd[0]<<
      // setw(14)<<vxd[1]<< setw(14)<<vxd[2]<< setw(14)<<vxd[3]<<
      // setw(14)<<vxd[4]<< setw(14)<<vxd[5]<<endl;
      str += "\tchild: " + iter->second[i].name +
             "\n\t\tmatrix :  " + std::to_string(vxd[0]) + "," +
             std::to_string(vxd[1]) + "," + std::to_string(vxd[2]) + "," +
             std::to_string(vxd[3]) + "," + std::to_string(vxd[4]) + "," +
             std::to_string(vxd[5]) + "\n";
    }
    cout << endl;
    str += "\n";
  }
  return str;
}

std::vector<std::string> TransFormsGroup::findPath(const std::string &start,
                                                   const std::string &end) {
  std::deque<TransFormsGroup::Path> que;
  std::vector<std::string> researched = {start};
  if (start == end)
    return researched;
  for (Child c : tfg[start]) {
    Path path = {c.name, {start, c.name}};
    que.push_back(path);
  }
  while (!que.empty()) {
    Path path = que.front();
    // cout<<"-----------------------------"<<endl;;
    // cout<<path.name<<":";
    // for(std::string p:path.path){
    //     cout<<"   "<<p;
    // }
    // cout<<"researched"<<":";
    // for(std::string r:researched){
    //     cout<<"   "<<r;
    // }
    // cout<<endl;
    // cout<<"before quesize:"<<que.size()<<endl;

    que.pop_front();

    if (std::find(researched.begin(), researched.end(), path.name) ==
        researched.end()) {
      // cout<<"not find path.name"<<endl;
      if (path.name == end) {
        // cout<<"fing node"<<endl;
        return path.path;
      } else {
        for (Child c : tfg[path.name]) {
          // cout<<"prepare add "<<c.name<<endl;
          if (std::find(researched.begin(), researched.end(), c.name) ==
              researched.end()) {
            std::vector<std::string> paths = {c.name};
            paths.insert(paths.begin(), path.path.begin(), path.path.end());
            Path temPath = {c.name, paths};
            que.push_back(temPath);
            // cout<<"added "<<c.name<<endl;
          }
        }
      }
    }
    // cout<<"quesize:"<<que.size()<<endl;
  }

  return researched;
}

/**
 * @brief 将点云从一个坐标系转换到另外一个坐标系
 *
 * @param base
 * @param point
 * @param target
 * @return std::vector<Vector3d>
 */
std::vector<Vector3d>
TransFormsGroup::getTransWithPointCloud(const std::string &base,
                                        std::vector<Vector3d> &points,
                                        const std::string &target) {
  Matrix4d base2target = getTransForm(target, base);
  Matrix3d rotate;
  Vector3d transpose;
  TransForms::HDecompose(base2target, rotate, transpose);
  for (uint64_t i = 0; i < points.size(); i++) {
    Vector3d temp_v3d = points[i].transpose() * rotate;
    points[i] = temp_v3d + transpose;
  }
  return points;
}