#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1 >> 20
struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
  int id; // 1--> open set, -1 --> closed set, 0 --> 还未被拓展到
  double height;
  Eigen::Vector3d coord; // 节点坐标（所处栅格的中心点坐标）
  Eigen::Vector3i dir;   // direction of expanding
  Eigen::Vector3i index; // 栅格的ID号

  double gScore, fScore; // 路径得分和启发得分
  GridNodePtr cameFrom;  // 父节点
  std::multimap<double, GridNodePtr>::iterator nodeMapIt;
  bool exist_second_height = false;  // 是否存在第二高度
  bool second_local_swell = false;

  GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
  {
    id = 0;
    height = -1.0;
    index = _index;
    coord = _coord;
    dir = Eigen::Vector3i::Zero();

    gScore = inf;
    fScore = inf;
    cameFrom = NULL;
    exist_second_height = false;
  }

    GridNode(int _X, int _Y, int _Z)
    {
        id = 1;
        height = -1.0;
        index(0) = _X;
        index(1) = _Y;
        index(2) = _Z;
        dir = Eigen::Vector3i::Zero();

        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
        exist_second_height = false;
    }

    bool operator<(const GridNode & node) const
    {
      return (node.index(0) < index(0)) || ((node.index(0) == index(0)) &&(node.index(1) < index(1)));
    }

  GridNode(){};
  ~GridNode(){};
};


#endif
