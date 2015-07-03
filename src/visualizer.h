#ifndef ARGOS_VISUALIZER_H
#define  ARGOS_VISUALIZER_H

#include <vector>
#include <fstream>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/concave_hull.h>

class ArgosVisualizer
{
private:
    pcl::visualization::PCLVisualizer* viewer;
public:
    ArgosVisualizer();
    void refresh();
    void clearWindow();
    void addConvexMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const size_t &id);
    void addConcaveMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const size_t &id);
    void addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    virtual ~ArgosVisualizer() {  }
};
#endif
