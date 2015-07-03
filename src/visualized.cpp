#include "visualizer.h"
std::vector<std::string> names;

void ArgosVisualizer::addConvexMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::string &id)
{
    names.push_back(id);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    std::vector< pcl::Vertices> polygons;
    hull.setInputCloud (cloud);
    hull.reconstruct (*cloud_hull,polygons);

    //boost::shared_ptr<pcl::PolygonMesh> model_polygon_mesh_;
    if (!viewer->updatePolygonMesh<pcl::PointXYZ> (cloud_hull, polygons,id))
        viewer->addPolygonMesh<pcl::PointXYZ> (cloud_hull, polygons,id);
}

void ArgosVisualizer::addConcaveMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::string &id)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> hull;
    std::vector< pcl::Vertices> polygons;
    hull.setInputCloud (cloud);
    hull.setAlpha(0.5);
    hull.reconstruct (*cloud_hull,polygons);

    //boost::shared_ptr<pcl::PolygonMesh> model_polygon_mesh_;
    if (!viewer->updatePolygonMesh<pcl::PointXYZ> (cloud_hull, polygons,id))
        viewer->addPolygonMesh<pcl::PointXYZ> (cloud_hull, polygons,id);
}

void ArgosVisualizer::addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    if (!viewer->updatePointCloud<pcl::PointXYZ> (cloud))
        viewer->addPointCloud<pcl::PointXYZ> (cloud);
    //viewer->addPointCloud<pcl::PointXYZ> (cloud);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
}

ArgosVisualizer::ArgosVisualizer(){
    viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition(0,0,0,0,0,3,0,-1,0);
    viewer->addCoordinateSystem (0.1);
    //viewer->initCameraParameters ();
    refresh();
}

void ArgosVisualizer::refresh(){
    if (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}

void ArgosVisualizer::clearWindow(){
    for (int i = 0; i < names.size(); ++i) {
        viewer->removePolygonMesh(names[i]);
    }
    names.clear();
    viewer->removeAllPointClouds();
    //refresh();
}
