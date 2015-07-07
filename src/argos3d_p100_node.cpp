/* \author Vojtech Spurny CTU in Prague*/

#include "../argos/include/bta.h"

/// PCL LIBRARIES
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>


#include "visualizer.h"
#include "time_measurement.h"
#include <stdio.h>
#include <time.h>

/**
 * Adjustable parameters
 */
// VoxelGrid
#define VOXEL_GRID_LEAF_SIZE 0.05                   // 5cm
// StatisticalOutlierRemoval filter
#define STAT_FILTER_MEAN_K 20                       // 20
#define STAT_FILTER_DEV_MUL_THRESH 1                // 1
// Euclidean Cluster Extraction
#define EUCL_CLUSTER_EXTRAC_TOLERANCE 0.15          // 0.15
#define EUCL_CLUSTER_EXTRAC_MIN_CLUSTER_SIZE 50     // 50



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ArgosVisualizer* visualizer;
std::vector<pcl::PointXYZ> obst_min, obst_max;


/**
 * Camera Driver Parameters
 */
BTA_Handle btaHandle;
BTA_Status status;
BTA_Frame *frame;

bool dataPublished;

/**
 *
 * @brief Initialize the camera and initial parameter values. Returns 1 if properly initialized.
 *
 */
int initialize(){
    /*
     * Camera Initialization
     */
    // If the camera is not connected at all, we will get an segmentation fault.
    BTA_Config config;
    status = BTAinitConfig(&config);
    if (status != BTA_StatusOk) {
        std::cout << "\t Could not connect: " << status << "\n";
        return 0;
    }
    config.frameMode = BTA_FrameModeXYZ;

    status = BTAopen(&config, &btaHandle);
    if (status != BTA_StatusOk) {
        std::cout << "\t Could not open: " << status << "\n";
        return 0;
    }

    BTA_DeviceInfo *deviceInfo;
    status = BTAgetDeviceInfo(btaHandle, &deviceInfo);
    if (status != BTA_StatusOk) {
        std::cout << "\t Could not read device info: " << status << "\n";
        return 0;
    }
    //printf("Device type: 0x%x\n", deviceInfo->deviceType);
    BTAfreeDeviceInfo(deviceInfo);

    dataPublished=true;
    visualizer = new ArgosVisualizer();
    return 1;
}


/**
 *
 * @brief Publish the data based on set up parameters.
 *
 */
bool publishData() {
    /*
     * Get frame from camera
     */
    status = BTAgetFrame(btaHandle, &frame,3000);  //3000;
    if (status != BTA_StatusOk) {
        std::cout << "\t Could transfer data: " << status << "\n";
        return 0;
    }

    /*
     * Obtain XYZcoordinates from 3Dcamera
     */
    float_t *xCoordinates, *yCoordinates, *zCoordinates;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    status = BTAgetXYZcoordinates(frame, (void **)&xCoordinates, (void **)&yCoordinates,
                                  (void **)&zCoordinates, &dataFormat, &unit, &xRes, &yRes);
    if (status != BTA_StatusOk) {
        std::cout << "\t Could get cartesian coordinates: " << status << "\n";
        return 0;
    }

    /*
     * Creating the pointcloud
     */
    PointCloud::Ptr cloud_raw (new PointCloud);
    cloud_raw->header.frame_id = "map";
    cloud_raw->height = 1;
    cloud_raw->width = xRes*yRes;

    for (size_t i = 0; i < xRes*yRes; ++i)	{
        pcl::PointXYZ temp_point;
        temp_point.x = xCoordinates[i];
        temp_point.y = yCoordinates[i];
        temp_point.z = zCoordinates[i];
        cloud_raw->points.push_back(temp_point);
    }
    /*
     * Downsampling a PointCloud using a VoxelGrid filter
     */
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_raw);
    sor.setLeafSize (VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE);
    sor.filter (*cloud_filtered);

    /*
     * Removing outliers using a StatisticalOutlierRemoval filter
     */
    PointCloud::Ptr cloud_filtered_without_outliers (new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
    sor_stat.setInputCloud (cloud_filtered);
    sor_stat.setMeanK (STAT_FILTER_MEAN_K);
    sor_stat.setStddevMulThresh (STAT_FILTER_DEV_MUL_THRESH);
    sor_stat.filter (*cloud_filtered_without_outliers);

    /*
     * Euclidean Cluster Extraction
     */
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered_without_outliers);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (EUCL_CLUSTER_EXTRAC_TOLERANCE);
    ec.setMinClusterSize (EUCL_CLUSTER_EXTRAC_MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize (cloud_filtered_without_outliers->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered_without_outliers);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vec_cluters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back (cloud_filtered_without_outliers->points[*pit]);
        }
        vec_cluters.push_back(cloud_cluster);
    }

    /*
     * ConvexHull for every cluster
     */
    /*
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vec_convex_hulls;
    for (int i = 0; i < vec_cluters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud (vec_cluters[i]);
        hull.reconstruct (*cloud);
        vec_convex_hulls.push_back(cloud);
    }
    */

    /*
     * Bounding box for every cluster
     */
    obst_min.clear();
    obst_max.clear();
    for (int i = 0; i < vec_cluters.size(); ++i) {
        obst_min.push_back(pcl::PointXYZ());
        obst_max.push_back(pcl::PointXYZ());
        pcl::getMinMax3D(*vec_cluters[i], obst_min[i], obst_max[i]);
    }

    /*
     * Visualization of convex hulls
     */
    visualizer->clearWindow();
    for (int i = 0; i < vec_cluters.size(); ++i) {
        //visualizer->addPointCloud(vec_cluters[i],i);
        //visualizer->addConvexMesh(vec_cluters[i],i);
        visualizer->addBox(obst_min[i],obst_max[i],i);
    }
    visualizer->refresh();


    status = BTAfreeFrame(&frame);
    if (status != BTA_StatusOk) {
        std::cout << "\t Could not free frame: " << status << "\n";
        return 0;
    }

    return 1;
}

/**
 *
 * @brief Main function
 *
 * @param [in] int
 * @param [in] char *
 *
 */
int main(int argc, char *argv[]) {
    std::cout << "---------------------------------------------------\n";
    std::cout << "|           Starting ARGOS3D - P100               |\n";
    std::cout << "---------------------------------------------------\n";
    std::cout << "- Initalizion of camera"<< "\n";
    //TimeMeasurement time;
    if(initialize()){
        std::cout << "- Camera initialized => Reading data"<< "\n";
        while (dataPublished)
        {
            //time.start();
            if(!publishData()) break;
            //time.end();
        }
    } else {
        std::cout << "- Cannot Initialize Camera :\n \t Check the parameters and try again!! \n";
        std::cout << "---------------------------------------------------\n";
        return 0;
    }
    std::cout << "---------------------------------------------------\n";
    BTAfreeFrame(&frame);
    BTAclose(&btaHandle);
    return 0;
}

