#include "../argos/include/bta.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/publisher.h>

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
#include <stdio.h>
#include <time.h>
#include <sstream>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const uint8_t c_r[] = {0,  0,  0,  0,255,255,255,255};
const uint8_t c_g[] = {0,  0,255,255,  0,  0,255,255};
const uint8_t c_b[] = {0,255,  0,255,  0,255,  0,255};
std::vector<pcl::PointIndices> cluster_indices;
ArgosVisualizer* visualizer;

/**
 * Camera Driver Parameters
 */
BTA_Handle btaHandle;
BTA_Status status;
BTA_Frame *frame;

/**
 * ROS Parameters
 */

bool dataPublished;

ros::Publisher pub_without_outlier;
ros::Publisher pub_non_filtered;
ros::Publisher pub_filtered;
ros::Publisher pub_cluster;
ros::Publisher pub_hull;

/**
 *
 * @brief This method prints help in command line if given --help option
 * or if there is any error in the options
 *
 */
int help() {
    std::cout << "\n Using help for argos3d_p100\n"
                 " You can set default configuration values for the camera with the following options: \n" << std::endl;
    std::cout << " Usage:\n rosrun argos3d_p100 argos3d_p100_node "<< std::endl
              << "\t-it *Integration_Time* \n\tIntegration time(in msec) for the sensor \n\t(min: 100 | max: 2700 | default: 1500) "<< std::endl
              << "\t-mf  *Modulation_Frequency* \n\tSet the modulation frequency(Hz) of the sensor \n\t(min: 5000000 | max: 30000000 | default: 30000000) "<< std::endl
              << "\n Example:" << std::endl
              << "rosrun argos3d_p100 argos3d_p100_node -it 1500 \n" << std::endl;
    exit(0);
} //print_help


/**
 *
 * @brief Initialize the camera and initial parameter values. Returns 1 if properly initialized.
 *
 * @param [in] int
 * @param [in] argv
 * @param [in] ros::NodeHandle
 *
 */
int initialize(int argc, char *argv[],ros::NodeHandle nh){
    /*
     * Camera Initialization
     */
    // If the camera is not connected at all, we will get an segmentation fault.
    BTA_Config config;
    status = BTAinitConfig(&config);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could not connect: " << status);
        return 0;
    }
    config.frameMode = BTA_FrameModeXYZ;//BTA_FrameModeXYZAmp; //BTA_FrameModeDistAmpFlags;
    //config.calibFileName = "xyz_calibration_00.bin";

    status = BTAopen(&config, &btaHandle);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could not open: " << status);
        return 0;
    }

    BTA_DeviceInfo *deviceInfo;
    status = BTAgetDeviceInfo(btaHandle, &deviceInfo);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could not read device info: " << status);
        return 0;
    }
    //printf("Device type: 0x%x\n", deviceInfo->deviceType);
    BTAfreeDeviceInfo(deviceInfo);
    /*
     * ROS Node Initialization
     */
    pub_non_filtered = nh.advertise<PointCloud> ("depth_non_filtered", 1);
    pub_filtered = nh.advertise<PointCloud> ("depth_filtered", 1);
    pub_without_outlier = nh.advertise<PointCloud> ("depth_filtered_without_outlier", 1);
    pub_cluster = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("depth_clusters", 1);
    pub_hull = nh.advertise<PointCloud> ("depth_hull", 1);
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
    visualizer->clearWindow();
    int j=0;
    std::ostringstream ss;

    /*
     * Get frame from camera
     */
    status = BTAgetFrame(btaHandle, &frame,3000);  //3000;
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could transfer data: " << status);
        return 0;
    }

    /*
     * Obtain PointClouds
     */
    float_t *xCoordinates, *yCoordinates, *zCoordinates;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    status = BTAgetXYZcoordinates(frame, (void **)&xCoordinates, (void **)&yCoordinates, (void **)&zCoordinates, &dataFormat, &unit, &xRes, &yRes);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could get cartesian coordinates: " << status);
        return 0;
    }

    /*
     * Obtain Amplitude Values
     */
    /*
    float_t *amplitudes;
    status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could get amplitude values: " << status);
        return 0;
    }
    */
    /*
     * Creating the pointcloud
     */
    PointCloud::Ptr msg_non_filtered (new PointCloud);
    msg_non_filtered->header.frame_id = "map";
    msg_non_filtered->height = 1;
    msg_non_filtered->width = xRes*yRes;

    for (size_t i = 0; i < xRes*yRes; ++i)	{
        pcl::PointXYZ temp_point;
        temp_point.x = xCoordinates[i];
        temp_point.y = yCoordinates[i];
        temp_point.z = zCoordinates[i];
        //temp_point.intensity = amplitudes[i] ;
        msg_non_filtered->points.push_back(temp_point);
    }
    /*
     * Filtering the PointCloud
     */
    PointCloud::Ptr msg_filtered (new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (msg_non_filtered);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);  //1cm
    sor.filter (*msg_filtered);

    /*
     * Filtering outlier
     */
    PointCloud::Ptr msg_filtered_without_outlier (new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
    sor_stat.setInputCloud (msg_filtered);
    sor_stat.setMeanK (20);
    sor_stat.setStddevMulThresh (1);
    sor_stat.filter (*msg_filtered_without_outlier);

    //printf("%ld, %ld\n",msg_filtered->points.size(),msg_filtered_without_outlier->points.size());
    //visualizer->addPointCloud(msg_filtered_without_outlier);

    /*
     * Publishing the messages
     */
    //pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    msg_non_filtered->header.stamp = frame->timeStamp;
    pub_non_filtered.publish (msg_non_filtered);

    msg_filtered->header.stamp = frame->timeStamp;
    pub_filtered.publish (msg_filtered);

    msg_filtered_without_outlier->header.stamp = frame->timeStamp;
    pub_without_outlier.publish (msg_filtered_without_outlier);

    ///
    // Create the segmentation object for the planar model and set all the parameters
    /*
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) msg_filtered_without_outlier->points.size ();
    printf("cl %ld \n",msg_filtered_without_outlier->points.size());
    while (msg_filtered_without_outlier->points.size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (msg_filtered_without_outlier);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (msg_filtered_without_outlier);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        if (cloud_plane->points.size()<100) break;
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        ss.str("");
        ss.clear();
        ss << j;
        visualizer->addConvexMesh(cloud_plane,ss.str());
        j++;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *msg_filtered_without_outlier = *cloud_f;
        //printf("%ld \n",msg_filtered_without_outlier->points.size());
    }
    //printf("msg %ld \n",msg_filtered_without_outlier->points.size());
    //pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(cloud_without_outliers,msg_filtered_without_outlier);
    */

    // Create CLUSTERS

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_cluster->points.resize(msg_filtered_without_outlier->size());
    for (size_t i = 0; i < msg_filtered_without_outlier->points.size(); i++) {
        cloud_cluster->points[i].x = msg_filtered_without_outlier->points[i].x;
        cloud_cluster->points[i].y = msg_filtered_without_outlier->points[i].y;
        cloud_cluster->points[i].z = msg_filtered_without_outlier->points[i].z;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    tree->setInputCloud (msg_filtered_without_outlier);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.15); // 10cm
    ec.setMinClusterSize (50); //50
    ec.setMaxClusterSize (msg_filtered_without_outlier->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (msg_filtered_without_outlier);
    //ec.PointIndicesPtr(cluster_indices);
    cluster_indices.clear();
    ec.extract (cluster_indices);

    //int j = 0;
    //std::ostringstream ss;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        ss.str("");
        ss.clear();
        cloud_projected->points.clear();
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_projected->points.push_back (msg_filtered_without_outlier->points[*pit]); //*
            //cloud_cluster->points[*pit].r = c_r[j];
            //cloud_cluster->points[*pit].g = c_g[j];
            //cloud_cluster->points[*pit].b = c_b[j];
        }
        ss << j;
        visualizer->addConvexMesh(cloud_projected,ss.str());
        j++;
        /*
        if (j>7) {
            printf("More than 7 clusters \n");
            break;
        }
        */
    }

    //cloud_cluster->header.stamp = frame->timeStamp;
    //cloud_cluster->header.frame_id = "map";
    //pub_cluster.publish (cloud_cluster);

    /*
    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PolygonMesh::Ptr model_polygon_mesh_;
    std::vector< pcl::Vertices> polygons;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull,polygons);

    std::cerr << "Convex hull has: " << cloud_hull->points.size ()
              << " data points." << std::endl;
    std::cerr << "\t: " << cloud_cluster->points.size ()
              << " data points." << std::endl;

    cloud_hull->header.stamp = frame->timeStamp;
    cloud_hull->header.frame_id = "map";
    pub_hull.publish (cloud_hull);
    */
    visualizer->refresh();

    status = BTAfreeFrame(&frame);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could not free frame: " << status);
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
    ROS_INFO("Starting argos3d_p100 ros...");
    ros::init (argc, argv, "argos3d_p100");
    ros::NodeHandle nh;

    if(initialize(argc, argv,nh)){
        ROS_INFO("Initalized Camera... Reading Data");
        ros::Rate loop_rate(200);
        while (nh.ok() && dataPublished)
        {
            if(!publishData())
                dataPublished=false;
            ros::spinOnce ();
            loop_rate.sleep ();
        }
    } else {
        ROS_WARN("Cannot Initialize Camera. Check the parameters and try again!!");
        return 0;
    }

    BTAfreeFrame(&frame);
    BTAclose(&btaHandle);
    return 0;
}

