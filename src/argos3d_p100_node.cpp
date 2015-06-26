#include "../argos/include/bta.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <stdio.h>
#include <time.h>
#include <sstream>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/**
 * Global Parameter Declarations
 */
float AmplitudeThreshold = 100;


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
//ros::Publisher pub_non_filtered;
ros::Publisher pub_filtered;

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
    config.frameMode = BTA_FrameModeXYZAmp;//BTA_FrameModeXYZAmp; //BTA_FrameModeDistAmpFlags;
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
    //pub_non_filtered = nh.advertise<PointCloud> ("depth_non_filtered", 1);
    pub_filtered = nh.advertise<PointCloud> ("depth_filtered", 1);
    pub_without_outlier = nh.advertise<PointCloud> ("depth_filtered_without_outlier", 1);
    dataPublished=true;
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
    status = BTAgetFrame(btaHandle, &frame,1000);  //3000;
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
    float_t *amplitudes;
    status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
    if (status != BTA_StatusOk) {
        ROS_ERROR_STREAM("Could get amplitude values: " << status);
        return 0;
    }

    /*
     * Creating the pointcloud
     */
    PointCloud::Ptr msg_non_filtered (new PointCloud);
    msg_non_filtered->header.frame_id = "tf_argos3d";
    msg_non_filtered->height = 1;
    msg_non_filtered->width = xRes*yRes;

    for (size_t i = 0; i < xRes*yRes; ++i)	{
        pcl::PointXYZI temp_point;
        temp_point.x = xCoordinates[i];
        temp_point.y = yCoordinates[i];
        temp_point.z = zCoordinates[i];
        temp_point.intensity = amplitudes[i] ;
        msg_non_filtered->points.push_back(temp_point);
    }
    /*
     * Filtering the PointCloud
     */
    PointCloud::Ptr msg_filtered (new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (msg_non_filtered);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*msg_filtered);

    /*
     * Filtering outlier
     */
    PointCloud::Ptr msg_filtered_without_outlier (new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_stat;
    sor_stat.setInputCloud (msg_filtered);
    sor_stat.setMeanK (50);
    sor_stat.setStddevMulThresh (1);
    sor_stat.filter (*msg_filtered_without_outlier);

    printf("%d, %d\n",msg_filtered->points.size(),msg_filtered_without_outlier->points.size());
    /*
     * Publishing the messages
     */
    //pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    //msg_non_filtered->header.stamp = frame->timeStamp;
    //pub_non_filtered.publish (msg_non_filtered);

    msg_filtered->header.stamp = frame->timeStamp;
    pub_filtered.publish (msg_filtered);

    msg_filtered_without_outlier->header.stamp = frame->timeStamp;
    pub_without_outlier.publish (msg_filtered_without_outlier);

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

