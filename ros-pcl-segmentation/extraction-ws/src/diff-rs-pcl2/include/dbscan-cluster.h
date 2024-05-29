#ifndef _DBSCAN_CLUSTER_H_
#define _DBSCAN_CLUSTER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <deque>
#include <numeric>

class dbscan {
public:
    void initialize(ros::NodeHandle& nh, const std::string& param_prefix);

    std_msgs::ColorRGBA assignColor(int id);
    double calculateMovingAverage();
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void clusterAndVisualize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void analyzeAndPrintObjectInfo(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster);
   
    void addFilledConvexHullMarkersTo(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster,
            visualization_msgs::MarkerArray& marker_array, int cluster_id, const std::string& frame_id);
    void addBoundingBoxMarker(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster,
            visualization_msgs::MarkerArray& marker_array, int cluster_id, const std::string& frame_id);
    void clearPreviousMarkers();
    void timerCallback(const ros::TimerEvent& event);

private:
    ros::Publisher marker_pub;
    ros::Publisher safety_pub;
    ros::Subscriber cloud_sub;
    ros::Timer safety_timer;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

    double line_thickness;
    double min_safety_dist;
    size_t window_size=10;

    std::deque<double> distance_window;

    void checkAndPublishSafety(const double& min_distance);
    bool has_received_msg = false; // track message reception
};

#endif // _DBSCAN_CLUSTER_H_
