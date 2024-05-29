/**
 *
 * @file dbscan-cluster.cpp
 * @brief
 * Implementation of the DBSCAN for clustering analysis of sensor data, along with visualization and safety checks. 
 * 
 * Functionalities :
 * - Initializing DBSCAN parameters
 * - Calculating moving average of the minimum distance
 * - Clearing previous visualization markers
 * - Adding filled convex hull markers to visualization marker array
 * - Clustering sensor data and visualizing it
 * - Analyzing object information and printing
 * - Adding bounding box markers
 * - Listening to callbacks of sensor data
 * - Periodically checking sensor data for safety assurance
 * 
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author pomelo925 (yoseph.huang@gmail.com)
 * @version 2.1
 * @date 2024-03-27
 *
 */

#include "dbscan-cluster.h"

std_msgs::ColorRGBA dbscan::assignColor(int id){
    std_msgs::ColorRGBA color;
    color.a = 1.0; // Full opacity
    switch (id % 6) {
        case 0: color.r = 1.0; color.g = 0.0; color.b = 0.0; break; // Red
        case 1: color.r = 0.0; color.g = 1.0; color.b = 0.0; break; // Green
        case 2: color.r = 0.0; color.g = 0.0; color.b = 1.0; break; // Blue
        case 3: color.r = 1.0; color.g = 1.0; color.b = 0.0; break; // Yellow
        case 4: color.r = 0.0; color.g = 1.0; color.b = 1.0; break; // Cyan
        case 5: color.r = 1.0; color.g = 0.0; color.b = 1.0; break; // Magenta
    }
    return color;
}

void dbscan::initialize(ros::NodeHandle& nh, const std::string& param_prefix) {
    // Update topic subscriptions and publications to use dynamic names if needed
    std::string visualization_topic, safety_topic, cloud_topic;

    visualization_topic = "diff_" + param_prefix + "/visualization_marker_array";
    safety_topic = "diff_" + param_prefix + "/ladybug/safe";
    cloud_topic = "diff_cam_" + param_prefix + "/differential/ladybug/cube";

    // Fetch DBSCAN parameters
    double cluster_tolerance, obstacle_existence_checking_period;
    int min_cluster_size, max_cluster_size;
    nh.getParam("diff_" + param_prefix + "/cluster_tolerance", cluster_tolerance);
    nh.getParam("diff_" + param_prefix + "/min_cluster_size", min_cluster_size);
    nh.getParam("diff_" + param_prefix + "/max_cluster_size", max_cluster_size);

    // Fetch additional parameters as needed
    nh.getParam("diff_" + param_prefix + "/line_thickness", line_thickness);
    nh.getParam("diff_" + param_prefix + "/min_safety_dist", min_safety_dist);
    nh.getParam("diff_" + param_prefix + "/obstacle_existence_checking_period", obstacle_existence_checking_period);

    
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(visualization_topic, 1);
    safety_pub = nh.advertise<std_msgs::Bool>(safety_topic, 1);
    cloud_sub = nh.subscribe(cloud_topic, 1, &dbscan::cloudCallback, this);

    safety_timer = nh.createTimer(ros::Duration(obstacle_existence_checking_period), &dbscan::timerCallback, this);


    // Configure the DBSCAN clustering
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
}

double dbscan::calculateMovingAverage() {
    if (distance_window.empty()) return 0.0;
    const double sum = std::accumulate(distance_window.begin(), distance_window.end(), 0.0);
    return sum / distance_window.size();
}

void dbscan::clearPreviousMarkers() {
    visualization_msgs::MarkerArray deleteMarkers;
    visualization_msgs::Marker marker;
    
    marker.action = visualization_msgs::Marker::DELETEALL;
    deleteMarkers.markers.push_back(marker);
    marker_pub.publish(deleteMarkers);
}

void dbscan::checkAndPublishSafety(const double& min_distance) {
    // Add the latest distance to the window
    distance_window.push_back(min_distance);
    
    // Maintain the window size
    if (distance_window.size() > window_size) distance_window.pop_front();
    
    // Apply the filter
    double filtered_distance = calculateMovingAverage();
    
    // Check the filtered distance and publish to safety topic
    std_msgs::Bool is_safe_msg;
    is_safe_msg.data = filtered_distance >= min_safety_dist; // True if filtered distance is safe
    safety_pub.publish(is_safe_msg);
}

void dbscan::addFilledConvexHullMarkersTo(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster, 
    visualization_msgs::MarkerArray& marker_array, int cluster_id, const std::string& frame_id){
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud(cloud_cluster);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::Vertices> hullPolygons;
    chull.reconstruct(*cloud_hull, hullPolygons);
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "hull";
    marker.id = cluster_id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color = assignColor(cluster_id);

    // Assuming the hullPolygons contains a single polygon for the convex hull
    for (const auto& poly : hullPolygons) {
        for (size_t i = 1; i <= poly.vertices.size() - 2; ++i) {
            // Add the first vertex of the triangle
            geometry_msgs::Point p;
            p.x = cloud_hull->points[poly.vertices[0]].x;
            p.y = cloud_hull->points[poly.vertices[0]].y;
            p.z = cloud_hull->points[poly.vertices[0]].z;
            marker.points.push_back(p);

            // Add the second vertex of the triangle
            p.x = cloud_hull->points[poly.vertices[i]].x;
            p.y = cloud_hull->points[poly.vertices[i]].y;
            p.z = cloud_hull->points[poly.vertices[i]].z;
            marker.points.push_back(p);
            
            // Add the third vertex of the triangle
            p.x = cloud_hull->points[poly.vertices[i+1]].x;
            p.y = cloud_hull->points[poly.vertices[i+1]].y;
            p.z = cloud_hull->points[poly.vertices[i+1]].z;
            marker.points.push_back(p);
        }
    }
    marker_array.markers.push_back(marker);
}

void dbscan::clusterAndVisualize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Check for empty cloud and clear previous markers
    if (cloud->empty()) {
        // ROS_WARN("Received an empty point cloud.");
        clearPreviousMarkers();
        return;
    }

    // Set up search tree and cluster extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Clear previous markers before adding new ones
    clearPreviousMarkers();

    // Prepare the marker array for visualization
    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < cluster_indices.size(); ++i) {
        // Extract the cluster and create a point cloud from the indices
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : cluster_indices[i].indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        // Analyze the cluster and print information
        analyzeAndPrintObjectInfo(cloud_cluster);
        // Add bounding box markers for visualization
        addBoundingBoxMarker(cloud_cluster, marker_array, i, cloud_msg->header.frame_id);
    }

    // Publish the markers to visualize in RViz
    if (!marker_array.markers.empty()) {
        marker_pub.publish(marker_array);
    }
}

void dbscan::analyzeAndPrintObjectInfo(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster) {
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

    // Calculate the 8 points of the bounding box
    std::vector<Eigen::Vector4f> bounding_box_points;
    bounding_box_points.push_back(Eigen::Vector4f(min_pt[0], min_pt[1], min_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(min_pt[0], min_pt[1], max_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(min_pt[0], max_pt[1], min_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(min_pt[0], max_pt[1], max_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(max_pt[0], min_pt[1], min_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(max_pt[0], min_pt[1], max_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(max_pt[0], max_pt[1], min_pt[2], 1));
    bounding_box_points.push_back(Eigen::Vector4f(max_pt[0], max_pt[1], max_pt[2], 1));

    // Calculate the volume
    double volume = (max_pt[0] - min_pt[0]) * (max_pt[1] - min_pt[1]) * (max_pt[2] - min_pt[2]);

    // Find the nearest point to the camera
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector4f nearest_point;
    for (const auto& point : bounding_box_points) {
        double distance = point.head<3>().norm();  // Assuming camera at (0,0,0)
        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = point;
        }
    }
    // Check the nearest point distance and publish to safety topic
    std_msgs::Bool is_safe_msg;
    is_safe_msg.data = (min_distance >= min_safety_dist); 
    safety_pub.publish(is_safe_msg);

    // Print the information
    // std::cout << "\033[2J\033[1;1H"; 
    // std::cout << "Object Information:" << std::endl;
    // std::cout << "Bounding Box Points:" << std::endl;
    // for (const auto& point : bounding_box_points) {
    //     std::cout << "(" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    // }
    // std::cout << "Volume: " << volume << " cubic meters" << std::endl;
    // std::cout << "Nearest Point to Camera: (" << nearest_point[0] << ", " << nearest_point[1] << ", " << nearest_point[2] << ")" << std::endl;
    
    std::cout << "\033[A\033[K";
    std::cout << "Distance to Nearest Point: " << min_distance << " meters" << std::endl;
}

void dbscan::addBoundingBoxMarker(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster,
                          visualization_msgs::MarkerArray& marker_array, 
                          int cluster_id, const std::string& frame_id) {
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = cluster_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = line_thickness;
    marker.color = assignColor(cluster_id);

    // Define the 8 corners of the bounding box
    std::vector<Eigen::Vector3f> corners;
    corners.push_back(Eigen::Vector3f(min_pt[0], min_pt[1], min_pt[2]));
    corners.push_back(Eigen::Vector3f(min_pt[0], min_pt[1], max_pt[2]));
    corners.push_back(Eigen::Vector3f(min_pt[0], max_pt[1], min_pt[2]));
    corners.push_back(Eigen::Vector3f(min_pt[0], max_pt[1], max_pt[2]));
    corners.push_back(Eigen::Vector3f(max_pt[0], min_pt[1], min_pt[2]));
    corners.push_back(Eigen::Vector3f(max_pt[0], min_pt[1], max_pt[2]));
    corners.push_back(Eigen::Vector3f(max_pt[0], max_pt[1], min_pt[2]));
    corners.push_back(Eigen::Vector3f(max_pt[0], max_pt[1], max_pt[2]));

    // Lines of the bounding box
    std::vector<std::pair<int, int>> lines = {
        {0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
        {7, 6}, {6, 4}, {0, 4}, {1, 5}, {3, 7}, {2, 6}
    };
    for (auto line : lines) {
        geometry_msgs::Point p1;
        p1.x = corners[line.first][0];
        p1.y = corners[line.first][1];
        p1.z = corners[line.first][2];
        marker.points.push_back(p1);
        geometry_msgs::Point p2;
        p2.x = corners[line.second][0];
        p2.y = corners[line.second][1];
        p2.z = corners[line.second][2];
        marker.points.push_back(p2);
    }
    marker_array.markers.push_back(marker);
}

void dbscan::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    has_received_msg = true;
    clusterAndVisualize(cloud_msg);
}


void dbscan::timerCallback(const ros::TimerEvent& event) {
    if (!has_received_msg) {
        // If no message received, assume safe and publish TRUE
        std_msgs::Bool is_safe_msg;
        is_safe_msg.data = true;
        safety_pub.publish(is_safe_msg);
    }
    // Reset the flag for the next interval
    has_received_msg = false;
}