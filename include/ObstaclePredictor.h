/*
 * Name: ObstaclePredictor.h
 * Author: Pranav Chintalapudi
 * Organization: ANSCER Robotics
 * Date: 2026-01-20
 * Version: 1.0
 * 
 * Description: Dynamic obstacle detection and prediction with adaptive noise filtering.
 *              Combines LiDAR and RGB-D camera for robust tracking of moving obstacles
 *              in the presence of robot ego-motion.
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <unordered_set>

enum class ObstacleType { UNKNOWN, STATIC, DYNAMIC };

struct Obstacle
{
  int id;
  double x, y, vx, vy;
  ros::Time lastSeen;
  int missedFrames, age;
  double totalMotion;
  bool isParallelMotion;
  ObstacleType type;
  bool has3dData;
  double heightEstimate;
  int consecutiveMovingFrames;
  double initialX, initialY, maxDeviation;
  double motionConsistencyScore;
  int consecutiveRejections;
  double lastValidVx, lastValidVy;

  Obstacle(int id_, double x_, double y_);
};

struct Detection
{
  Eigen::Vector2d position;
  bool has3d;
  double height;

  Detection(const Eigen::Vector2d& pos, bool has3d_ = false, double h = 0.0);
};

class ObstaclePredictor
{
public:
  ObstaclePredictor();

private:
  // ROS handles
  ros::NodeHandle m_nh, m_pnh;
  ros::Subscriber m_subScan, m_subCam, m_subOdom;
  ros::Publisher m_pubObstacles, m_pubPredictions;

  // TF and sensor data
  std::unique_ptr<tf::TransformListener> p_tfListener;
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_lastCamCloud;
  ros::Time m_lastCamStamp, m_lastStamp;
  std::string m_lastCamFrame;

  // Tracking state
  std::vector<Obstacle> m_obstacles;
  std::unordered_set<int> m_lastPublishedIds;
  int m_nextId;

  // Robot state
  double m_robotVx, m_robotVy, m_robotOmega;
  bool m_isRobotMoving, m_isRobotRotating;

  // Core parameters
  double m_clusterTol, m_assocThresh, m_predHorizon, m_predDt;
  double m_minHeight, m_maxHeight, m_mergeThresh;
  double m_motionLinearThresh, m_motionAngularThresh;
  int m_maxMissed, m_minAgeForClassification;
  bool m_useCameraDuringMotion;

  // Noise model parameters
  double m_baseNoiseGate, m_rotationNoiseScale, m_linearNoiseScale;
  double m_minRotationNoise, m_farObjectThreshold, m_farObjectNoise;
  double m_relativeVelThresh;

  // Velocity validation parameters
  double m_maxPlausibleVelocity;
  double m_maxAccelerationStationary;
  double m_maxAccelerationMoving;
  double m_maxAccelerationRotating;
  double m_velocityRecoveryFactor;
  int m_maxConsecutiveRejections;
  bool m_enableAccelerationCheck;
  double m_occlusionDisplacementThreshold;
  double m_occlusionTimeThreshold;

  // Classification parameters - dual mode
  double m_minDynamicSpeedStationary, m_minDynamicSpeedMoving;
  double m_unknownToDynamicSpeedStationary, m_unknownToDynamicSpeedMoving;
  double m_staticToUnknownSpeedStationary, m_staticToUnknownSpeedMoving;

  // Parallel motion parameters
  double m_parallelMotionAlignmentThreshold;
  double m_parallelMotionMinSpeed;
  double m_parallelMotionMinRobotSpeed;

  /**
   * @brief Callback for laser scan data
   * @param msg LaserScan message
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief Callback for camera point cloud data
   * @param msg PointCloud2 message
   */
  void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
   * @brief Callback for robot odometry
   * @param msg Odometry message
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief Process point cloud into obstacle detections
   * @param cloud Input point cloud
   * @param stamp Timestamp
   * @param sourceFrame Frame ID of cloud
   * @param is3dSensor Whether this is a 3D sensor
   * @return Vector of detections
   */
  std::vector<Detection> processCloudToDetections(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time& stamp,
      const std::string& sourceFrame, bool is3dSensor);

  /**
   * @brief Get camera detections synchronized with scan
   * @param stamp Current scan timestamp
   * @return Vector of camera detections
   */
  std::vector<Detection> getCameraDetections(const ros::Time& stamp);

  /**
   * @brief Merge LiDAR and camera detections
   * @param lidarDets LiDAR detections
   * @param cameraDets Camera detections
   * @return Fused detections
   */
  std::vector<Detection> mergeDetections(
      const std::vector<Detection>& lidarDets, const std::vector<Detection>& cameraDets);

  /**
   * @brief Extract cluster centroids from point cloud
   * @param cloud Input cloud
   * @param heights Output vector of cluster heights (optional)
   * @return Vector of 2D centroids
   */
  std::vector<Eigen::Vector2d> extractCentroids(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<double>* heights = nullptr);

  /**
   * @brief Update all obstacle tracks with new detections
   * @param dets Current detections
   * @param stamp Current timestamp
   */
  void updateTracks(const std::vector<Detection>& dets, const ros::Time& stamp);

  /**
   * @brief Update single obstacle track
   * @param obs Obstacle to update
   * @param det Detection to associate
   * @param stamp Current timestamp
   * @param robotPos Robot position in odom frame
   * @param haveTf Whether TF is available
   */
  void updateObstacle(Obstacle& obs, const Detection& det, const ros::Time& stamp,
                      const Eigen::Vector2d& robotPos, bool haveTf);

  /**
   * @brief Classify obstacle as static/dynamic/unknown
   * @param obs Obstacle to classify
   */
  void classifyObstacle(Obstacle& obs);

  /**
   * @brief Publish visualization markers
   */
  void publishMarkers();

  /**
   * @brief Calculate adaptive noise gate based on robot motion
   * @param obs Obstacle being evaluated
   * @param robotPos Robot position
   * @param haveTf Whether TF is available
   * @return Noise gate threshold in meters
   */
  double calculateNoiseGate(const Obstacle& obs, const Eigen::Vector2d& robotPos, 
                           bool haveTf) const;

  /**
   * @brief Check if detection likely from occlusion/re-acquisition
   * @param obs Obstacle track
   * @param displacement Distance moved since last update
   * @param dt Time delta
   * @return True if likely occlusion event
   */
  bool isLikelyOcclusion(const Obstacle& obs, double displacement, double dt) const;
};
