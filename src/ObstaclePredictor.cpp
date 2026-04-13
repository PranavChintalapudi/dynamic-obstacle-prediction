/*
 * Name: ObstaclePredictor.cpp
 * Author: Pranav Chintalapudi
 * Organization: ANSCER Robotics
 * Date: 2026-01-20
 * Version: 1.0
 * Description: Dynamic obstacle detection and prediction with adaptive noise filtering
 */

#include "dynamic_obstacle_prediction/ObstaclePredictor.h"

// ============================================================================
// Struct Implementations
// ============================================================================

Obstacle::Obstacle(int id_, double x_, double y_)
  : id(id_), x(x_), y(y_), vx(0.0), vy(0.0), lastSeen(ros::Time::now()),
    missedFrames(0), age(0), totalMotion(0.0), isParallelMotion(false),
    type(ObstacleType::UNKNOWN), has3dData(false), heightEstimate(0.0),
    consecutiveMovingFrames(0), initialX(x_), initialY(y_), maxDeviation(0.0),
    motionConsistencyScore(0.0), consecutiveRejections(0),
    lastValidVx(0.0), lastValidVy(0.0) {}

Detection::Detection(const Eigen::Vector2d& pos, bool has3d_, double h)
  : position(pos), has3d(has3d_), height(h) {}

// ============================================================================
// Constructor
// ============================================================================

ObstaclePredictor::ObstaclePredictor()
  : m_pnh("~"), m_nextId(0), m_robotVx(0.0), m_robotVy(0.0), m_robotOmega(0.0),
    m_isRobotMoving(false), m_isRobotRotating(false)
{
  // Load clustering & detection parameters
  m_pnh.param("cluster_tolerance", m_clusterTol, 0.35);
  m_pnh.param("association_threshold", m_assocThresh, 1.0);
  m_pnh.param("max_missed_frames", m_maxMissed, 10);
  m_pnh.param("prediction_horizon", m_predHorizon, 2.5);
  m_pnh.param("prediction_dt", m_predDt, 0.2);
  m_pnh.param("min_obstacle_height", m_minHeight, 0.05);
  m_pnh.param("max_obstacle_height", m_maxHeight, 2.0);
  m_pnh.param("detection_merge_threshold", m_mergeThresh, 0.45);
  
  // Robot motion detection
  m_pnh.param("motion_linear_threshold", m_motionLinearThresh, 0.05);
  m_pnh.param("motion_angular_threshold", m_motionAngularThresh, 0.08);
  m_pnh.param("use_camera_during_motion", m_useCameraDuringMotion, false);
  
  // Noise model parameters
  m_pnh.param("base_noise_gate", m_baseNoiseGate, 0.08);
  m_pnh.param("rotation_noise_scale", m_rotationNoiseScale, 0.40);
  m_pnh.param("linear_noise_scale", m_linearNoiseScale, 0.20);
  m_pnh.param("min_rotation_noise", m_minRotationNoise, 0.15);
  m_pnh.param("far_object_threshold", m_farObjectThreshold, 2.0);
  m_pnh.param("far_object_noise", m_farObjectNoise, 0.25);
  m_pnh.param("relative_velocity_threshold", m_relativeVelThresh, 0.12);
  
  // Velocity validation parameters
  m_pnh.param("max_plausible_velocity", m_maxPlausibleVelocity, 12.0);
  m_pnh.param("max_acceleration_stationary", m_maxAccelerationStationary, 3.0);
  m_pnh.param("max_acceleration_moving", m_maxAccelerationMoving, 8.0);
  m_pnh.param("max_acceleration_rotating", m_maxAccelerationRotating, 20.0);
  m_pnh.param("velocity_recovery_factor", m_velocityRecoveryFactor, 0.75);
  m_pnh.param("max_consecutive_rejections", m_maxConsecutiveRejections, 4);
  m_pnh.param("enable_acceleration_check", m_enableAccelerationCheck, true);
  m_pnh.param("occlusion_displacement_threshold", m_occlusionDisplacementThreshold, 2.0);
  m_pnh.param("occlusion_time_threshold", m_occlusionTimeThreshold, 1.0);
  
  // Dual-mode classification thresholds
  m_pnh.param("min_dynamic_speed_stationary", m_minDynamicSpeedStationary, 0.25);
  m_pnh.param("min_dynamic_speed_moving", m_minDynamicSpeedMoving, 0.50);
  m_pnh.param("unknown_to_dynamic_speed_stationary", m_unknownToDynamicSpeedStationary, 0.30);
  m_pnh.param("unknown_to_dynamic_speed_moving", m_unknownToDynamicSpeedMoving, 0.60);
  m_pnh.param("static_to_unknown_speed_stationary", m_staticToUnknownSpeedStationary, 0.35);
  m_pnh.param("static_to_unknown_speed_moving", m_staticToUnknownSpeedMoving, 0.70);
  m_pnh.param("min_age_for_classification", m_minAgeForClassification, 10);
  
  // Parallel motion detection
  m_pnh.param("parallel_motion_alignment_threshold", m_parallelMotionAlignmentThreshold, 0.85);
  m_pnh.param("parallel_motion_min_speed", m_parallelMotionMinSpeed, 0.25);
  m_pnh.param("parallel_motion_min_robot_speed", m_parallelMotionMinRobotSpeed, 0.08);

  // Setup ROS communication
  m_subScan = m_nh.subscribe("/scan", 1, &ObstaclePredictor::scanCallback, this);
  m_subCam = m_nh.subscribe("camera/depth/points", 1, &ObstaclePredictor::cameraCallback, this);
  m_subOdom = m_nh.subscribe("/odom", 1, &ObstaclePredictor::odomCallback, this);
  m_pubObstacles = m_nh.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles_markers", 1);
  m_pubPredictions = m_nh.advertise<visualization_msgs::MarkerArray>("/predicted_obstacle_paths", 1);

  p_tfListener.reset(new tf::TransformListener);
  p_lastCamCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  if (ros::Time::isSimTime()) {
    ROS_WARN("[ObstaclePredictor] Waiting for sim time...");
    ros::Time::waitForValid();
  }

  ROS_INFO("[ObstaclePredictor] Initialized with adaptive noise filtering");
}

// ============================================================================
// Callbacks
// ============================================================================

void ObstaclePredictor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  m_robotVx = msg->twist.twist.linear.x;
  m_robotVy = msg->twist.twist.linear.y;
  m_robotOmega = msg->twist.twist.angular.z;
  
  double linearSpeed = std::hypot(m_robotVx, m_robotVy);
  m_isRobotMoving = (linearSpeed > m_motionLinearThresh);
  m_isRobotRotating = (std::fabs(m_robotOmega) > m_motionAngularThresh);
}

void ObstaclePredictor::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, *p_lastCamCloud);
  m_lastCamStamp = msg->header.stamp;
  m_lastCamFrame = msg->header.frame_id;
}

void ObstaclePredictor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  m_lastStamp = msg->header.stamp;

  // Convert scan to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
    
    float angle = msg->angle_min + i * msg->angle_increment;
    lidarCloud->push_back(pcl::PointXYZ(r * std::cos(angle), r * std::sin(angle), 0.15f));
  }

  // Process detections
  std::vector<Detection> lidarDets = processCloudToDetections(lidarCloud, msg->header.stamp, 
                                                               "base_link", false);
  
  std::vector<Detection> cameraDets;
  bool useCam = m_useCameraDuringMotion || (!m_isRobotMoving && !m_isRobotRotating);
  if (useCam && !p_lastCamCloud->empty() && 
      std::fabs((msg->header.stamp - m_lastCamStamp).toSec()) < 0.3) {
    cameraDets = getCameraDetections(msg->header.stamp);
  }
  
  std::vector<Detection> fusedDets = mergeDetections(lidarDets, cameraDets);
  
  ROS_INFO_THROTTLE(2.0, "[Fusion] %lu detections (%lu LiDAR + %lu camera) | Robot: %s",
                    fusedDets.size(), lidarDets.size(), cameraDets.size(),
                    (m_isRobotMoving || m_isRobotRotating) ? "MOVING" : "stationary");

  updateTracks(fusedDets, msg->header.stamp);
  publishMarkers();
}

// ============================================================================
// Detection Processing
// ============================================================================

std::vector<Detection> ObstaclePredictor::getCameraDetections(const ros::Time& stamp)
{
  std::vector<Detection> detections;
  tf::StampedTransform tfCamBase;
  
  try {
    p_tfListener->lookupTransform("base_link", m_lastCamFrame, ros::Time(0), tfCamBase);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(5.0, "[Camera] TF failed: %s", ex.what());
    return detections;
  }

  // Transform camera cloud to base_link and filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr camBase(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& p : p_lastCamCloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.z < 0.3 || p.z > 5.0) continue;

    tf::Vector3 vBase = tfCamBase * tf::Vector3(p.x, p.y, p.z);
    double dist2d = std::hypot(vBase.x(), vBase.y());
    if (dist2d >= 0.3 && dist2d <= 5.0 && vBase.z() >= 0.0 && vBase.z() <= 2.0) {
      camBase->push_back(pcl::PointXYZ(vBase.x(), vBase.y(), vBase.z()));
    }
  }

  return camBase->empty() ? detections : processCloudToDetections(camBase, stamp, "base_link", true);
}

std::vector<Detection> ObstaclePredictor::processCloudToDetections(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time& stamp,
    const std::string& sourceFrame, bool is3dSensor)
{
  std::vector<Detection> detections;
  if (cloud->empty()) return detections;

  // Transform to odom frame
  tf::StampedTransform tfOdomBase;
  try {
    p_tfListener->lookupTransform("odom", sourceFrame, ros::Time(0), tfOdomBase);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(5.0, "[Processing] TF failed: %s", ex.what());
    return detections;
  }

  // Transform and filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr odomCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& p : cloud->points) {
    tf::Vector3 v = tfOdomBase * tf::Vector3(p.x, p.y, p.z);
    if (v.z() >= m_minHeight && v.z() <= m_maxHeight) {
      double dist = std::hypot(v.x(), v.y());
      if (dist >= 0.25 && dist <= 8.0) {
        odomCloud->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
      }
    }
  }

  if (odomCloud->empty()) return detections;

  // Downsample and remove ground
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(0.08f, 0.08f, 0.08f);
  voxel.setInputCloud(odomCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxel.filter(*filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr nonGround(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pt : filtered->points) {
    if (pt.z > 0.05) nonGround->push_back(pt);
  }

  if (nonGround->size() < 5) return detections;

  // Extract clusters
  std::vector<double> heights;
  std::vector<Eigen::Vector2d> centroids = extractCentroids(nonGround, &heights);
  
  for (size_t i = 0; i < centroids.size(); ++i) {
    detections.emplace_back(centroids[i], is3dSensor, heights[i]);
  }

  return detections;
}

std::vector<Detection> ObstaclePredictor::mergeDetections(
    const std::vector<Detection>& lidarDets, const std::vector<Detection>& cameraDets)
{
  std::vector<Detection> merged;
  std::vector<bool> matched(cameraDets.size(), false);

  for (const auto& ld : lidarDets) {
    Detection best = ld;
    double bestDist = m_mergeThresh;
    int bestIdx = -1;

    for (size_t i = 0; i < cameraDets.size(); ++i) {
      if (matched[i]) continue;
      double dist = (ld.position - cameraDets[i].position).norm();
      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = i;
      }
    }
    
    if (bestIdx >= 0) {
      best.position = (ld.position + cameraDets[bestIdx].position) / 2.0;
      best.has3d = true;
      best.height = cameraDets[bestIdx].height;
      matched[bestIdx] = true;
    }
    merged.push_back(best);
  }

  for (size_t i = 0; i < cameraDets.size(); ++i) {
    if (!matched[i]) merged.push_back(cameraDets[i]);
  }

  return merged;
}

std::vector<Eigen::Vector2d> ObstaclePredictor::extractCentroids(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<double>* heights)
{
  std::vector<Eigen::Vector2d> centroids;
  if (heights) heights->clear();
  if (cloud->size() < 5) return centroids;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(m_clusterTol);
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);

  for (const auto& cluster : clusters) {
    Eigen::Vector2d mean(0, 0);
    double heightSum = 0.0;
    
    for (int idx : cluster.indices) {
      mean.x() += cloud->points[idx].x;
      mean.y() += cloud->points[idx].y;
      heightSum += cloud->points[idx].z;
    }
    
    mean /= cluster.indices.size();
    double avgHeight = heightSum / cluster.indices.size();

    if (mean.norm() >= 0.4) {
      centroids.push_back(mean);
      if (heights) heights->push_back(avgHeight);
    }
  }

  return centroids;
}

// ============================================================================
// Tracking
// ============================================================================

void ObstaclePredictor::updateTracks(const std::vector<Detection>& dets, const ros::Time& stamp)
{
  for (auto& obs : m_obstacles) obs.missedFrames++;

  // Get robot pose
  tf::StampedTransform tfOdomBase;
  Eigen::Vector2d robotPos(0, 0);
  bool haveTf = false;
  
  try {
    p_tfListener->lookupTransform("odom", "base_link", ros::Time(0), tfOdomBase);
    robotPos.x() = tfOdomBase.getOrigin().x();
    robotPos.y() = tfOdomBase.getOrigin().y();
    haveTf = true;
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(5.0, "[updateTracks] TF failed: %s", ex.what());
  }

  double adaptiveThresh = m_assocThresh * (m_isRobotMoving || m_isRobotRotating ? 1.8 : 1.0);

  // Associate detections
  for (const auto& det : dets) {
    Obstacle* bestTrack = nullptr;
    double bestDist = adaptiveThresh;

    for (auto& obs : m_obstacles) {
      double dt = (stamp - obs.lastSeen).toSec();
      if (dt < 0 || dt > 2.0) continue;

      double predX = obs.x + obs.vx * dt;
      double predY = obs.y + obs.vy * dt;
      double dist = std::hypot(det.position.x() - predX, det.position.y() - predY);

      if (dist < bestDist) {
        bestDist = dist;
        bestTrack = &obs;
      }
    }

    if (bestTrack) {
      updateObstacle(*bestTrack, det, stamp, robotPos, haveTf);
    } else {
      m_obstacles.emplace_back(m_nextId++, det.position.x(), det.position.y());
      Obstacle& newObs = m_obstacles.back();
      newObs.lastSeen = stamp;
      newObs.has3dData = det.has3d;
      newObs.heightEstimate = det.height;
      ROS_INFO("[Track %d] New at [%.2f, %.2f]", newObs.id, det.position.x(), det.position.y());
    }
  }

  // Remove stale tracks
  m_obstacles.erase(std::remove_if(m_obstacles.begin(), m_obstacles.end(),
    [this](const Obstacle& o) { return o.missedFrames > m_maxMissed; }), m_obstacles.end());
  
  // Log stats
  int nStatic = 0, nDynamic = 0, nUnknown = 0;
  for (const auto& o : m_obstacles) {
    if (o.type == ObstacleType::STATIC) nStatic++;
    else if (o.type == ObstacleType::DYNAMIC) nDynamic++;
    else nUnknown++;
  }
  
  ROS_INFO_THROTTLE(3.0, "[Tracks] %lu total | S:%d D:%d U:%d | Robot: %.2fm/s %.2frad/s",
                    m_obstacles.size(), nStatic, nDynamic, nUnknown,
                    std::hypot(m_robotVx, m_robotVy), m_robotOmega);
}

double ObstaclePredictor::calculateNoiseGate(const Obstacle& obs, 
                                              const Eigen::Vector2d& robotPos, 
                                              bool haveTf) const
{
  double noiseGate = m_baseNoiseGate;
  
  if (m_isRobotRotating && haveTf) {
    double distFromRobot = std::hypot(obs.x - robotPos.x(), obs.y - robotPos.y());
    double expectedTangentialSpeed = distFromRobot * std::fabs(m_robotOmega);
    noiseGate = std::max(m_minRotationNoise, 0.05 + expectedTangentialSpeed * m_rotationNoiseScale);
    if (distFromRobot > m_farObjectThreshold) {
      noiseGate = std::max(noiseGate, m_farObjectNoise);
    }
  } else if (m_isRobotMoving) {
    double robotSpeed = std::hypot(m_robotVx, m_robotVy);
    noiseGate = std::max(m_minRotationNoise, m_baseNoiseGate + robotSpeed * m_linearNoiseScale);
  }
  
  return noiseGate;
}

bool ObstaclePredictor::isLikelyOcclusion(const Obstacle& obs, double displacement, 
                                          double dt) const
{
  if (obs.missedFrames >= 4 && displacement > m_occlusionDisplacementThreshold) return true;
  if (displacement > 3.5 && dt < 0.5) return true;
  if (m_isRobotRotating && displacement > 1.5 && obs.missedFrames >= 3 && dt < m_occlusionTimeThreshold) {
    return true;
  }
  return false;
}

void ObstaclePredictor::updateObstacle(Obstacle& obs, const Detection& det, 
                                        const ros::Time& stamp, const Eigen::Vector2d& robotPos, 
                                        bool haveTf)
{
  double dt = (stamp - obs.lastSeen).toSec();
  double oldX = obs.x, oldY = obs.y;
  double oldVx = obs.vx, oldVy = obs.vy;
  
  double instantVx = 0.0, instantVy = 0.0, instantSpeed = 0.0;
  
  if (dt > 0.02 && dt < 1.0) {
    double dx = det.position.x() - obs.x;
    double dy = det.position.y() - obs.y;
    double displacement = std::hypot(dx, dy);
    
    if (!isLikelyOcclusion(obs, displacement, dt)) {
      instantVx = dx / dt;
      instantVy = dy / dt;
      instantSpeed = std::hypot(instantVx, instantVy);
      
      // Multi-stage validation
      bool velocityRejected = false;
      std::string rejectionReason = "";
      
      // Stage 1: Absolute plausibility
      if (instantSpeed > m_maxPlausibleVelocity) {
        velocityRejected = true;
        rejectionReason = "exceeds max velocity";
        obs.consecutiveRejections++;
      }
      // Stage 2: Acceleration check
      else if (m_enableAccelerationCheck && obs.age >= 2) {
        double oldSpeed = std::hypot(oldVx, oldVy);
        double deltaV = std::fabs(instantSpeed - oldSpeed);
        
        double maxAccel;
        if (m_isRobotRotating) {
          maxAccel = m_maxAccelerationRotating;
        } else if (m_isRobotMoving) {
          maxAccel = m_maxAccelerationMoving;
        } else {
          maxAccel = m_maxAccelerationStationary;
        }
        
        double maxDeltaV = maxAccel * dt;
        
        if (deltaV > maxDeltaV && oldSpeed > 0.15 && obs.consecutiveRejections < 2) {
          velocityRejected = true;
          rejectionReason = "high acceleration";
          obs.consecutiveRejections++;
        }
      }
      
      if (velocityRejected) {
        if (obs.consecutiveRejections >= m_maxConsecutiveRejections) {
          ROS_INFO_THROTTLE(2.0, "[Track %d] %d rejections - accepting (reason: %s)", 
                           obs.id, obs.consecutiveRejections, rejectionReason.c_str());
          
          double acceptanceFactor = m_isRobotRotating ? 0.4 : 0.7;
          obs.vx = instantVx * acceptanceFactor;
          obs.vy = instantVy * acceptanceFactor;
          obs.consecutiveRejections = 0;
          obs.lastValidVx = obs.vx;
          obs.lastValidVy = obs.vy;
        } else {
          obs.vx *= m_velocityRecoveryFactor;
          obs.vy *= m_velocityRecoveryFactor;
          
          if (std::hypot(obs.lastValidVx, obs.lastValidVy) > 0.15) {
            double blendFactor = 0.5;
            obs.vx = blendFactor * obs.lastValidVx + (1.0 - blendFactor) * obs.vx;
            obs.vy = blendFactor * obs.lastValidVy + (1.0 - blendFactor) * obs.vy;
          }
        }
      } else {
        // Velocity accepted
        obs.consecutiveRejections = 0;
        
        double noiseGate = calculateNoiseGate(obs, robotPos, haveTf);
        
        if (displacement < noiseGate * dt) {
          instantVx = instantVy = 0.0;
        } else if (m_isRobotMoving && !m_isRobotRotating && haveTf) {
          double distFromRobot = std::hypot(obs.x - robotPos.x(), obs.y - robotPos.y());
          double velocityDiff = std::hypot(instantVx - m_robotVx, instantVy - m_robotVy);
          
          // Check for parallel motion with stricter criteria
          double robotSpeed = std::hypot(m_robotVx, m_robotVy);
          obs.isParallelMotion = false;
          
          if (robotSpeed > m_parallelMotionMinRobotSpeed && instantSpeed > m_parallelMotionMinSpeed) {
            double robotDirX = m_robotVx / robotSpeed;
            double robotDirY = m_robotVy / robotSpeed;
            double obstacleDirX = instantVx / instantSpeed;
            double obstacleDirY = instantVy / instantSpeed;
            double directionAlignment = robotDirX * obstacleDirX + robotDirY * obstacleDirY;
            
            if (directionAlignment > m_parallelMotionAlignmentThreshold) {
              double speedRatio = instantSpeed / robotSpeed;
              if (speedRatio > 0.5 && speedRatio < 2.0) {
                obs.isParallelMotion = true;
                ROS_INFO_THROTTLE(2.0, "[Track %d] Parallel motion (align=%.2f, speedRatio=%.2f)", 
                                 obs.id, directionAlignment, speedRatio);
              }
            }
          }
          
          if (!obs.isParallelMotion && distFromRobot < 1.5 && velocityDiff < m_relativeVelThresh) {
            instantVx = instantVy = 0.0;
          }
        }
        
        // Smooth velocity
        double alpha;
        if (obs.missedFrames >= 2) {
          alpha = 0.20;
        } else if (m_isRobotRotating) {
          alpha = 0.25;
        } else if (obs.age < 3) {
          alpha = 0.35;
        } else {
          alpha = m_isRobotMoving ? 0.40 : 0.45;
        }
        
        obs.vx = alpha * instantVx + (1.0 - alpha) * obs.vx;
        obs.vy = alpha * instantVy + (1.0 - alpha) * obs.vy;
        
        double currentSpeed = std::hypot(obs.vx, obs.vy);
        if (currentSpeed > 0.1) {
          obs.lastValidVx = obs.vx;
          obs.lastValidVy = obs.vy;
        }
        
        // Update motion consistency
        if (obs.age >= 2 && instantSpeed > 0.05) {
          double oldSpeed = std::hypot(oldVx, oldVy);
          if (oldSpeed > 0.05) {
            double dot = (instantVx * oldVx + instantVy * oldVy) / (instantSpeed * oldSpeed);
            double consistency = (dot + 1.0) / 2.0;
            obs.motionConsistencyScore = 0.3 * consistency + 0.7 * obs.motionConsistencyScore;
          }
        }
        
        double finalSpeed = std::hypot(obs.vx, obs.vy);
        if (finalSpeed < 0.05) {
          obs.vx *= 0.3;
          obs.vy *= 0.3;
        }
      }
    } else {
      ROS_INFO_THROTTLE(1.0, "[Track %d] Occlusion detected (disp=%.2fm, dt=%.2fs)", 
                        obs.id, displacement, dt);
      obs.vx *= 0.4;
      obs.vy *= 0.4;
      obs.motionConsistencyScore *= 0.6;
      obs.consecutiveRejections = 0;
    }
  } else if (dt >= 1.0) {
    obs.vx = 0.0;
    obs.vy = 0.0;
    obs.motionConsistencyScore = 0.0;
    obs.consecutiveRejections = 0;
  }

  // Update position and metadata
  obs.x = det.position.x();
  obs.y = det.position.y();
  obs.lastSeen = stamp;
  obs.missedFrames = 0;
  obs.totalMotion += std::hypot(det.position.x() - oldX, det.position.y() - oldY);
  obs.age++;
  
  if (!m_isRobotMoving && !m_isRobotRotating) {
    double dev = std::hypot(obs.x - obs.initialX, obs.y - obs.initialY);
    obs.maxDeviation = std::max(obs.maxDeviation, dev);
  } else if (std::hypot(obs.vx, obs.vy) < 0.1) {
    obs.initialX = obs.x;
    obs.initialY = obs.y;
    obs.maxDeviation = 0.0;
  }

  if (det.has3d) {
    obs.has3dData = true;
    obs.heightEstimate = det.height;
  }
  
  classifyObstacle(obs);
}

void ObstaclePredictor::classifyObstacle(Obstacle& obs)
{
  double speed = std::hypot(obs.vx, obs.vy);
  double avgMotionPerFrame = obs.totalMotion / obs.age;
  
  bool robotActive = m_isRobotMoving || m_isRobotRotating;
  double unknownToDynamicThresh = robotActive ? m_unknownToDynamicSpeedMoving : m_unknownToDynamicSpeedStationary;
  double staticToUnknownThresh = robotActive ? m_staticToUnknownSpeedMoving : m_staticToUnknownSpeedStationary;
  
  // Early classification
  if (obs.age < m_minAgeForClassification) {
    if (speed >= 1.0 && obs.age >= 5 && avgMotionPerFrame > 0.25 && 
        obs.motionConsistencyScore > 0.65) {
      obs.type = ObstacleType::DYNAMIC;
      ROS_INFO("[Track %d] Fast early -> Dynamic (speed=%.2fm/s)", obs.id, speed);
    } 
    else if (obs.isParallelMotion && obs.age >= 5 && speed > 0.30 && 
             avgMotionPerFrame > 0.10 && obs.motionConsistencyScore > 0.6) {
      obs.type = ObstacleType::DYNAMIC;
      ROS_INFO("[Track %d] Parallel early -> Dynamic (speed=%.2fm/s)", obs.id, speed);
    } 
    else if (obs.type != ObstacleType::UNKNOWN) {
      obs.type = ObstacleType::UNKNOWN;
    }
    return;
  }
  
  bool hasMovedFromStart = (obs.maxDeviation > 0.40);
  bool hasConsistentMotion = (obs.motionConsistencyScore > 0.55);
  
  switch (obs.type) {
    case ObstacleType::UNKNOWN: {
      if (obs.isParallelMotion) {
        if (speed >= 0.30 && avgMotionPerFrame > 0.10 && 
            obs.motionConsistencyScore > 0.6 && hasMovedFromStart) {
          obs.type = ObstacleType::DYNAMIC;
          obs.consecutiveMovingFrames = 0;
          ROS_INFO("[Track %d] Parallel -> Dynamic (speed=%.2fm/s, dev=%.2fm)", 
                   obs.id, speed, obs.maxDeviation);
        }
      } else if (!robotActive) {
        if (speed >= unknownToDynamicThresh && avgMotionPerFrame > 0.12 && 
            hasMovedFromStart && hasConsistentMotion) {
          obs.type = ObstacleType::DYNAMIC;
          obs.consecutiveMovingFrames = 0;
          ROS_INFO("[Track %d] Unknown -> Dynamic [stationary] (speed=%.2fm/s, dev=%.2fm)", 
                   obs.id, speed, obs.maxDeviation);
        }
      } else {
        if (speed >= unknownToDynamicThresh && avgMotionPerFrame > 0.20 && 
            hasMovedFromStart && hasConsistentMotion) {
          obs.type = ObstacleType::DYNAMIC;
          obs.consecutiveMovingFrames = 0;
          ROS_INFO("[Track %d] Unknown -> Dynamic [moving] (speed=%.2fm/s, dev=%.2fm)", 
                   obs.id, speed, obs.maxDeviation);
        }
      }
      
      if (speed < 0.06 && avgMotionPerFrame < 0.03 && ++obs.consecutiveMovingFrames >= 8) {
        obs.type = ObstacleType::STATIC;
        obs.consecutiveMovingFrames = 0;
        ROS_INFO("[Track %d] Unknown -> Static (speed=%.2fm/s)", obs.id, speed);
      } else if (speed >= 0.06) {
        obs.consecutiveMovingFrames = 0;
      }
      break;
    }
    
    case ObstacleType::STATIC:
      if (speed >= staticToUnknownThresh && avgMotionPerFrame > 0.15 && 
          ++obs.consecutiveMovingFrames >= 12) {
        obs.type = ObstacleType::UNKNOWN;
        obs.consecutiveMovingFrames = 0;
        ROS_INFO("[Track %d] Static -> Unknown (speed=%.2fm/s)", obs.id, speed);
      } else if (speed < staticToUnknownThresh) {
        obs.consecutiveMovingFrames = 0;
      }
      break;
      
    case ObstacleType::DYNAMIC:
      if (obs.isParallelMotion && speed > 0.20) {
        obs.consecutiveMovingFrames = 0;
      } else if (speed < 0.10 && avgMotionPerFrame < 0.06 && 
                 ++obs.consecutiveMovingFrames >= 10) {
        obs.type = ObstacleType::UNKNOWN;
        obs.consecutiveMovingFrames = 0;
        ROS_INFO("[Track %d] Dynamic -> Unknown (stopped: speed=%.2fm/s)", obs.id, speed);
      } else if (speed >= 0.10) {
        obs.consecutiveMovingFrames = 0;
      }
      break;
  }
}

// ============================================================================
// Visualization
// ============================================================================

void ObstaclePredictor::publishMarkers()
{
  visualization_msgs::MarkerArray obsMarkers, predMarkers;
  std::unordered_set<int> currentIds;
  ros::Time now = m_lastStamp.isZero() ? ros::Time::now() : m_lastStamp;

  for (const auto& obs : m_obstacles) {
    if (obs.type != ObstacleType::DYNAMIC) continue;
    currentIds.insert(obs.id);

    // Obstacle sphere
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "odom";
    sphere.header.stamp = now;
    sphere.ns = "obstacles";
    sphere.id = obs.id;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x = obs.x;
    sphere.pose.position.y = obs.y;
    sphere.pose.position.z = obs.has3dData ? obs.heightEstimate : 0.3;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5;
    sphere.color.r = 0.0;
    sphere.color.g = 1.0;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0.5);
    obsMarkers.markers.push_back(sphere);

    // Prediction path
    visualization_msgs::Marker line;
    line.header.frame_id = "odom";
    line.header.stamp = now;
    line.ns = "predictions";
    line.id = obs.id;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 0.0;
    line.color.g = 0.0;
    line.color.b = 1.0;
    line.color.a = 0.8;
    line.lifetime = ros::Duration(0.5);

    double vizHeight = obs.has3dData ? obs.heightEstimate : 0.3;
    for (double t = 0; t <= m_predHorizon; t += m_predDt) {
      geometry_msgs::Point p;
      p.x = obs.x + obs.vx * t;
      p.y = obs.y + obs.vy * t;
      p.z = vizHeight;
      line.points.push_back(p);
    }
    predMarkers.markers.push_back(line);
  }

  // Delete old markers
  for (int id : m_lastPublishedIds) {
    if (currentIds.count(id)) continue;
    
    visualization_msgs::Marker del;
    del.header.frame_id = "odom";
    del.header.stamp = now;
    del.action = visualization_msgs::Marker::DELETE;
    del.id = id;
    del.ns = "obstacles";
    obsMarkers.markers.push_back(del);
    del.ns = "predictions";
    predMarkers.markers.push_back(del);
  }
  
  m_lastPublishedIds = currentIds;
  m_pubObstacles.publish(obsMarkers);
  m_pubPredictions.publish(predMarkers);
}
