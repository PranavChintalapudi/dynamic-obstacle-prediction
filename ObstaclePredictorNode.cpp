/*
 * Name: ObstaclePredictorNode.cpp
 * Author: Pranav Chintalapudi
 * Organization: ANSCER Robotics
 * Date: 2026-01-20
 * Version: 1.0
 * 
 * Description: Main node entry point for dynamic obstacle prediction
 */

#include "dynamic_obstacle_prediction/ObstaclePredictor.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_predictor_node");
  
  ObstaclePredictor predictor;
  
  ros::spin();
  
  return 0;
}