#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

void publishJointState(ros::Publisher &publisher, double value, std::string infoString = "", unsigned delay = 0);

geometry_msgs::Pose getPose(double x = 0.0, double y = 0.0, double z = 0.0, double roll = 0.0,
                            double pitch = 0.0, double yaw = 0.0);

std::vector<std::string> getGazeboWorldModelsName(ros::NodeHandle &nodeHandle);

void addGazeboModel(ros::NodeHandle &nodeHandle, std::string modelName, std::string modelXml, geometry_msgs::Pose initialPose);

void addGazeboModelFromDatabase(ros::NodeHandle &nodeHandle, std::string modelName, std::string modelType,
                                geometry_msgs::Pose initialPose);

void addGazeboModelFromSdf(ros::NodeHandle &nodeHandle, std::string modelName, std::string fileName,
                                geometry_msgs::Pose initialPose);

void updateGazeboModel(ros::NodeHandle &nodeHandle, std::string modelName, geometry_msgs::Pose pose);

void deleteGazeboModel(ros::NodeHandle &nodeHandle, std::string modelName);

void runGrasp(unsigned graspIndex, bool preGrasp = false, std::string graspsFileName = "");


#endif // UTILS_H

