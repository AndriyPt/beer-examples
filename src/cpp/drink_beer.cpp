#include <assert.h>
#include <algorithm>
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

#include "utils.h"


void addOrUpdateModel(ros::NodeHandle &nodeHandle, std::vector<std::string> &existingModels, std::string modelName,
                      geometry_msgs::Pose pose, std::string modelType = "", std::string fileName = "", bool hardReset = false) {

    bool addNewModel = false;
    if (std::find(existingModels.begin(), existingModels.end(), modelName) != existingModels.end()) {
        ROS_INFO("Updating %s position...", modelName.c_str());
        if (hardReset) {
            deleteGazeboModel(nodeHandle, modelName);
            sleep(2);
            addNewModel = true;
        }
        else {
            updateGazeboModel(nodeHandle, modelName, pose);
        }
    }
    else {
        ROS_INFO("Adding new %s...", modelName.c_str());
        addNewModel = true;
    }

    if (addNewModel) {

        assert((!fileName.empty() || !modelType.empty()) && "Either modelType or fileName should be provided");
        assert((fileName.empty() || modelType.empty()) && "Only one parameter should be provided modelType or fileName");

        if (!fileName.empty()) {
            addGazeboModelFromSdf(nodeHandle, modelName, fileName, pose);
        }
        else {
            addGazeboModelFromDatabase(nodeHandle, modelName, modelType, pose);
        }
    }
}

void createWorldItems(ros::NodeHandle &nodeHandle) {

    std::vector<std::string> existingModels = getGazeboWorldModelsName(nodeHandle);

    geometry_msgs::Pose tablePose = getPose(0.8, 0.45, 0.0, 0.0, 0.0, M_PI_2);
    addOrUpdateModel(nodeHandle, existingModels, "main_table", tablePose, "table");

    geometry_msgs::Pose trashBoxPose = getPose(0.6, -0.55);
    addOrUpdateModel(nodeHandle, existingModels, "trash_box", trashBoxPose, "",
                     ros::package::getPath("beer_examples") + "/models/wooden_box.sdf");


    geometry_msgs::Pose beerPose = getPose(0.67, -0.22, 1.015);
    addOrUpdateModel(nodeHandle, existingModels, "my_beer", beerPose, "",
                     ros::package::getPath("beer_examples") + "/models/beer.sdf", true);

    sleep(2);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "drink_beer_cpp");

    ros::NodeHandle nodeHandler;

    ros::Publisher arm_swing_publisher = nodeHandler.advertise<std_msgs::Float64>("sa_ss_position_controller/command", 10, true);
    ros::Publisher arm_rotation_publisher = nodeHandler.advertise<std_msgs::Float64>("sa_sr_position_controller/command", 10, true);
    ros::Publisher elbow_swing_publisher = nodeHandler.advertise<std_msgs::Float64>("sa_es_position_controller/command", 10, true);
    ros::Publisher elbow_rotation_publisher = nodeHandler.advertise<std_msgs::Float64>("sa_er_position_controller/command", 10, true);

    publishJointState(elbow_swing_publisher, 2.0, "Moving elbow up...");
    publishJointState(arm_swing_publisher, 1.0, "Moving arm up...", 2);
    publishJointState(arm_rotation_publisher, -0.25, "Moving arm to object...");
    publishJointState(elbow_rotation_publisher, -0.7, "Rotating elbow...", 2);

    createWorldItems(nodeHandler);

    ROS_INFO("Preparing for grasp...");
    const unsigned powerGraspVerticalIndex = 3;
    runGrasp(powerGraspVerticalIndex, true);

    publishJointState(arm_swing_publisher, 0.78, "Moving arm down...");
    publishJointState(elbow_swing_publisher, 0.6, "Moving elbow down...", 2);
    publishJointState(arm_rotation_publisher, -0.2, "Moving arm to object...", 2);

    ROS_INFO("Grasping...");
    runGrasp(powerGraspVerticalIndex);

    publishJointState(arm_swing_publisher, 1.6, "Moving hand up...");
    publishJointState(elbow_swing_publisher, 2.0, "", 4);

    publishJointState(arm_rotation_publisher, -2.0, "Rotating arm...");
    publishJointState(elbow_rotation_publisher, 0.7, "Rotating elbow...", 2);

    publishJointState(elbow_swing_publisher, 0.3, "Throwing object...");
    runGrasp(powerGraspVerticalIndex, true);

    publishJointState(arm_swing_publisher, 0.8, "Moving arm...");
    publishJointState(arm_rotation_publisher, 0.0, "Rotating arm...");
    publishJointState(elbow_swing_publisher, 0.6, "Moving elbow up...");
    publishJointState(elbow_rotation_publisher, -0.7, "Rotating elbow...", 4);

    ROS_INFO("Showing Ok...");
    const unsigned okSignGraspIndex = 0;
    runGrasp(okSignGraspIndex, false, ros::package::getPath("beer_examples") + "/resource/grasps.yaml");

    return 0;
}

