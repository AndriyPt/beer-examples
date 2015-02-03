#include "utils.h"

#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

void publishJointState(ros::Publisher &publisher, double value, std::string infoString, unsigned delay) {

    ROS_INFO_COND(!infoString.empty(), "%s", infoString.c_str());

    std_msgs::Float64 message;
    message.data = value;
    publisher.publish(message);

    if (delay > 0) {
        sleep(delay);
    }
}

geometry_msgs::Pose getPose(double x, double y, double z, double roll, double pitch, double yaw) {

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Pose result;
    result.position.x = x;
    result.position.y = y;
    result.position.z = z;
    result.orientation.x = quaternion.getX();
    result.orientation.y = quaternion.getY();
    result.orientation.z = quaternion.getZ();
    result.orientation.w = quaternion.getW();

    return result;
}


std::vector<std::string> getGazeboWorldModelsName(ros::NodeHandle &nodeHandle) {

    std::vector<std::string> result;
    ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    gazebo_msgs::GetWorldProperties getWorldProperties;

    if (client.call(getWorldProperties)) {
        result.insert(result.end(), getWorldProperties.response.model_names.begin(),
                      getWorldProperties.response.model_names.end());
    }

    return result;
}

void addGazeboModel(ros::NodeHandle &nodeHandle, std::string modelName, std::string modelXml, geometry_msgs::Pose initialPose) {

    ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawnModel;
    spawnModel.request.model_name = modelName;
    spawnModel.request.model_xml = modelXml;
    spawnModel.request.robot_namespace = ros::this_node::getNamespace();
    spawnModel.request.initial_pose = initialPose;
    spawnModel.request.reference_frame = "";
    client.call(spawnModel);
}

void addGazeboModelFromDatabase(ros::NodeHandle &nodeHandle, std::string modelName, std::string modelType,
                                geometry_msgs::Pose initialPose) {

    const unsigned bufferSize = 300;
    char buffer[bufferSize];
    snprintf(buffer, bufferSize,
            "<sdf version=\"1.4\">"
            "  <world name=\"default\">"
            "    <include>"
            "       <uri>model://%s</uri>"
            "    </include>"
            "  </world>"
            "</sdf>", modelType.c_str());
    addGazeboModel(nodeHandle, modelName, buffer, initialPose);
}

void addGazeboModelFromSdf(ros::NodeHandle &nodeHandle, std::string modelName, std::string fileName,
                                geometry_msgs::Pose initialPose) {

    std::ifstream file(fileName.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();

    addGazeboModel(nodeHandle, modelName, buffer.str(), initialPose);
}

void updateGazeboModel(ros::NodeHandle &nodeHandle, std::string modelName, geometry_msgs::Pose pose) {

    ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setModelState;

    setModelState.request.model_state.model_name = modelName;
    setModelState.request.model_state.pose = pose;
    setModelState.request.model_state.reference_frame = "world";
    client.call(setModelState);
}

void deleteGazeboModel(ros::NodeHandle &nodeHandle, std::string modelName) {

    ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel deleteModel;
    deleteModel.request.model_name = modelName;
    client.call(deleteModel);
}

std::string unsignedToString(unsigned value) {
    const unsigned bufferSize = 300;
    char buffer[bufferSize];
    snprintf(buffer, bufferSize, "%u", value);
    return std::string(buffer);
}

void runGrasp(unsigned graspIndex, bool preGrasp, std::string graspsFileName) {

    system("rosparam delete /quick_grasp/grasps_file");

    std::string command("rosrun sr_grasp quick_grasp ");

    if (!graspsFileName.empty()) {
        command.append("_grasps_file:=");
        command.append(graspsFileName);
    }

    command.append(" <<'EOF'\n");
    command.append(unsignedToString(graspIndex));
    command.append("\n");

    if (preGrasp) {
        command.append("p\n");
    }
    else {
        command.append("g\n");
    }
    command.append("q\n");
    command.append("EOF\n");

    system(command.c_str());
}
