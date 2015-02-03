#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient, GoalStatus
from sr_robot_msgs.msg import GraspAction, GraspGoal
from sr_grasp import GraspStash
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, SetModelState, DeleteModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import tf.transformations as tft


def run_grasp(grasp, pre=False):
    goal = GraspGoal()
    goal.grasp = grasp
    goal.pre_grasp = pre
    client = SimpleActionClient('grasp', GraspAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(20.0))
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Success!")
    else:
        rospy.logerr("Fail!")


def get_grasp(grasp_name, filename=None):
    stash = GraspStash()
    if filename is None:
        stash.load_all()
    else:
        stash.load_yaml_file(filename)

    for index, grasp in enumerate(stash.get_all()):
        if grasp.id == grasp_name:
            return grasp

    error_message = "No grasp {} found!".format(grasp_name)
    rospy.logerr(error_message)
    raise Exception(error_message)


def publish_joint_state(publisher, value, info_string=None, delay=0):

    if publisher is None:
        raise Exception("Not initialized publisher")

    if info_string is not None:
        rospy.loginfo(info_string)

    publisher.publish(value)

    if delay > 0:
        rospy.sleep(delay)


def get_pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    quaternion = tft.quaternion_from_euler(roll, pitch, yaw)
    object_pose = Pose()
    object_pose.position.x = float(x)
    object_pose.position.y = float(y)
    object_pose.position.z = float(z)
    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]
    return object_pose


def get_gazebo_world_models_name():
    rospy.wait_for_service("/gazebo/get_world_properties")
    world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    result = world_properties()

    if result is not None:
        return result.model_names
    return []


def add_gazebo_model(model_name, model_xml, initial_pose):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model(model_name, model_xml, rospy.get_namespace(), initial_pose, "")


def add_gazebo_model_from_database(model_name, model_type, initial_pose):
    model_database_template = """<sdf version="1.4">
        <world name="default">
        <include>
        <uri>model://MODEL_NAME</uri>
        </include>
        </world>
        </sdf>"""
    add_gazebo_model(model_name, model_database_template.replace("MODEL_NAME", model_type),
        initial_pose)


def add_gazebo_model_from_sdf(model_name, filename, initial_pose):
    with open(filename, "r") as content_file:
        model_xml = content_file.read()
    add_gazebo_model(model_name, model_xml, initial_pose)


def update_gazebo_model(model_name, pose):
    rospy.wait_for_service("/gazebo/set_model_state")
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.pose = pose
    model_state.twist = Twist()
    model_state.reference_frame = "world"
    set_model_state(model_state)


def delete_gazebo_model(model_name):
    rospy.wait_for_service("/gazebo/delete_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    delete_model(model_name)