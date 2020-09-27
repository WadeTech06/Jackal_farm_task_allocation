#!/usr/bin/env python
import rospy
import sys
import select
import roslaunch
import rospkg
import os
import random
import numpy as np
import actionlib
from nav_msgs.srv import GetPlan
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from jackal_farm.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Initialize variables
key = ""
num_of_robots = 3
task_count = 0
task_dict = {}

jackal_pose = [Pose()] * num_of_robots


def key_CB(key_msg):
    global key
    key = key_msg.data


def odometry0Cb(msg):
    global jackal_pose
    jackal_pose[0] = msg.pose.pose


def odometry1Cb(msg):
    global jackal_pose
    jackal_pose[1] = msg.pose.pose


def odometry2Cb(msg):
    global jackal_pose
    jackal_pose[2] = msg.pose.pose


def launch_subscribers():
    rospy.init_node('launch_control')
    rospy.Subscriber("/key_opt", String, key_CB)
    rospy.Subscriber('/jackal0/jackal_velocity_controller/odom',
                     Odometry, odometry0Cb)
    rospy.Subscriber('/jackal1/jackal_velocity_controller/odom',
                     Odometry, odometry1Cb)
    rospy.Subscriber('/jackal2/jackal_velocity_controller/odom',
                     Odometry, odometry2Cb)


def print_instructions():
    print ""
    print "Press 5 to spawn random task"
    print "Press 6 to start allocation"
    print ""


def send_goals(assignments):
    count = 0
    global task_dict

    for i in assignments:
        robot_base = '/jackal%s/move_base' % count
        frame_name = "jackal%s/odom" % count
        client = actionlib.SimpleActionClient(robot_base, MoveBaseAction)
        print "waiting for %s" % robot_base
        client.wait_for_server()

        x = task_dict[i]['x']
        y = task_dict[i]['y']

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        print "jackal %s sent to task %s" % (count, i)
        count += 1


def start():
    global jackal_pose
    global task_dict
    global task_count
    global num_of_robots

    cost_vector = []

    # TODO: figure out how to not hardcode robots
    for x in range(3):
        srv_name = 'jackal%s/move_base/NavfnROS/make_plan' % x
        frame_name = "jackal%s/odom" % x

        get_plan = rospy.ServiceProxy(srv_name, GetPlan)
        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = frame_name
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = jackal_pose[x].position.x
        start.pose.position.y = jackal_pose[x].position.y

        for y in range(task_count):
            goal = PoseStamped()
            goal.header.seq = 0
            goal.header.frame_id = frame_name
            goal.header.stamp = rospy.Time(0)
            goal.pose.position.x = task_dict[y]['x']
            goal.pose.position.y = task_dict[y]['y']

            req = GetPlan()
            req.start = start
            req.goal = goal
            req.tolerance = .5
            resp = get_plan(req.start, req.goal, req.tolerance)

            path_msg = resp.plan
            path_length = 0
            for i in range(len(path_msg.poses) - 1):
                position_a_x = path_msg.poses[i].pose.position.x
                position_b_x = path_msg.poses[i+1].pose.position.x
                position_a_y = path_msg.poses[i].pose.position.y
                position_b_y = path_msg.poses[i+1].pose.position.y

                path_length += np.sqrt(np.power((position_b_x - position_a_x),
                                                2) + np.power((position_b_y - position_a_y), 2))

            cost_vector.append(path_length)

    print(cost_vector)
    get_assignments = rospy.ServiceProxy("/solve_hungarian", hung)
    req = hung()
    req.num_of_robots = num_of_robots
    req.num_of_task = task_count
    req.cost_vector = np.array(cost_vector)
    resp = get_assignments(req.num_of_robots, req.num_of_task, req.cost_vector)
    print(resp.assignments)
    print(resp.cost)

    send_goals(resp.assignments)


def check_buttons():
    global key
    global task_count
    global task_dict

    # Spawn Task
    if key == "5":
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        cmd = ("rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH`/strawberry-basket/model.sdf -sdf -model strawberry-basket%s -y %f -x %f -z 0 " % (task_count, y, x))
        os.system(cmd)
        task_dict[task_count] = {'x': x, 'y': y}
        task_count += 1

    if key == "6":
        start()

    key = ""

def main():
    # start node to subscribe to joy messages node end messages
    launch_subscribers()

    # check buttons and launch the appropriate file
    while not rospy.is_shutdown():
        check_buttons()
    rospy.spin()


if __name__ == '__main__':
    print_instructions()

    main()
