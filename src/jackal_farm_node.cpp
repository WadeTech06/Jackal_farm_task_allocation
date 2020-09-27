#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "hungarian-algorithm-cpp/Hungarian.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jackal_farm");

  // system("rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH`/strawberry-basket/model.sdf -sdf -model strawberry-basket1 -y 57 -x 50 -z 2 ");

  /*
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac0("jackal0/move_base", true);
  MoveBaseClient ac1("jackal1/move_base", true);

  //wait for the action server to come up
  while (!ac0.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for ac0 move_base action server to come up");
  }

  while (!ac1.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for ac1 move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal0;
  move_base_msgs::MoveBaseGoal goal1;

  //we'll send a goal to the robot to move 1 meter forward
  goal0.target_pose.header.frame_id = "jackal0/odom";
  goal0.target_pose.header.stamp = ros::Time::now();

  goal0.target_pose.pose.position.x = 1.0;
  goal0.target_pose.pose.orientation.w = 1.0;

  goal1.target_pose.header.frame_id = "jackal1/odom";
  goal1.target_pose.header.stamp = ros::Time::now();

  goal1.target_pose.pose.position.x = 1.0;
  goal1.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac0.sendGoal(goal0);
  ac1.sendGoal(goal1);

  ac0.waitForResult();

  if (ac0.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, robot0 the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  ac1.waitForResult();

  if (ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, robot1 the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason"); */

  // vector<vector<double>> costMatrix = {{5, 100, 100, 100, 100},
  //                                    {3, 7, 3, 100, 100},
  //                                    {6, 100, 2, 100, 4},
  //                                    {100, 100, 5, 6, 2},
  //                                    {100, 4, 100, 1, 6}};

  vector<vector<double>> costMatrix(3);
  vector<double> costVector = {5.7786926382979802, 11.132981372248176, 6.4786269728021662, 5.5365799622507801, 11.025189367691054, 6.7342578131561854, 5.5365799622507801, 11.025076162212819, 6.7342578131561854};
      for (int i = 0; i < 3; i++)
    {
        costMatrix[i].resize(3);
    }

    for (int i = 0; i < costVector.size(); i++)
    {
        int row = i / 3;
        int col = i % 3;
        costMatrix[row][col] = costVector[i];
    }

  HungarianAlgorithm HungAlgo;
  vector<int> assignment;

  double cost = HungAlgo.Solve(costMatrix, assignment);

  for (unsigned int x = 0; x < costMatrix.size(); x++)
    std::cout << x << "," << assignment[x] << "\t";

  std::cout << "\ncost: " << cost << std::endl;

  return 0;
}

//TODO: get roboot path plan and calculate the assignments