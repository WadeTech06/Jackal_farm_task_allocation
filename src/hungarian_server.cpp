#include "ros/ros.h"
#include "hungarian-algorithm-cpp/Hungarian.h"
#include "jackal_farm/hung.h"

bool solve(jackal_farm::hung::Request &req, jackal_farm::hung::Response &res)
{
    vector<int> assignment;
    vector<vector<double>> costMatrix(req.num_of_robots);

    for (int i = 0; i < req.num_of_robots; i++)
    {
        costMatrix[i].resize(req.num_of_tasks);
    }

    for (int i = 0; i < req.cost_vector.size(); i++)
    {
        int row = i / req.num_of_robots;
        int col = i % req.num_of_tasks;
        costMatrix[row][col] = req.cost_vector[i];
    }

    HungarianAlgorithm HungAlgo;
    double cost = HungAlgo.Solve(costMatrix, assignment);
    res.assignments = assignment;
    res.cost = cost;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solve_hungarian_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("solve_hungarian", solve);
    ros::spin();

    return 0;
}