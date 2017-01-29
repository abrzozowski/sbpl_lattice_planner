#include <ros/ros.h>

#include "sbpl_lattice_planner/sbpl_lattice_planner.h"

namespace sbpl_lattice_planner {
    class SBPLLatticePlannerWithCostmap : public SBPLLatticePlanner
    {
    public:
        SBPLLatticePlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS *cmap_ros);

    private:
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);

        ros::Subscriber goal_sub_;
        costmap_2d::Costmap2DROS* cmap_ros_;
    };

    SBPLLatticePlannerWithCostmap::SBPLLatticePlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS *cmap_ros) :
        SBPLLatticePlanner(name, cmap_ros)
    {
        ros::NodeHandle private_nh("~");

        cmap_ros_ = cmap_ros;
        goal_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &SBPLLatticePlannerWithCostmap::goalCallback, this);
    }

    void SBPLLatticePlannerWithCostmap::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        ROS_INFO("Got the new goal.");

        tf::Stamped<tf::Pose> global_pose;
        cmap_ros_->getRobotPose(global_pose);
        std::vector<geometry_msgs::PoseStamped> path;
        geometry_msgs::PoseStamped start;
        start.header.stamp = global_pose.stamp_;
        start.header.frame_id = global_pose.frame_id_;
        start.pose.position.x = global_pose.getOrigin().x();
        start.pose.position.y = global_pose.getOrigin().y();
        start.pose.position.z = global_pose.getOrigin().z();
        start.pose.orientation.x = global_pose.getRotation().x();
        start.pose.orientation.y = global_pose.getRotation().y();
        start.pose.orientation.z = global_pose.getRotation().z();
        start.pose.orientation.w = global_pose.getRotation().w();

        makePlan(start, *goal, path);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sbpl_lattice_planner_node");

    ROS_INFO("Starting sbpl lattice planner node");

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("costmap", tf);

    sbpl_lattice_planner::SBPLLatticePlannerWithCostmap planner("sbpl_lattice_planner", &costmap);

    ros::spin();

    return 0;
}
