//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <hector_exploration_planner/hector_exploration_planner.h>

class ExplorationGateway
{
public:
  ExplorationGateway() : move_base_ac_("move_base", true), costmap_2d_("global_costmap", tf_listener_), planner_()
  {
  }

  bool init()
  {
    ros::NodeHandle pnh("~");

    pnh.param("frequency",    frequency_,     1.0);
    pnh.param("close_enough", close_enough_,  0.3);  // close enough to current exploration goal
    pnh.param("goal_timeout", goal_timeout_, 30.0);  // maximum time to reach an exploration goal

    planner_.initialize("hector_exploration_planner", &costmap_2d_);

    // Wait for the move_base action servers to come up
    ros::Time t0 = ros::Time::now();
    double timeout = 10.0;

    while ((move_base_ac_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
    {
      if ((ros::Time::now() - t0).toSec() > timeout/2.0)
        ROS_WARN_THROTTLE(3, "Waiting for move_base action server to come up...");

      if ((ros::Time::now() - t0).toSec() > timeout)
      {
        ROS_ERROR("Timeout while waiting for move_base action server to come up");
        return false;
      }
    }

    exploration_goal_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("exploration_goal", 2, true);

    return true;
  }

  void spin()
  {
    bool have_goal = false;
    move_base_msgs::MoveBaseGoal mb_goal;

    ros::Rate rate(frequency_);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();

      if ((have_goal == true) && (move_base_ac_.waitForResult(ros::Duration(0.01)) == false))
      {
        // We are still pursuing a goal...

        if ((ros::Time::now() - mb_goal.target_pose.header.stamp).toSec() >= goal_timeout_)
        {
          ROS_WARN("Cannot reach goal after %.2f seconds; request a new one (current state is %s)",
                    goal_timeout_, move_base_ac_.getState().toString().c_str());
        }
        else
        {
          tf::Stamped<tf::Pose> robot_gb;
          if (costmap_2d_.getRobotPose(robot_gb) == false)
          {
            ROS_WARN_THROTTLE(2, "Cannot get robot pose from costmap 2D");
            continue;
          }

          // When close enough to current exploration goal, request for a new
          // one, so we avoid the final slow approach and subgoal obsession
          double distance =  std::sqrt(std::pow(robot_gb.getOrigin().x() - mb_goal.target_pose.pose.position.x, 2)
                                     + std::pow(robot_gb.getOrigin().y() - mb_goal.target_pose.pose.position.y, 2));
          if (distance > close_enough_)
          {
            continue;
          }
          else
          {
            ROS_DEBUG("Close enough to current goal (%.2f <= %.2f m); request a new one", distance, close_enough_);
          }
        }
      }
      else
      {
        have_goal = false;
      }

      // No active goal, or close enough to it, or timeout; in all these cases, request a new goal
      tf::Stamped<tf::Pose> robot_pose_tf;
      costmap_2d_.getRobotPose(robot_pose_tf);

      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(robot_pose_tf, pose);
      nav_msgs::Path path;
      planner_.doExploration(pose, path.poses);

      if (path.poses.size() == 0)
      {
        ROS_WARN("Exploration planner returned an empty path; retrying...");
        continue;
      }

      mb_goal.target_pose = path.poses.back();
      mb_goal.target_pose.header.stamp = ros::Time::now();
      exploration_goal_pub_.publish(mb_goal.target_pose);

      ROS_INFO("New exploration goal: %.2f, %.2f, %.2f",
               mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
               tf::getYaw(mb_goal.target_pose.pose.orientation));

      // TODO This is a horrible workaround for a problem I cannot solve: send a new goal
      // when the previous one has been cancelled return immediately with succeeded state
      int times_sent = 0;
      do
      {
        move_base_ac_.sendGoal(mb_goal);
        times_sent++;
      } while ((move_base_ac_.waitForResult(ros::Duration(0.1)) == true) &&
               (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED));

      if (times_sent > 1)
        ROS_WARN("Again the strange case of instantaneous goals... (goal sent %d times)", times_sent);

      have_goal = true;
    }
  }

protected:
  double frequency_;
  double close_enough_;
  double goal_timeout_;

  hector_exploration_planner::HectorExplorationPlanner planner_;
  tf::TransformListener   tf_listener_;
  costmap_2d::Costmap2DROS costmap_2d_;
  ros::Publisher exploration_goal_pub_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ExplorationGateway eg;
  if (eg.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }
  ROS_INFO("%s initialized", ros::this_node::getName().c_str());
  eg.spin();
  return 0;
}
