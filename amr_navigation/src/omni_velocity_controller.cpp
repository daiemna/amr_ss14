#include "omni_velocity_controller.h"
#include <ros/console.h>
#include <math.h>
//========================= YOUR CODE HERE =========================
// Instructions: implement all the functions that you have declared
//               in the header file.



OmniVelocityController::OmniVelocityController(double l_max_vel, double l_max_acc, double l_tolerance,
                                               double a_max_vel, double a_max_acc, double a_tolerance)
: l_max_vel_(l_max_vel)
, l_max_acc_(l_max_acc)
, l_tolerance_(l_tolerance)
, a_max_vel_(a_max_vel)
, a_max_acc_(a_max_acc)
, a_tolerance_(a_tolerance)
{
}

void OmniVelocityController::setTargetPose(const Pose& pose)
{
  target_pose_ = pose;
  linear_complete_ = false;
  angular_complete_ = false;
}

bool OmniVelocityController::isTargetReached() const
{
  return linear_complete_ & angular_complete_;
}

Velocity OmniVelocityController::computeVelocity(const Pose& actual_pose)
{
	// ROS_INFO("DNA_DEBUG: Compute Velocity Called!");
	// ROS_INFO("DNA_DEBUG: max acc : %2f, max velocity : %2f, max angular vel : %2f, max angular acc: %2f",l_max_acc_,l_max_vel_,a_max_vel_,a_max_acc_);
	ROS_INFO("DNA_DEBUG: current pose:[%2f,%2f,%2f] required pose: [%2f,%2f,%2f]",actual_pose.x,actual_pose.y,actual_pose.theta,target_pose_.x,target_pose_.y,target_pose_.theta);
	//check for validation
	if(std::abs(target_pose_.x) > 8 || std::abs(target_pose_.y) > 8)
		return Velocity();
	// Displacement and orientation to the target in world frame
    double dx = target_pose_.x - actual_pose.x;
    double dy = target_pose_.y - actual_pose.y;
    double vel =0, theta = 0, vel_x = 0, vel_y = 0, gama = 0;
    // Step 1: compute remaining distances
    double linear_dist = getDistance(target_pose_, actual_pose);
    double angular_dist = getShortestAngle(target_pose_.theta, actual_pose.theta);

    
    Velocity nextVelocity;
  	vel = l_max_vel_ * (linear_dist/4);
  	vel = vel > l_max_vel_ ? l_max_vel_ : vel;
  	theta = atan2(dy,dx);
  	vel_x = vel * cos(theta);
  	vel_y = vel * sin(theta);
  	gama = a_max_vel_ * angular_dist / (2*M_PI);
  	gama = gama > a_max_vel_ ? a_max_vel_ : gama;

    if (std::abs(linear_dist) < l_tolerance_)
    {
    	linear_complete_ = true;
    	nextVelocity.x = 0.0;
    	nextVelocity.y = 0.0;
  	}
  	else
  	{
  		nextVelocity.x = vel_x;
  		nextVelocity.y = vel_y;
  	}

  	if(std::abs(angular_dist) < a_tolerance_)
  	{
  		angular_complete_ = true;
  		nextVelocity.theta = 0.0;
  	}
  	else
  	{
  		nextVelocity.theta = gama;
  	}
  	
  	return nextVelocity;
}


//==================================================================
