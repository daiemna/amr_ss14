#include "omni_velocity_controller.h"

//========================= YOUR CODE HERE =========================
// Instructions: implement all the functions that you have declared
//               in the header file.



OmniVelocityController::OmniVelocityController(double l_max_vel, double l_tolerance,
                                               double a_max_vel, double a_tolerance)
: l_max_vel_(l_max_vel)
, l_tolerance_(l_tolerance)
, a_max_vel_(a_max_vel)
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
	// TODO: implement omni drive here!
}


//==================================================================
