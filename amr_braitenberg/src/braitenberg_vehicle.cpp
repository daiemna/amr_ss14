#include "braitenberg_vehicle.h"
#include <ros/console.h>
BraitenbergVehicle::BraitenbergVehicle()
: type_(TYPE_A)
, factor1_(1.0)
, factor2_(0.0)
{
}

BraitenbergVehicle::BraitenbergVehicle(Type type, float factor1, float factor2)
: type_(type)
, factor1_(factor1)
, factor2_(factor2)
{
}

void BraitenbergVehicle::computeWheelSpeeds(float left_in, float right_in, float& left_out, float& right_out)
{
  // Daiem Nadir Ali
  //==================== YOUR CODE HERE ====================
  // Instructions: based on the input from the left and
  //               right sonars compute the speeds of the
  //               wheels. Use the parameters stored in the
  //               private fields type_, factor1_, and
  //               factor2_ (if applicable).


  // =======================================================
  // ROS_INFO("20140405: left_in %2.2f right_in %2.2f",left_in,right_in);
  //changing the sonar sensitivity by factor supplied by rqt GUI
	if(type_ == BraitenbergVehicle::TYPE_A)
  {
    left_out = left_in * factor1_; 
    right_out = right_in * factor1_; 
  }
  else if(type_ == BraitenbergVehicle::TYPE_B)
  {
    left_out = right_in * factor1_; 
    right_out = left_in * factor1_; 
  }
  else
  {
    left_out = left_in * factor1_ + right_in * factor2_; 
    right_out = right_in * factor1_ + left_in * factor2_;
  }

  // ROS_INFO("20140405: left_in %2.2f right_in %2.2f",left_in,right_in);
	// }
}
