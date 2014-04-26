#!/usr/bin/env python

import rospy
import math
import planar
from planar import Point, Vec2, EPSILON
from planar.c import Line
from math import degrees


#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specification and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'

class BugBrain:

    
    # declearing constants
    def LEFT_WALLFOLLOWING(self):
        return 0;
    def RIGHT_WALLFOLLOWING(self):
        return 1;
    def LEFT_SIDE(self):
        return -1;
    def RIGHT_SIDE(self):
        return 1;
    def TOLERANCE(self):
        return planar.EPSILON;

    def __init__(self, goal_x, goal_y, side):
        self.wall_side = side;
        self.wp_destination = Point(goal_x, goal_y);
        self.path_started = False;
        #the tolerence == planar.EPSILON at default value does not work good
        planar.set_epsilon(0.3);

    # method to determin if the destenation is on opposit side of wall being followed.
    # @param: distance
    #         signed distance from the robot to goal.
    #         can be obtained by path_line.distance_to(ROBOT CURRENT POSITION).
    def is_destination_opposite_to_wall(self,distance):
        direction = math.copysign(1,distance);

        if(self.wall_side == self.LEFT_WALLFOLLOWING()):
            if(direction == self.RIGHT_SIDE()):
                return True;
            else:
                return False;
        else:
            if(direction == self.LEFT_SIDE()):
                return True;
            else:
                return False;

    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        # compute and store necessary variables
        theta = degrees(theta);
        position = Point(x,y);

        self.ln_path = Line.from_points([position,self.wp_destination]);
        # saving where it started wall following
        self.wp_wf_start = position;

        pass

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        # compute and store necessary variables
        self.path_started = False;
        pass

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        # if the robot goes around an obstacle and
        # reaches the starting point and the destenation is still not reached then
        # the goal is unreachable.
        distance_to_path= self.ln_path.distance_to(Point(x,y));

        if(abs(distance_to_path) < self.TOLERANCE() and Vec2(x,y).almost_equals(self.wp_wf_start) and self.path_started):
            rospy.logwarn("UNREACHABLE POINT!");
            return True

        return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """

        theta = degrees(theta);
        self.current_theta  =theta;

        self.wp_current_position = Point(x,y);
        self.current_direction = Vec2.polar(angle = theta,length = 1);
        #Robot Orientation Line.
        self.ln_current_orentation = Line(Vec2(x,y),self.current_direction);

        # the prependicular line to the path
        self.ln_distance = self.ln_path.perpendicular(self.wp_current_position);
        
        
        distance_to_path= self.ln_path.distance_to(Point(x,y));
        self.distance_to_path = distance_to_path;
        distance_to_destination = self.ln_current_orentation.distance_to(self.wp_destination);
        if(abs(distance_to_path) > 1):
            self.path_started =True;

        """
        checking if distance to the straight path is approx. 0 and
        if destenation on the opposit side of wall then leave the path
        NOTE and TODO: works only for the circles not for complex path.
        """
        if(abs(distance_to_path) < self.TOLERANCE() and
         self.is_destination_opposite_to_wall(distance_to_destination)):
            self.wp_wf_stop = Point(x,y);
            return True;

        return False

#==============================================================================