#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import rospy
import roslib
import math
roslib.load_manifest(PACKAGE)
import smach
from preemptable_state import PreemptableState
from math import copysign
from types import MethodType
from geometry_msgs.msg import Twist

__all__ = ['construct']

#=============================== YOUR CODE HERE ===============================
# Instructions: write a function for each state of wallfollower state machine.
#               The function should have exactly one argument (userdata
#               dictionary), which you should use to access the input ranges
#               and to provide the output velocity.
#               The function should have at least one 'return' statement, which
#               returns one of the possible outcomes of the state.
#               The function should not block (i.e. have infinite loops), but
#               rather it should implement just one iteration (check
#               conditions, compute velocity), because it will be called
#               regularly from the state machine.
#
# Hint: below is an example of a state that moves the robot forward until the
#       front sonar readings are less than the desired clearance. It assumes
#       that the corresponding variables ('front_min' and 'clearance') are
#       available in the userdata dictionary.
#

# def search(ud):
#     rospy.loginfo("Mode is {0}".format(ud.mode));
#     rospy.loginfo("min distance is: {0}".format(ud.clearance));
#     rospy.loginfo("distance Left: {0}".format(ud.front_min));

#     if ud.front_min < ud.clearance:
#         ud.velocity = (0,0,0)
#         return 'found_obstacle'
#     ud.velocity = (0.1, 0, 0)

def charge(ud):
    if ud.front_min < ud.clearance:
        if ud.front_alignment < ud.angularClearance and ud.back_alignment < ud.angularClearance :
            return 'found_corner';
        ud.velocity = (0,0,0)
        return 'found_obstacle_infront'
    ud.velocity = (1, 0, 0)

#rotate till robot is aligned with wall    
def rotate(ud):
    # rospy.logwarn("front {0},back {1},clearance {2}".format(ud.front_alignment,ud.back_alignment,ud.angularClearance));
    if ud.front_alignment < ud.angularClearance and ud.back_alignment < ud.angularClearance:
        ud.velocity = (0,0,0);
        return 'front_back_alignment_done';
    else:
        if ud.front_alignment > ud.back_alignment and ud.front_alignment > angularClearance:
            ud.velocity = (0,0,-1*mode_sign(ud.mode));
            return 'repeat_state';
        elif ud.front_alignment < ud.back_alignment and ud.back_alignment > ud.angularClearance:
            ud.velocity = (0,0,1*mode_sign(ud.mode));
            return 'repeat_state';
        else:
            ud.velocity = (0,0,0);
            return 'front_back_alignment_done';

def corner():
    if ud.front_alignment < ud.angularClearance and ud.back_alignment < ud.angularClearance:
        #concave corner
        ud.velocity = (0,0,1*mode_sign(ud.mode));
        return 'rotate_bot';
    else:
        ud.velocity = (0,0,0);
        return "move_forward"


    # if ud.front_alignment <= ud.angularClearance:
    #     if ud.front_alignment < ud.angularClearance and ud.back_alignment < ud.angularClearance:
    #         ud.velocity = (0,0,0);
    #         return 'front_back_alignment_done'
    #     ud.velocity = (0,0,0);
    #     return 'front_alignment_done'

    # elif ud.back_alignment <= ud.angularClearance:
    #     if ud.front_alignment < ud.angularClearance and ud.back_alignment < ud.angularClearance:
    #         ud.velocity = (0,0,0);
    #         return 'front_back_alignment_done'
    #     ud.velocity = (0,0,0);
    #     return 'back_alignment_done'
    # ud.velocity = (0,0,1);

#==============================================================================

def mode_sign(mode):
    if(mode == 1):
        return 1;
    else:
        return -1;

def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback.
    """
    #============================= YOUR CODE HERE =============================
    # Instructions: store the ranges from a ROS message into the userdata
    #               dictionary of the state machine.
    #               'ranges' is a list or Range messages (that should be
    #               familiar to you by now). It implies that to access the
    #               actual range reading of, say, sonar number 3, you need to
    #               write:
    #
    #                   ranges[3].range
    #
    #               For example, to create an item called 'front_min', which
    #               contains the minimum between the ranges reported by the two
    #               front sonars, you would write the following:
    #
    #                   self.userdata.front_min = min(ranges[3].range, ranges[4].range)
    #
    # Hint: you can just store the whole array of the range readings, but to
    #       simplify the code in your state functions, you may compute
    #       additional values, e.g. the difference between the reading of the
    #       side sonars, or the minimum of all sonar readings, etc.
    #
    # Hint: you can access all the variables stored in userdata. This includes
    #       the current settings of the wallfollower (that is clearance and the
    #       mode of wallfollowing). Think about how you could make your state
    #       functions independent of wallfollowing mode by smart preprocessing
    #       of the sonar readings.

    #division by max range to normalize 
    # max_range = self.userdata.range_max_val;
    # if max_range = 1 disable the normalization;
    max_range = 1;
    # sonars for front clearance
    self.userdata.front_min = min(ranges[2].range/max_range,ranges[5].range/max_range,ranges[3].range/max_range, ranges[4].range/max_range);

    # sonars for side clearance either left or right sonar
    # will be assigned min(sonar_8,sonar_7) in case of right wall following
    self.userdata.side_min = min(ranges[self.userdata.rangeSonars[0]].range/max_range,
        ranges[self.userdata.rangeSonars[1]].range/max_range);

    # front anligenment will be assgined range of sonar 6 in case of right wall following
    self.userdata.front_alignment = ranges[self.userdata.alignmentSonars[0]].range/max_range;

    # back_alignment will be assgined range of sonar 9 in case of right wall following
    self.userdata.back_alignment = ranges[self.userdata.alignmentSonars[1]].range/max_range;

    #==========================================================================


def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    #division by max range to normalize 
    max_range = self.userdata.range_max_val;
    # if max_range = 1 disable the normalization;
    # max_range = 1;

    twist = Twist()
    twist.linear.x = self.userdata.velocity[0] * self.userdata.front_min/max_range;
    twist.linear.y = self.userdata.velocity[1] * (min(self.userdata.front_alignment,self.userdata.back_alignment)/max_range);
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    # twist.angular.z = self.userdata.velocity[2] * (max(self.userdata.front_alignment,self.userdata.back_alignment)/max_range);

    # -0.262 = sin(50) 50 is angle of front_alignment or back_alignment from robot reference position
    # twist.angular.z = self.userdata.velocity[2] * math.atan((-0.262 * min(self.userdata.front_alignment,self.userdata.back_alignment))/(self.userdata.front_min/2));
    rospy.logwarn("front {0},back {1},clearance {2}".format(self.userdata.front_alignment,self.userdata.back_alignment,self.userdata.angularClearance));
    twist.angular.z = self.userdata.velocity[2] * 0.1;

    # rospy.logwarn("angular clearance : {0} rotation {1}".format(self.userdata.angularClearance,twist.angular.z));
    #============================= YOUR CODE HERE =============================
    # Instructions: although this function is implemented, you may need to
    #               slightly tweak it if you decided to handle wallfolllowing
    #               mode in "the smart way".
    # Hint: state machine userdata is accessible in this function as well, for
    #       example you can read the current wallfollowing mode with
    #
    #           self.userdata.mode
    #


    #==========================================================================
    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client.
    """

    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    # 0.667  = cos(angle between 6 and 7 sonar) / if it left wall following
    self.userdata.angularClearance = self.userdata.clearance/0.667;
    # handel left and right wallfollowing here
    if self.userdata.mode == 0:
        #LEFT wallfollowing
        self.userdata.rangeSonars = [0,15];
        self.userdata.alignmentSonars = [1,14];
    else:
        #RIGTH wallfollowing
        self.userdata.rangeSonars = [8,7];
        self.userdata.alignmentSonars = [6,9];
    return config


def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.6
    sm.userdata.angularClearance = 1.0;
    sm.userdata.ranges = None
    sm.userdata.configChaged = False;
    sm.userdata.rangeSonars = [8,7];
    sm.userdata.alignmentSonars = [6,9];
    sm.userdata.range_max_val = 5;

    # Add states
    with sm:
        #=========================== YOUR CODE HERE ===========================
        # Instructions: construct the state machine by adding the states that
        #               you have implemented.
        #               Below is an example how to add a state:
        #
        # smach.StateMachine.add('SEARCH',
        # PreemptableState(Search,
        # input_keys=['front_min', 'clearance'],
        # output_keys=['velocity'],
        # outcomes=['found_obstacle']),
        # transitions={'found_obstacle': 'SEARCH'})
        #
        #               First argument is the state label, an arbitrary string
        #               (by convention should be uppercase). Second argument is
        #               an object that implements the state. In our case an
        #               instance of the helper class PreemptableState is
        #               created, and the state function in passed. Moreover,
        #               we have to specify which keys in the userdata the
        #               function will need to access for reading (input_keys)
        #               and for writing (output_keys), and the list of possible
        #               outcomes of the state. Finally, the transitions are
        #               specified. Normally you would have one transition per
        #               state outcome.
        #
        # Note: The first state that you add will become the initial state of
        #       the state machine.

        
        smach.StateMachine.add('CHARGE',
        PreemptableState(charge,
        input_keys=['front_min', 'clearance','angularClearance','front_alignment','back_alignment'],
        output_keys=['velocity'],
        outcomes=['found_obstacle_infront','found_corner']),
        transitions={'found_obstacle_infront': 'ROTATE',
                    'found_corner' : 'CORNER'});

        smach.StateMachine.add('ROTATE',
        PreemptableState(rotate,
        input_keys=['back_alignment', 'front_alignment', 'angularClearance', 'mode'],
        output_keys=['velocity'],
        outcomes=['front_back_alignment_done','repeat_state']),
        transitions={'front_back_alignment_done': 'CHARGE',
        'repeat_state':'ROTATE'});

        smach.StateMachine.add('CORNER',
        PreemptableState(corner,
        input_keys=['back_alignment', 'front_alignment', 'angularClearance', 'mode', 'front_min'],
        output_keys=['velocity'],
        outcomes=['move_forward','rotate_bot']),
        transitions={'move_forward': 'CHARGE',
        'rotate_bot':'ROTATE'});

        # smach.StateMachine.add('ROTATE',
        # PreemptableState(rotate,
        # input_keys=['back_alignment', 'front_alignment', 'angularClearance'],
        # output_keys=['velocity'],
        # outcomes=['front_back_alignment_done','front_alignment_done', 'back_alignment_done']),
        # transitions={'front_back_alignment_done': 'CHARGE',
        # 'front_alignment_done':'ROTATE',
        # 'back_alignment_done':'ROTATE'});

        #======================================================================
    return sm
