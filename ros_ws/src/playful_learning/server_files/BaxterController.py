
from __future__ import print_function

import rospy
import baxter_interface
from baxter_interface import RobotEnable, settings
from BaxterTrajectory import BaxterTrajectory

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_core_msgs.msg import EndEffectorCommand
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Int32, UInt16

import struct
import json
from time import sleep
from time import time as curTime
from collections import OrderedDict

from threading import Thread

import math
import sys
sys.path.insert(0, '/home/drl/ros_ws/src')

from helpers import *

##############################################
##############################################
# BaxterController Class
##############################################
##############################################

# s0: increasing moves outward, -45 is straight in front
# w0: increasing moves inward, 0 is straight out from arm

# s1: increasing moves arm down, 0 is horizontal
# e1: increasing moves arm down, 0 is horizontal
# w1: increasing moves arm down, 0 is straight to arm

# for straight wrist, want s1+e1+w1 = 0

class BaxterController:
  # initialize the baxter controller
  # @param should_print [bool] whether to print status updates by default (most methods have override should_print parameters too)
  def __init__(self, print_debug=False):
    self._print_debug = print_debug
    self._print('initializing BaxterController')
    try: # node may already be initialized by someone using this class
      rospy.init_node('baxterController')
    except Exception as e:
      errMessage = str(e)
      if 'init_node' in errMessage.lower() and 'already been called' in errMessage.lower():
        pass
      else:
        raise
    self._baxter = RobotEnable()

    # Set up the limbs and grippers.
    self._limb_names = ['left', 'right']
    self._joint_names_noLimb = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    self._joint_names = OrderedDict()
    self._limbs = OrderedDict()
    self._neutral_joint_angles_rad = OrderedDict()
    self._resting_joint_angles_rad = OrderedDict()
    self._absolute_torque_limits = OrderedDict()
    self._should_abort_move_to_joint_angles_rad = OrderedDict()
    self._should_abort_trajectory = OrderedDict()
    self._move_to_joint_angles_rad_thread = OrderedDict()
    self._move_to_joint_angles_rad_in_progress = OrderedDict()
    self._trajectory_in_progress = OrderedDict()
    self._trajectory_thread = OrderedDict()
    self._trajectories = OrderedDict()
    self._grippers = OrderedDict()
    self._max_gripper_force_percent = 40
    for limb_name in self._limb_names:
      self._limbs[limb_name] = baxter_interface.Limb(limb_name)
      self._joint_names[limb_name] = self.prepend_limb_name(limb_name, self._joint_names_noLimb)
      self._neutral_joint_angles_rad[limb_name] = None # will use Baxter's default neutral position
      self._resting_joint_angles_rad[limb_name] = self.prepend_limb_name(limb_name, OrderedDict([
                                                              ('s0', -0.80 if limb_name == 'left' else 0.80),
                                                              ('s1', -0.25),
                                                              ('e0',  0.00),
                                                              ('e1',  1.50),
                                                              ('w0',  0.00),
                                                              ('w1', -1.30),
                                                              ('w2', 0 if limb_name == 'left' else -1.5), ]))
      self._absolute_torque_limits[limb_name] = self.prepend_limb_name(limb_name, OrderedDict([
                                                          ('s0', [-50, 50]),
                                                          ('s1', [-50, 50]),
                                                          ('e0', [-50, 50]),
                                                          ('e1', [-50, 50]),
                                                          ('w0', [-15, 15]),
                                                          ('w1', [-15, 15]),
                                                          ('w2', [-15, 15]), ]))
      # Store a trajectory in case the user would like to use it.
      self._trajectories[limb_name] = BaxterTrajectory(limb_name=limb_name)
      self._trajectory_thread[limb_name] = None
      rospy.on_shutdown(self._trajectories[limb_name].stop)
      # Store placeholders for move-to-joint-angles threads.
      self._should_abort_move_to_joint_angles_rad[limb_name] = False
      self._should_abort_trajectory[limb_name] = False
      self._move_to_joint_angles_rad_thread[limb_name] = None
      self._move_to_joint_angles_rad_in_progress[limb_name] = False
      self._trajectory_in_progress[limb_name] = False
      # Set up the grippers.
      self._grippers[limb_name] = baxter_interface.Gripper(limb_name)
      self._grippers[limb_name].calibrate()
      self._grippers[limb_name].set_holding_force(25) # percent (default: 30)
      self._grippers[limb_name].set_moving_force(25)  # percent (default: 30)
    
    # Enable Baxter.
    self.enable_baxter()
    self.set_baxter_sonar_state(0)
    self.set_state_publish_rate(100)
    
    self._print('BaxterController initialization complete')
  
  def quit(self, should_print=None):
    self._print('BaxterController quitting!', should_print=should_print)
    self._print('BaxterController successfully quit', should_print=should_print)
    
  def _print(self, msg, should_print=None):
    should_print = self._print_debug if should_print is None else should_print
    if should_print:
      print(msg)
    
    
  ##############################################
  # Baxter state
  ##############################################
  
  """
  enable baxter
  """
  def enable_baxter(self, should_print=None):
    self._print('Enabling Baxter', should_print=should_print)
    self._baxter.enable()
    if not self._baxter.state().enabled:
      self._print('  Error enabling Baxter... retrying', should_print=should_print)
      self._baxter.reset()
      self._baxter.enable()
      if not self._baxter.state().enabled:
        raise AssertionError('Baxter did not enable properly')
  
  """
  disable baxter
  """
  def disable_baxter(self, move_to_resting=False, prompt_before_disabling=False, should_print=None):
    self.abort_movement()
    if move_to_resting:
      self.move_to_resting()
      self.wait_for_movement_completion()
    if prompt_before_disabling:
      entry = raw_input('Press Enter to disable Baxter (or q to abort): ').strip()
      if entry.lower() == 'q':
        return
    self._print('Disabling Baxter', should_print=should_print)
    self._baxter.disable()
    if self._baxter.state().enabled:
      self._print('  Error disabling... retrying', should_print=should_print)
      self._baxter.reset()
      self._baxter.disable()
      if self._baxter.state().enabled:
        raise AssertionError('Baxter did not disable properly')
  
  """
  enable or disable baxter
  """
  def set_baxter_enabled_state(self, state, should_print=None):
    if state:
      self.enable_baxter(should_print=should_print)
    else:
      self.disable_baxter(should_print=should_print)
  
  """
  enable or disable sonar
  """
  def set_baxter_sonar_state(self, state, should_print=None):
    self._print('Setting sonar state to %s' % (state), should_print=should_print)
    pub = rospy.Publisher('/robot/sonar/head_sonar/set_sonars_enabled', UInt16, queue_size=10)
    sleep(0.5)
    pub.publish(1 if state else 0)
    sleep(0.5)
    
  """
  set state publish rate (default 100, max 1000)
  """
  def set_state_publish_rate(self, rate_hz=100, should_print=None):
    rate_hz = min(rate_hz, 1000)
    rate_hz = max(rate_hz, 1)
    self._print('Setting state publish rate to %g Hz' % (rate_hz), should_print=should_print)
    pub = rospy.Publisher('/robot/joint_state_publish_rate', UInt16, queue_size=10)
    sleep(0.5)
    pub.publish(rate_hz)
    sleep(0.5)
    
  """
  get a dict of torque limits, based on Baxter's inherent limitations
  @param scaleFactor [float] how much to scale the built-in torque limits (probably a number between 0 and 1)
  """
  def get_torque_limits(self, scale_factor=1, should_print=None):
    limits  = self._absolute_torque_limits
    limits = scale(limits, scale_factor)
    self._print('Got scaled torque limits [Nm]: %s' % getVariableStr(limits), should_print=should_print)
    return limits
    
    
  ##############################################
  # Baxter position and effort
  ##############################################
  
  """
  get the current neutral angles angles of each joint
  @return [OrderedDict] map from limb name to a dict mapping joint name to joint angle (in radians)
  """
  def get_neutral_joint_angles_rad(self, should_print=None):
    neutral_angles_rad = self._neutral_joint_angles_rad
    for limb_name in neutral_angles_rad:
      if neutral_angles_rad[limb_name] is None: # using Baxter's default neutral (-30, 43, 72 deg)
        neutral_angles_rad[limb_name] = OrderedDict([('s0', 0), ('s1', -0.55), ('e0', 0), ('e1', 0.75), ('w0', 0), ('w1', 1.26), ('w2', 0)])
        neutral_angles_rad[limb_name] = self.prepend_limb_name(limb_name, neutral_angles_rad[limb_name])
      self._print('See neutral joint angles [deg]: %s' % getVariableStr(rad2deg(neutral_angles_rad[limb_name])), should_print=should_print)
    return neutral_angles_rad
  
  """
  get the current resting angles angles of each joint
  @return [OrderedDict] map from limb name to a dict mapping joint name to joint angle (in radians)
  """
  def get_resting_joint_angles_rad(self, should_print=None):
    resting_angles_rad = self._resting_joint_angles_rad
    for limb_name in resting_angles_rad:
      self._print('See resting joint angles [deg]: %s' % getVariableStr(rad2deg(resting_angles_rad[limb_name])), should_print=should_print)
    return resting_angles_rad
  
  """
  get the angles of each joint
  @return [OrderedDict] map from limb name to a dict mapping joint name to joint angle (in radians)
  """
  def get_joint_angles_rad(self, should_print=None):
    joint_angles_rad = OrderedDict()
    for limb_name in self._limbs:
      joint_angles_rad[limb_name] = self._limbs[limb_name].joint_angles()
      joint_angles_rad[limb_name] = OrderedDict([(jointName, joint_angles_rad[limb_name][jointName]) for jointName in self._joint_names[limb_name]])
      self._print('See current joint angles [deg]: %s' % getVariableStr(rad2deg(joint_angles_rad[limb_name])), should_print=should_print)
    return joint_angles_rad
    
  """
  get the velocities of each joint
  @return [OrderedDict] map from limb name to a dict mapping joint name to joint velocity (in radians/sec)
  """
  def get_joint_velocities_rad_s(self, should_print=None):
    joint_velocities_rad_s = OrderedDict()
    for limb_name in self._limbs:
      joint_velocities_rad_s[limb_name] = self._limbs[limb_name].joint_velocities()
      joint_velocities_rad_s[limb_name] = OrderedDict([(jointName, joint_velocities_rad_s[limb_name][jointName]) for jointName in self._joint_names[limb_name]])
      self._print('See joint velocities [deg/s]: %s' % getVariableStr(rad2deg(joint_velocities_rad_s[limb_name])), should_print=should_print)
    return joint_velocities_rad_s
    
  """
  get the efforts of each joint
  @return [OrderedDict] map from limb name to a dict mapping joint name to joint effort (in Nm)
  """
  def get_joint_efforts_Nm(self, should_print=None):
    joint_efforts_Nm = OrderedDict()
    for limb_name in self._limbs:
      joint_efforts_Nm[limb_name] = self._limbs[limb_name].joint_efforts()
      joint_efforts_Nm[limb_name] = OrderedDict([(jointName, joint_efforts_Nm[limb_name][jointName]) for jointName in self._joint_names[limb_name]])
      self._print('See joint efforts [Nm]: %s' % getVariableStr(joint_efforts_Nm[limb_name]), should_print=should_print)
    return joint_efforts_Nm
    
  """
  set the joint angles to use as 'neutral'
  if this is never called, Baxter's default neutral will be used (midpoint of each joint range)
  @param limb_name 'left' or 'right'
  @param joint_angles_rad [list, tuple, dict, OrderedDict] the angles to record (in radians)
    if a list or tuple, assumes it corresponds to the joints in self._joint_names[limb_name]
    if a dict, assumes is map from joint name to desired angle (unspecified joints will not be set)
  @return [OrderedDict] map from joint name to recorded joint angle
  @raise [AssertionError] if input angles is not of an appropriate format
  """
  def set_neutral_joint_angles_rad(self, limb_name, joint_angles_rad, should_print=None):
    # parse the angles into a dict
    if isinstance(joint_angles_rad, (list, tuple)):
      if len(joint_angles_rad) != len(self._joint_names[limb_name]):
        raise AssertionError('Invalid argument given to set_neutral_joint_angles_rad')
      joint_angles_rad = OrderedDict([(jointName, joint_angles_rad[i]) for i, jointName in enumerate(self._joint_names[limb_name])])
    if not isinstance(joint_angles_rad, dict):
      raise AssertionError('Invalid argument given to set_neutral_joint_angles_rad')
    # record the joint angles
    self._neutral_joint_angles_rad[limb_name] = joint_angles_rad
    self._print('Saved neutral joint angles [deg]: %s' % getVariableStr(rad2deg(joint_angles_rad)), should_print=should_print)
    return joint_angles_rad

  """
  set the joint angles to use as 'resting'
  @param limb_name 'left' or 'right'
  @param joint_angles_rad [list, tuple, dict, OrderedDict] the angles to record (in radians)
    if a list or tuple, assumes it corresponds to the joints in self._joint_names[limb_name]
    if a dict, assumes is map from joint name to desired angle (unspecified joints will not be set)
  @return [OrderedDict] map from joint name to recorded joint angle
  @raise [AssertionError] if input angles is not of an appropriate format
  """
  def set_resting_joint_angles_rad(self, limb_name, joint_angles_rad, should_print=None):
    # parse the angles into a dict
    if isinstance(joint_angles_rad, (list, tuple)):
      if len(joint_angles_rad) != len(self._joint_names[limb_name]):
        raise AssertionError('Invalid argument given to set_resting_joint_angles_rad')
      joint_angles_rad = OrderedDict(
        [(jointName, joint_angles_rad[i]) for i, jointName in enumerate(self._joint_names[limb_name])])
    if not isinstance(joint_angles_rad, dict):
      raise AssertionError('Invalid argument given to set_resting_joint_angles_rad')
    # record the joint angles
    self._resting_joint_angles_rad[limb_name] = joint_angles_rad
    self._print('Saved resting joint angles [deg]: %s' % getVariableStr(rad2deg(joint_angles_rad)),
                should_print=should_print)
    return joint_angles_rad

  """
  set the joint angles (non-blocking by default)
  @param joint_angles_rad dict mapping limb name to a [list, tuple, dict, OrderedDict] which is the angles to apply (in radians)
    if a list or tuple, assumes it corresponds to the joints in self._joint_names[limb_name]
    if a dict, assumes is map from joint name to desired angle (unspecified joints will not be set)
  @return [OrderedDict] map from limb name to a mapping from joint name to applied joint angle (in radians)
  @raise [AssertionError] if input angles is not of an appropriate format
  """
  def move_to_joint_angles_rad(self, joint_angles_rad_byLimb, timeout_s=15, 
                               tolerance_rad=settings.JOINT_ANGLE_TOLERANCE, should_print=None):
    for limb_name in joint_angles_rad_byLimb:
      joint_angles_rad = joint_angles_rad_byLimb[limb_name]
      # parse the angles into a dict
      if isinstance(joint_angles_rad, (list, tuple)):
        if len(joint_angles_rad) != len(self._joint_names[limb_name]):
          raise AssertionError('Invalid argument given to setJointAngles')
        joint_angles_rad = OrderedDict([(jointName, joint_angles_rad[i]) for i, jointName in enumerate(self._joint_names[limb_name])])
      if not isinstance(joint_angles_rad, dict):
        raise AssertionError('Invalid argument given to setJointAngles')
      joint_angles_rad_byLimb[limb_name] = joint_angles_rad
      # move the limb if a blocking call is desired
      # use move_to_joint_positions which creates a safe interpolation from current to target angles
      self._print('Moving to joint angles [deg]: %s' % getVariableStr(rad2deg(joint_angles_rad)), should_print=should_print)
      # if wait_for_each_limb_completion:
      #   self._limbs[limb_name].move_to_joint_positions(joint_angles_rad, timeout=timeout_s, threshold=tolerance_rad)
      #   # self._limbs[limb_name].set_joint_positions(joint_angles_rad, raw=False) # fewer safety measures included in this call
      # if non-blocking is desired, create a thread so it can be aborted with via its "test" callback
      # else:
      def worker(limb_name, joint_angles_rad):
        rospy.loginfo("Starting smooth async movement...")
        try:
          self._limbs[limb_name].move_to_joint_positions(
              joint_angles_rad, 
              timeout=timeout_s, 
              threshold=tolerance_rad,
              test=lambda: self._should_abort_move_to_joint_angles_rad_callback(limb_name)
          )
        finally:
          self._move_to_joint_angles_rad_in_progress[limb_name] = False
        rospy.loginfo("Movement thread has exited.")
      self.abort_movement(limb_name)
      self._should_abort_move_to_joint_angles_rad[limb_name] = False
      self._move_to_joint_angles_rad_in_progress[limb_name] = True
      self._move_to_joint_angles_rad_thread[limb_name] = Thread(target=worker, args=(limb_name, joint_angles_rad))
      self._move_to_joint_angles_rad_thread[limb_name].daemon = True
      self._move_to_joint_angles_rad_thread[limb_name].start()
    return joint_angles_rad_byLimb
  def _should_abort_move_to_joint_angles_rad_callback(self, limb_name):
      return self._should_abort_move_to_joint_angles_rad[limb_name]
  def abort_movement(self, limb_name=None):
    if limb_name is None:
      for limb_name in self._limb_names:
        self.abort_movement(limb_name)
      return
    if self._move_to_joint_angles_rad_thread[limb_name] and self._move_to_joint_angles_rad_thread[limb_name].is_alive():
      self._should_abort_move_to_joint_angles_rad[limb_name] = True
      self._move_to_joint_angles_rad_thread[limb_name].join(timeout=30)
      self._should_abort_move_to_joint_angles_rad[limb_name] = False
    if self._trajectory_thread[limb_name] and self._trajectory_thread[limb_name].is_alive():
      self._should_abort_trajectory[limb_name] = True
      self._trajectory_thread[limb_name].join(timeout=30)
      self._should_abort_trajectory[limb_name] = False
    self.stop_gripper(limb_name)
  def is_movement_in_progress(self, limb_name=None):
    if limb_name is None:
      return True in [self.is_movement_in_progress(limb_name=limb_name) for limb_name in self._limb_names]
    return (
      self._move_to_joint_angles_rad_in_progress[limb_name] 
      or self._trajectory_in_progress[limb_name]
      or self.is_gripper_moving(limb_name)
    )
  def wait_for_movement_completion(self, limb_name=None, timeout_s=15):
    t0 = time.time()
    while self.is_movement_in_progress(limb_name) and time.time() - t0 < timeout_s:
      time.sleep(0.1)
    return not self.is_movement_in_progress(limb_name)
  
  """
  move to the neutral position
  """
  def move_to_neutral(self, limb_name=None, should_print=None):
    if limb_name is None:
      for limb_name in self._limb_names:
        self.move_to_neutral(limb_name=limb_name, should_print=should_print)
      return
    self._print('Moving to neutral for %s' % limb_name, should_print=should_print)
    if self._neutral_joint_angles_rad[limb_name] is not None:
      self.move_to_joint_angles_rad({limb_name: self._neutral_joint_angles_rad[limb_name]}, should_print=should_print)
    else:
      self._move_to_joint_angles_rad_in_progress[limb_name] = True
      self.abort_movement(limb_name)
      self._limbs[limb_name].move_to_neutral()
      self._move_to_joint_angles_rad_in_progress[limb_name] = False
    
  """
  move to the resting position (usually preparation for disabling)
  """
  def move_to_resting(self, limb_name=None, should_print=None):
    if limb_name is None:
      for limb_name in self._limb_names:
        self.move_to_resting(limb_name=limb_name, should_print=should_print)
      return
    self._print('Moving to resting for %s' % limb_name, should_print=should_print)
    if self._resting_joint_angles_rad[limb_name] is not None:
      self.move_to_joint_angles_rad({limb_name: self._resting_joint_angles_rad[limb_name]}, should_print=should_print)
    else:
      self._print('  *** No resting angles have been set', should_print=should_print)
  
  """
  set the gripper pose (non-blocking by default)
  @return [OrderedDict] map from joint name to applied joint angle (in radians)
  @raise [AssertionError] if input angles is not of an appropriate format
  """
  def move_to_gripper_pose(self, gripper_position_m_byLimb,
                           gripper_orientation_quaternion_wijk_byLimb,
                           timeout_s=15,
                           seed_joint_angles_rad_byLimb=None,
                           tolerance_rad=settings.JOINT_ANGLE_TOLERANCE, should_print=None):
    if seed_joint_angles_rad_byLimb is None:
      seed_joint_angles_rad_byLimb = dict([(limb_name, None) for limb_name in gripper_position_m_byLimb])
    found_joint_angles_byLimb = {}
    for limb_name in gripper_position_m_byLimb:
      # Compute joint angles.
      joint_angles_rad = self.get_joint_angles_rad_for_gripper_pose(
        limb_name,
        gripper_position_m_byLimb[limb_name],
        gripper_orientation_quaternion_wijk_byLimb[limb_name],
        seed_joint_angles_rad=seed_joint_angles_rad_byLimb[limb_name],
        nullspace_goal_position=None,
        should_print=should_print)
      # Move to the position.
      if joint_angles_rad is not None:
        found_joint_angles_byLimb[limb_name] = True
        self.move_to_joint_angles_rad({limb_name: joint_angles_rad},
                                      timeout_s=timeout_s,
                                      tolerance_rad=tolerance_rad, should_print=should_print)
      else:
        found_joint_angles_byLimb[limb_name] = False
    return found_joint_angles_byLimb
    
  ##############################################
  # Inverse kinematics
  ##############################################
  
  """
  Get joint angles for a given gripper position and orientation.
  @param gripper_position_m
  @param gripper_orientation_quaternion_wijk
  """
  def get_joint_angles_rad_for_gripper_pose(self, limb_name,
                                            gripper_position_m,
                                            gripper_orientation_quaternion_wijk,
                                            seed_joint_angles_rad=None,
                                            nullspace_goal_position=None,
                                            should_print=None):
    ns = "ExternalTools/%s/PositionKinematicsNode/IKService" % limb_name
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # Add desired pose to compute.
    ikreq.pose_stamp.append(PoseStamped(
                              header=hdr,
                              pose=Pose(
                                position=Point(
                                  x=gripper_position_m[0],
                                  y=gripper_position_m[1],
                                  z=gripper_position_m[2],
                                ),
                                orientation=Quaternion(
                                  x=gripper_orientation_quaternion_wijk[1],
                                  y=gripper_orientation_quaternion_wijk[2],
                                  z=gripper_orientation_quaternion_wijk[3],
                                  w=gripper_orientation_quaternion_wijk[0],
                                ),
                              )))
    # Request inverse kinematics from base to hand link of the target limb.
    #ikreq.tip_names.append('%s_hand' % limb_name)
    if seed_joint_angles_rad is not None:
      # Parse the angles into a list of angles
      if isinstance(seed_joint_angles_rad, (list, tuple)):
        if len(seed_joint_angles_rad) != len(self._joint_names[limb_name]):
          raise AssertionError('Invalid seed given to get_joint_angles_rad_for_gripper_pose')
        seed_joint_angles_rad = OrderedDict([(joint_name, seed_joint_angles_rad[i]) for i, joint_name in enumerate(self._joint_names[limb_name])])
      elif isinstance(seed_joint_angles_rad, dict):
        seed_joint_angles_rad = OrderedDict([(joint_name, seed_joint_angles_rad[joint_name]) for joint_name in self._joint_names[limb_name]])
      else:
        raise AssertionError('Invalid seed given to get_joint_angles_rad_for_gripper_pose')
      # Add the seed.
      ikreq.seed_mode = ikreq.SEED_USER
      seed = JointState()
      seed.name = list(seed_joint_angles_rad.keys())
      seed.position = list(seed_joint_angles_rad.values())
      ikreq.seed_angles.append(seed)
    #if nullspace_goal_position is not None:
    #  # Once the primary IK task is solved, the solver will then try to bias
    #  # the joint angles toward the goal joint configuration. The null space is
    #  # the extra degrees of freedom the joints can move without affecting the
    #  # primary IK task.
    #  ikreq.use_nullspace_goal.append(True)
    #  # The nullspace goal can either be the full set or subset of joint angles
    #  goal = JointState()
    #  goal.name = ['right_j1', 'right_j2', 'right_j3']
    #  goal.position = nullspace_goal_position #[0.1, -0.3, 0.5]
    #  ikreq.nullspace_goal.append(goal)
    #  # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
    #  # If empty, the default gain of 0.4 will be used
    #  ikreq.nullspace_gain.append(0.4)
      
    # Send the request to compute a joint angle solution.
    try:
      self._print('Computing inverse kinematics for position %s orientation %s' % (str(gripper_position_m), str(gripper_orientation_quaternion_wijk)))
      rospy.wait_for_service(ns, 5.0)
      resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
      rospy.logerr('Service call failed: %s' % (e,))
      self._print('Error computing inverse kinematics', should_print=should_print)
      self._print('%s' % (e), should_print=should_print)
      return None

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if resp_seeds[0] != resp.RESULT_INVALID:
      seed_str = {
                  ikreq.SEED_USER: 'User Provided Seed',
                  ikreq.SEED_CURRENT: 'Current Joint Angles',
                  ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                 }.get(resp_seeds[0], 'None')
      self._print('Found valid IK joint solution from seed type: %s' % (seed_str,), should_print=should_print)
      # Format solution into Limb API-compatible dictionary
      joint_angles_rad = dict(zip(resp.joints[0].name, resp.joints[0].position))
      joint_angles_rad = OrderedDict([(joint_name, joint_angles_rad[joint_name]) for joint_name in self._joint_names[limb_name]])
      return joint_angles_rad
    else:
      self._print('Error computing inverse kinematics: invalid pose', should_print=should_print)
      return None
  
  ##############################################
  # Trajectories
  ##############################################
  
  """
  Build a trajectory that achieves a sequence of joint angles at specified times.
  @param times_from_start_s list of times at which each position should be achieved, as seconds since trajectory start
  @param joint_angles_rad list of [list, tuple, dict, OrderedDict] the angles to apply (in radians)
    Each entry corresponds to a position for that timestep.
    If an entry is a list or tuple, assumes it corresponds to the joints in self._jointNames
    If an entry is a dict, assumes it maps from joint name to desired angle (unspecified joints will not be set)
  @raise [AssertionError] if input angles is not of an appropriate format
  """
  def build_trajectory_from_joint_angles(self, limb_name, times_from_start_s, joint_angles_rad, goal_time_tolerance_s=0.1, should_print=None):
    if not isinstance(times_from_start_s, (list, tuple)) \
        or not isinstance(joint_angles_rad, (list, tuple)) \
        or len(times_from_start_s) != len(joint_angles_rad):
      raise AssertionError('Invalid arguments given to build_trajectory')
    
    # Clear the current trajectory.
    self._print('Clearing the trajectory', should_print=should_print)
    self._trajectories[limb_name].set_goal_time_tolerance_s(goal_time_tolerance_s)
    self._trajectories[limb_name].clear()
    
    # Add the desired points.
    for timestep_index in range(len(times_from_start_s)):
      joint_angles_rad_forTimestep = joint_angles_rad[timestep_index]
      # Parse the angles into a list of angles
      if isinstance(joint_angles_rad_forTimestep, (list, tuple)):
        if len(joint_angles_rad_forTimestep) != len(self._joint_names[limb_name]):
          raise AssertionError('Invalid argument given to setJointAngles')
        joint_angles_rad_forTimestep = OrderedDict([(joint_name, joint_angles_rad_forTimestep[i]) for i, joint_name in enumerate(self._joint_names[limb_name])])
      elif isinstance(joint_angles_rad_forTimestep, dict):
        joint_angles_rad_forTimestep = OrderedDict([(joint_name, joint_angles_rad_forTimestep[joint_name]) for joint_name in self._joint_names[limb_name]])
      else:
        raise AssertionError('Invalid joint angles given to build_trajectory')
      joint_angles_rad_forTimestep = list(joint_angles_rad_forTimestep.values())
      # Add the trajectory point.
      self._print('Adding trajectory point at %6.2f s [deg]: %s' % (times_from_start_s[timestep_index], getVariableStr(rad2deg(joint_angles_rad_forTimestep))), should_print=should_print)
      self._trajectories[limb_name].add_point(joint_angles_rad_forTimestep, times_from_start_s[timestep_index])
  
  """
  Build a trajectory that achieves a sequence of gripper poses at specified times.
  @param times_from_start_s list of times at which each position should be achieved, as seconds since trajectory start
  @param joint_angles_rad list of [list, tuple, dict, OrderedDict] the angles to apply (in radians)
    Each entry corresponds to a position for that timestep.
    If an entry is a list or tuple, assumes it corresponds to the joints in self._jointNames
    If an entry is a dict, assumes it maps from joint name to desired angle (unspecified joints will not be set)
  @raise [AssertionError] if input angles is not of an appropriate format
  """
  def build_trajectory_from_gripper_poses(self, limb_name, times_from_start_s,
                                          gripper_positions_m,
                                          gripper_orientations_quaternion_wijk,
                                          goal_time_tolerance_s=0.1,
                                          initial_seed_joint_angles_rad=None,
                                          should_print=None):
    # Compute inverse kinematics for each pose.
    joint_angles_rad = []
    seed_joint_angles_rad = initial_seed_joint_angles_rad
    for timestep_index in range(len(times_from_start_s)):
      # Seed the solver with the previous solution to help make motion more continuous/smooth.
      if timestep_index > 0:
        seed_joint_angles_rad = joint_angles_rad[timestep_index-1]
      # Compute joint angles for this timestep.
      joint_angles_rad.append(self.get_joint_angles_rad_for_gripper_pose(
        limb_name,
        gripper_position_m=gripper_positions_m[timestep_index],
        gripper_orientation_quaternion_wijk=gripper_orientations_quaternion_wijk[timestep_index],
        seed_joint_angles_rad=seed_joint_angles_rad, should_print=should_print))
      if joint_angles_rad[-1] is None:
        return False
    
    # Build a trajectory using the computed joint angles.
    self.build_trajectory_from_joint_angles(limb_name,
                                            times_from_start_s=times_from_start_s, joint_angles_rad=joint_angles_rad,
                                            goal_time_tolerance_s=goal_time_tolerance_s, should_print=should_print)
    return True
    
  """
  Run the current trajectory.
  @param wait_for_completion whether to wait for the trajectory to finish.
  """
  def run_trajectory(self, limb_names=None, should_print=None):
    if limb_names is None:
      limb_names = list(self._trajectories.keys())
    # Move each limb to the starting location.
    for limb_name in limb_names:
      self.move_to_trajectory_start(limb_name, should_print=should_print)
    self.wait_for_movement_completion(timeout_s=30)
    # Run the trajectories simultaneously.
    def worker(limb_name):
      self._print('Running the current trajectory for %s' % limb_name, should_print=should_print)
      self._trajectory_in_progress[limb_name] = True
      self._trajectories[limb_name].run(wait_for_completion=False)
      time.sleep(0.5)
      while self._trajectories[limb_name].is_running():
        time.sleep(0.100)
        if self._should_abort_trajectory[limb_name]:
          self._trajectories[limb_name].stop()
      self._trajectory_in_progress[limb_name] = False
      if not self.trajectory_succeeded(limb_name):
        result = self.get_trajectory_result_code(limb_name)
        self._print('ERROR running trajectory: code [%d], string [%s]' % (result.error_code, result.error_string))
    for limb_name in limb_names:
      self.abort_movement(limb_name)
      self._should_abort_trajectory[limb_name] = False
      self._trajectory_thread[limb_name] = Thread(target=worker, args=(limb_name,))
      self._trajectory_thread[limb_name].daemon = True
      self._trajectory_thread[limb_name].start()

  def get_trajectory_result_code(self, limb_name):
    if self._trajectories[limb_name].is_running():
      return None
    return self._trajectories[limb_name].result().error_code
  def trajectory_succeeded(self, limb_name):
    result_code = self.get_trajectory_result_code(limb_name)
    # SUCCESSFUL = 0
    # INVALID_GOAL = -1
    # INVALID_JOINTS = -2
    # OLD_HEADER_TIMESTAMP = -3
    # PATH_TOLERANCE_VIOLATED = -4
    # GOAL_TOLERANCE_VIOLATED = -5
    if result_code in [-4, -1, -2, -3]:
      return False
    else:
      # Note that have seen 0 and -5 after seemingly successful trajectories.
      return True

  def move_to_trajectory_start(self, limb_name, should_print=None):
    self._print('Moving to the trajectory start point for %s' % limb_name, should_print=should_print)
    self.move_to_joint_angles_rad({limb_name: self._trajectories[limb_name].get_joint_angles_rad(step_index=0)})
  
  def move_to_trajectory_index(self, limb_name, step_index, should_print=None):
    self._print('Moving to trajectory index %d' % step_index, should_print=should_print)
    self.move_to_joint_angles_rad({limb_name: self._trajectories[limb_name].get_joint_angles_rad(step_index=step_index)})

  ##############################################
  # Gripper
  ##############################################
  """
  Open or close the gripper.
  @param gripper_open_percent from 0 (closed) to 100 (open)
  @param force_threshold_percent the force at which to stop movement; from 0 to 100, with an internal maximum cutoff for safety
  """
  def move_gripper(self, limb_name, gripper_open_percent, force_threshold_percent=15):
    gripper_open_percent = max(0, min(100, gripper_open_percent))
    force_threshold_percent = max(0, min(self._max_gripper_force_percent, force_threshold_percent))
    self._grippers[limb_name].set_moving_force(force_threshold_percent)
    self._grippers[limb_name].set_holding_force(force_threshold_percent)
    self._grippers[limb_name].command_position(gripper_open_percent, block=False)

  def close_gripper(self, limb_name):
    self.move_gripper(limb_name, gripper_open_percent=0)

  def open_gripper(self, limb_name):
    self.move_gripper(limb_name, gripper_open_percent=100)
  
  def calibrate_gripper(self, limb_name):
    self._grippers[limb_name].calibrate()

  def get_gripper_position_open_percent(self, limb_name):
    return self._grippers[limb_name].position()
  
  def get_gripper_force_percent(self, limb_name):
    return self._grippers[limb_name].force()

  def stop_gripper(self, limb_name):
    if self._grippers[limb_name].moving():
      self._grippers[limb_name].stop(block=False)

  def is_gripper_grasping(self, limb_name):
    return self._grippers[limb_name].gripping()

  def is_gripper_moving(self, limb_name):
    return self._grippers[limb_name].moving()
  
  ##############################################
  # Miscellaneous helper functions
  ##############################################
  
  """
  append limb name to joint names
  will do so for all keys if a dict, all items if a list, or simply the given variable otherwise
  example, if given 's0' will output 'left_s0'
  """
  def prepend_limb_name(self, limb_name, invar):
    return addString(invar, '%s_' % limb_name, append=False)
    

##############################################
##############################################
# code to run if running this script on its own
##############################################
##############################################

if __name__ == '__main__':
  controller = BaxterController(print_debug=True)
  
  # Test direct movements to positions.
  print()
  print('='*50)
  print('Testing direct movements')
  print('='*50)
  controller.move_to_resting('left')
  controller.move_to_resting('right')
  while controller.is_movement_in_progress('left') or controller.is_movement_in_progress('right'):
    time.sleep(0.1)
  time.sleep(5)
  controller.move_to_joint_angles_rad(controller.get_joint_angles_rad())
  while controller.is_movement_in_progress('left') or controller.is_movement_in_progress('right'):
      time.sleep(0.1)
  time.sleep(5)
  controller.move_to_neutral('left')
  controller.move_to_neutral('right')
  while controller.is_movement_in_progress('left') or controller.is_movement_in_progress('right'):
      time.sleep(0.1)
  time.sleep(5)
  
  # Test a trajectory.
  print()
  print('='*50)
  print('Testing a trajectory from joint angles')
  print('='*50)
  controller.build_trajectory_from_joint_angles('left',
    times_from_start_s=[5, 10, 15],
    joint_angles_rad=[
    list(controller.get_resting_joint_angles_rad()['left'].values()),
    list(controller.get_neutral_joint_angles_rad()['left'].values()),
    list(controller.get_resting_joint_angles_rad()['left'].values()),
  ])
  controller.build_trajectory_from_joint_angles('right',
    times_from_start_s=[5, 10, 15],
    joint_angles_rad=[
    list(controller.get_resting_joint_angles_rad()['right'].values()),
    list(controller.get_neutral_joint_angles_rad()['right'].values()),
    list(controller.get_resting_joint_angles_rad()['right'].values()),
  ])
  success = controller.run_trajectory(['left', 'right'])
  while controller.is_movement_in_progress('left') or controller.is_movement_in_progress('right'):
    time.sleep(0.1)

  # Test solving inverse kinematics.
  print()
  print('='*50)
  print('Testing inverse kinematics')
  print('='*50)
  test_gripper_positions_m = {
    'left':  [0.657579481614,  0.851981417433, 0.0388352386502],
    'right': [0.656982770038, -0.852598021641, 0.0388609422173]
  }
  test_gripper_quaternions_wijk = {
    'left':  [0.262162481772, -0.366894936773, 0.885980397775,  0.108155782462],
    'right': [0.261868353356,  0.367048116303, 0.885911751787, -0.108908281936],
  }
  for limb_name in controller._limb_names:
    joint_angles_rad = controller.get_joint_angles_rad_for_gripper_pose(limb_name,
      gripper_position_m=test_gripper_positions_m[limb_name],
      gripper_orientation_quaternion_wijk=test_gripper_quaternions_wijk[limb_name])
    printVariable(rad2deg(joint_angles_rad), 'joint_angles_deg', level=1)

  # Disable Baxter and quit.
  print()
  print('='*50)
  print('Quitting')
  print('='*50)
  controller.disable_baxter(move_to_resting=True, prompt_before_disabling=True)
  controller.quit()
  
  print()
  print('Done!')
  print()
  

  
  
  
  
  
  
  

