
from BaxterController import BaxterController

baxter_controller = BaxterController()

should_quit = False
while not should_quit:
  # Process command from remote client.

  #########################################
  # Current state
  #########################################

  # Get joint angles for both limbs.
  baxter_controller.get_joint_angles_rad()

  # Get joint velocities for both limbs.
  baxter_controller.get_joint_velocities_rad_s()

  # Get joint torques for both limbs.
  baxter_controller.get_joint_efforts_Nm()

  # Check whether movement is in progress.
  limb_name = None # get from the client
  baxter_controller.is_movement_in_progress(limb_name=limb_name)

  # Wait until movement is completed.
  limb_name = None # get from the client
  baxter_controller.wait_for_movement_completion(limb_name=limb_name)
  # return whether the movement successfully finished

  #########################################
  # Control for single targets
  #########################################

  # Stop all limb and gripper movement.
  baxter_controller.abort_movement()
  
  # Move to neutral pose.
  limb_name = None # get from the client
  baxter_controller.move_to_neutral(limb_name=limb_name)

  # Move to resting pose.
  limb_name = None # get from the client
  baxter_controller.move_to_resting(limb_name=limb_name)

  # Move to target joint angles.
  joint_angles_rad_byLimb = None # get from the client
  baxter_controller.move_to_joint_angles_rad(joint_angles_rad_byLimb=joint_angles_rad_byLimb)

  # Move to target gripper pose and orientation.
  gripper_position_m_byLimb = None # get from the client
  gripper_orientation_quaternion_wijk_byLimb = None # get from the client
  baxter_controller.move_to_gripper_pose(
    gripper_position_m_byLimb=gripper_position_m_byLimb,
    gripper_orientation_quaternion_wijk_byLimb=gripper_orientation_quaternion_wijk_byLimb)
  # return whether angles were found for the requested poses

  # Inverse kinematics for a pose.
  limb_name = None # get from the client
  gripper_position_m = None # get from the client
  gripper_orientation_quaternion_wijk = None # get from the client
  seed_joint_angles_rad = None # get from the client
  baxter_controller.get_joint_angles_rad_for_gripper_pose(
    limb_name=limb_name,
    gripper_position_m=gripper_position_m,
    gripper_orientation_quaternion_wijk=gripper_orientation_quaternion_wijk,
    seed_joint_angles_rad=seed_joint_angles_rad,
  )

  #########################################
  # Trajectory control
  #########################################

  # Build a trajectory from a sequence of joint angles.
  limb_name = None # get from the client
  times_from_start_s = None # get from the client
  joint_angles_rad = None # get from the client
  baxter_controller.build_trajectory_from_joint_angles(
    limb_name=limb_name, 
    times_from_start_s=times_from_start_s, 
    joint_angles_rad=joint_angles_rad,
    )

  # Build a trajectory from a sequence of gripper positions and orientations.
  limb_name = None # get from the client
  times_from_start_s = None # get from the client
  gripper_positions_m = None # get from the client
  gripper_orientations_quaternion_wijk = None # get from the client
  baxter_controller.build_trajectory_from_gripper_poses(
    limb_name=limb_name, 
    times_from_start_s=times_from_start_s,
    gripper_positions_m=gripper_positions_m,
    gripper_orientations_quaternion_wijk=gripper_orientations_quaternion_wijk,
  )

  # Run the currently built trajectory.
  limb_names = None # get from the client
  baxter_controller.run_trajectory(limb_names=limb_names)

  # Get whether a trajectory succeeded.
  limb_name = None # get from the client
  baxter_controller.trajectory_succeeded(limb_name=limb_name)
  # return result to client

  #########################################
  # Gripper control
  #########################################

  # Controller gripper by position or force.
  limb_name = None # get from the client
  gripper_open_percent = None # get from the client
  force_threshold_percent = None # get from the client
  baxter_controller.move_gripper(
    limb_name=limb_name, 
    gripper_open_percent=gripper_open_percent, 
    force_threshold_percent=force_threshold_percent)

  # Stop the gripper.
  limb_name = None # get from the client
  baxter_controller.stop_gripper(limb_name=limb_name)

  # Get current gripper position.
  limb_name = None # get from the client
  baxter_controller.get_gripper_position_open_percent(limb_name=limb_name)
  # return result to client

  # Get current gripper force.
  limb_name = None # get from the client
  baxter_controller.get_gripper_force_percent(limb_name=limb_name)
  # return result to client

  # Get gripper grasp status.
  limb_name = None # get from the client
  baxter_controller.is_gripper_grasping(limb_name=limb_name)
  # return result to client

  # Get gripper movement status.
  limb_name = None # get from the client
  baxter_controller.is_gripper_moving(limb_name=limb_name)
  # return result to client

