"""Linear API examples: small left-arm/gripper/head moves, cameras, and screen."""

from BaxterRemoteController_client import BaxterRemoteController


SERVER_URL = "http://192.168.0.180:8765"

with BaxterRemoteController(SERVER_URL) as robot:
    # Synchronous feedback. Change "left" to "right" to use the other arm.
    print("Complete state:", robot.get_state())
    print("Joint angles (rad):", robot.get_joint_angles_rad())
    print("Joint velocities (rad/s):", robot.get_joint_velocities_rad_s())
    print("Joint torques (Nm):", robot.get_joint_efforts_Nm())
    print("End-effector poses:", robot.get_end_effector_poses())
    print("Gripper opening (%):", robot.get_gripper_position_open_percent("left"))
    print("Gripper force (%):", robot.get_gripper_force_percent("left"))
    print("Grasping:", robot.is_gripper_grasping("left"))
    print("Gripper moving:", robot.is_gripper_moving("left"))
    print("Movement in progress:", robot.is_movement_in_progress())
    print("Head state:", robot.get_head_state())
    print("Head pan (rad):", robot.get_head_pan_rad())
    print("Head tilt:", robot.get_head_tilt_rad())  # None: no continuous tilt feedback.

    # Either camera method returns PNG bytes; no image package is required.
    left_png = robot.get_camera_frame("left_hand_camera")
    right_png = robot.get_wrist_camera_frame("right")
    print("Wrist PNG sizes (bytes):", len(left_png), len(right_png))

    # Motion calls return an Operation. Inspect it or wait for completion.
    joints = robot.get_joint_angles_rad()["left"]
    joints["left_w2"] += 0.01
    motion = robot.move_to_joint_angles_rad({"left": joints}, timeout_s=30)
    print("Operation:", motion.status())
    print("Result:", motion.wait(timeout_s=40))

    # Move the measured end-effector pose by 2 mm, preserving orientation.
    joints = robot.get_joint_angles_rad()["left"]
    pose = robot.get_end_effector_poses()["left"]
    position = pose["position_m"][:]
    position[0] += 0.002
    orientation = pose["orientation_wijk"]
    solution = robot.get_joint_angles_rad_for_gripper_pose(
        "left", position, orientation, seed_joint_angles_rad=joints)
    print("IK solution:", solution)
    robot.move_to_gripper_pose(
        {"left": position}, {"left": orientation},
        seed_joint_angles_rad_byLimb={"left": joints}, timeout_s=30).wait()

    # Jog helpers fetch the current position on the server before adding a delta.
    print("Before joint jog:", robot.get_joint_angles_rad()["left"])
    robot.jog_joint("left", "left_w2", -0.01).wait()
    print("Before position jog:", robot.get_end_effector_poses()["left"])
    robot.jog_endpoint("left", "x", -0.002).wait()
    print("Before orientation jog:", robot.get_end_effector_poses()["left"])
    robot.jog_endpoint("left", "yaw", 0.01).wait()

    # Calibration moves the fingers through their range; leave this gripper empty.
    robot.calibrate_gripper("right").wait()
    opening = robot.get_gripper_position_open_percent("left")
    robot.move_gripper("left", max(0, opening - 1), force_threshold_percent=15).wait()
    print("Before gripper jog:", robot.get_gripper_position_open_percent("left"))
    robot.jog_gripper("left", 1.0).wait()
    robot.stop_gripper("left")

    pan = robot.get_head_pan_rad()
    robot.set_head_pan_rad(pan + 0.02, tolerance_rad=0.01).wait()
    print("Before nod:", robot.get_head_state())
    robot.nod_head(times=1).wait()  # Baxter's fixed nod gesture.
    # robot.set_head_tilt_rad(0.1)  # Raises RemoteError: continuous tilt unsupported.

    robot.set_halo_led(red_percent=0, green_percent=25).wait()
    robot.set_sonar_leds("auto").wait()
    robot.show_screen_color([24, 32, 48]).wait()
    # A 2 x 2 RGB image, packed as three bytes per pixel.
    pixels = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255])
    robot.show_screen_image_rgb(2, 2, pixels).wait()
    # robot.show_screen_image("picture.png").wait()  # Local file; requires Pillow.

    # These presets make larger moves, so enable them individually when desired.
    # robot.move_to_neutral("left", timeout_s=30).wait(timeout_s=40)
    # robot.move_to_resting("left", timeout_s=30).wait(timeout_s=40)
    # robot.open_gripper("left").wait()
    # robot.close_gripper("left").wait()

    print("Idle:", robot.wait_for_movement_completion(timeout_s=40))
    robot.abort_movement()  # Request cancellation of any remaining motion.
    print("Raw RPC example:", robot.call("get_state"))

# Leaving the context calls robot.close(), releasing control even after an error.
