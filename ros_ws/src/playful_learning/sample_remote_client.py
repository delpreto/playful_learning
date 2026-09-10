"""Read state; optional wrist snapshots, arm movement, head, and screen examples."""

import argparse
from pprint import pprint

from BaxterRemoteController_client import BaxterRemoteController, RemoteError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://127.0.0.1:8765")
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--motion", action="store_true", help="Also command real movement")
    parser.add_argument("--images", action="store_true", help="Save available cameras as PNG files")
    parser.add_argument("--head", action="store_true", help="Run head, light, and screen examples")
    parser.add_argument("--screen-image", metavar="FILE", help="Display a local image (requires Pillow)")
    args = parser.parse_args()
    limb = args.limb

    with BaxterRemoteController(args.server) as robot:
        pprint({"angles_rad": robot.get_joint_angles_rad(),
                "velocities_rad_s": robot.get_joint_velocities_rad_s(),
                "torques_Nm": robot.get_joint_efforts_Nm(),
                "end_effector_poses": robot.get_end_effector_poses(),
                "head": robot.get_head_state(),
                "movement_in_progress": robot.is_movement_in_progress()})
        for name in ("left", "right"):
            pprint({"limb": name,
                    "gripper_open_percent": robot.get_gripper_position_open_percent(name),
                    "gripper_force_percent": robot.get_gripper_force_percent(name),
                    "grasping": robot.is_gripper_grasping(name),
                    "gripper_moving": robot.is_gripper_moving(name)})
        if args.images:
            for camera in ("left_hand_camera", "right_hand_camera"):
                try:
                    frame = robot.get_camera_frame(camera)
                except RemoteError as error:
                    print(camera + ": " + str(error))
                    continue
                with open(camera + ".png", "wb") as image_file:
                    image_file.write(frame)
                print("Saved " + camera + ".png")
        if args.head:
            robot.set_head_pan_rad(robot.get_head_pan_rad()).wait()
            robot.nod_head().wait()
            robot.set_halo_led(red_percent=0, green_percent=25).wait()
            robot.set_sonar_leds("auto").wait()
            robot.show_screen_color([24, 32, 48]).wait()
            print("Head examples finished. Tilt angle:", robot.get_head_tilt_rad())
        if args.screen_image:
            robot.show_screen_image(args.screen_image).wait()
        if not args.motion:
            print("Sample finished. Use --motion for the arm/gripper examples.")
            return

        # Start from the measured pose. Every motion is explicitly awaited.
        joints = robot.get_joint_angles_rad()[limb]
        pose = robot.get_end_effector_poses()[limb]
        position, orientation = pose["position_m"], pose["orientation_wijk"]
        solution = robot.get_joint_angles_rad_for_gripper_pose(
            limb, position, orientation, seed_joint_angles_rad=joints)
        print("IK solution:", solution)
        if solution is None:
            raise RuntimeError("No IK solution for the measured pose")

        robot.move_to_joint_angles_rad({limb: joints}).wait(timeout_s=30)
        robot.move_to_gripper_pose(
            {limb: position}, {limb: orientation},
            seed_joint_angles_rad_byLimb={limb: joints}).wait(timeout_s=30)

        # Jog one joint by 0.01 rad, then return to the measured joint targets.
        robot.jog_joint(limb, limb + "_w2", 0.01).wait(timeout_s=30)
        robot.move_to_joint_angles_rad({limb: joints}).wait(timeout_s=30)

        # Both trajectory examples hold the measured joint/endpoint targets.
        joints = robot.get_joint_angles_rad()[limb]
        robot.build_trajectory_from_joint_angles(limb, [1.0, 2.0], [joints, joints])
        robot.run_trajectory([limb]).wait(timeout_s=30)
        print("Joint trajectory succeeded:", robot.trajectory_succeeded(limb))

        pose = robot.get_end_effector_poses()[limb]
        built = robot.build_trajectory_from_gripper_poses(
            limb, [1.0, 2.0], [pose["position_m"]] * 2,
            [pose["orientation_wijk"]] * 2, initial_seed_joint_angles_rad=joints)
        if not built:
            raise RuntimeError("No IK solution for the pose trajectory")
        robot.run_trajectory([limb]).wait(timeout_s=30)
        print("Pose trajectory succeeded:", robot.trajectory_succeeded(limb))

        opening = robot.get_gripper_position_open_percent(limb)
        robot.move_gripper(limb, opening, force_threshold_percent=15).wait(timeout_s=30)
        robot.stop_gripper(limb)
        print("Idle:", robot.wait_for_movement_completion(timeout_s=30))
        robot.abort_movement()

        # Other examples to enable individually after checking the workspace:
        # robot.move_to_neutral(limb).wait(timeout_s=30)
        # robot.move_to_resting(limb).wait(timeout_s=30)
        # robot.jog_endpoint(limb, "x", 0.002).wait(timeout_s=30)
        # robot.jog_endpoint(limb, "yaw", 0.01).wait(timeout_s=30)
        # robot.jog_gripper(limb, 1.0).wait(timeout_s=30)
        # robot.open_gripper(limb).wait(timeout_s=30)
        # robot.close_gripper(limb).wait(timeout_s=30)


if __name__ == "__main__":
    main()
