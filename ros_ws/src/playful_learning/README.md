# Baxter remote control

The old desktop runs a Python 2.7 JSON/HTTP server next to BaxterController. The
new computer runs the Python 3.8+ client or browserInterface. The transport and
browserInterface use the standard library, with no package installation or internet connection
required. Only the old desktop needs the existing ROS/Baxter SDK installation.
The optional Python image-file helper uses Pillow on the new computer.

```
Python client ------------------ JSON/HTTP ------ old desktop ----- ROS ----- Baxter
Browser -- localhost HTTP proxy -- JSON/HTTP -----/
```

The browser files are local, including their styling and JavaScript. The proxy
runs on the new computer and forwards requests to the old desktop.

There is no password or token to configure. Anyone who can reach the old
desktop's HTTP port can read state, view cameras, and send robot commands.
Use this service on a trusted private LAN; do not forward its port to the internet.
HTTP traffic is unencrypted.

## Files and compatibility

| File | Purpose | Python |
| --- | --- | --- |
| `BaxterRemoteController_server.py` | HTTP API, validation, operation tracking, control lease | 2.7 or 3.8+ |
| `remote_robot.py` | Adapter for the existing controller and Baxter SDK | 2.7 |
| `remote_simulation.py` | Hardware-free backend for trying the client/interface | 2.7 or 3.8+ |
| `BaxterCameras.py` | Read camera topics and encode PNG frames without image packages | 2.7 or 3.8+ |
| `BaxterHeadController.py` | Head motion, halo/sonar lights, and RGB screen output | 2.7 |
| `BaxterRemoteController_client.py` | Importable client and operation waits | 3.8+ |
| `sample_remote_client.py` | Short programmatic example | 3.8+ |
| `browser_interface/BaxterRemoteController_browserInterface.py`, `browser_interface/browser_interface/` | Local browser server and assets, inside `client_files/` | 3.8+ |

`BaxterController.py`, `BaxterTrajectory.py`, and `helpers.py` are unchanged. The
adapter confines necessary access to the controller's private SDK objects to one
new module, including endpoint reads, cancellation, and trajectory completion
checks. The remote service owns one controller instance; avoid simultaneously
running another program that commands the same robot.

## 1. Copy the files

The project is arranged as follows. Copy the contents of `server_files/` to the
old desktop; keep `client_files/` on the new computer. The tests are optional for
normal client use.

```text
playful_learning/
  README.md
  server_files/
    BaxterRemoteController_server.py
    BaxterController.py
    BaxterTrajectory.py
    helpers.py
    remote_robot.py
    remote_simulation.py
    BaxterCameras.py
    BaxterHeadController.py
  client_files/
    BaxterRemoteController_client.py
    sample_remote_client.py
    browser_interface/
      BaxterRemoteController_browserInterface.py
      browser_interface/
        index.html
        app.js
        style.css
    tests/
```

From PowerShell in `playful_learning/` (beside this README), with the destination
folder already present:

```powershell
scp .\server_files\*.py USER@OLD_DESKTOP:~/ros_ws/src/playful_learning/
```

Replace `USER` and `OLD_DESKTOP` with the old desktop's account and router-facing
IP address. USB transfer works equally well if SSH is unavailable. This copies
all eight server modules, including the three original controller modules, into
one folder on the old desktop. If you instead copy the `server_files/` folder
itself, change into that folder before starting the server.

On the new computer, preserve the `client_files/` layout above so the browser
launcher can find the client and its assets. For simulation or regression tests,
also keep `server_files/` beside `client_files/`. The original robot modules are
unnecessary when connecting the client to the real server.

Use the controller modules from this same project revision on the old desktop;
the adapter reads a few SDK/controller internals. No edits to those modules are
needed for this remote interface.

These scripts run directly with Python. No new catkin package, `catkin_make`,
`rosrun` registration, or ROS installation on the new computer is required. The
old desktop's existing Baxter packages still need their usual working workspace.

## 2. Start the old desktop's ROS environment and server

Find the old desktop's address on the shared router with `ip addr`. For example,
it might be `192.168.1.50`. This can differ from the interface used to communicate
with Baxter. Keep the existing working robot hostname and `ROS_IP` settings in
`baxter.sh`; bind the HTTP server to the **router-facing** address.

In the first old-desktop terminal:

```bash
cd ~/ros_ws
./baxter.sh
rosrun baxter_interface joint_trajectory_action_server.py --limb both --mode position
```

`baxter.sh` opens a configured shell; enter the `rosrun` command in that shell.
If the trajectory servers are already running, keep those existing servers. Both
arms' servers are required by the current controller's initialization. Physical
Baxter already has its ROS master, so this setup does not need another `roscore`.

In a second old-desktop terminal:

```bash
cd ~/ros_ws
./baxter.sh
rostopic echo -n 1 /robot/state
cd ~/ros_ws/src/playful_learning
python2.7 -B BaxterRemoteController_server.py --host 192.168.1.50 --port 8765
```

Replace `192.168.1.50` with the old desktop's actual router-facing IP. The default
host is `127.0.0.1`, which only permits connections from the same computer.
**Starting the real server constructs BaxterController, enables Baxter, and
calibrates its grippers**, following the existing controller's initialization.
Have the robot workspace clear before starting it.

If Ubuntu's firewall is enabled, permit TCP 8765 from the new computer's address.
For example, after substituting both addresses:

```bash
sudo ufw status
sudo ufw allow from 192.168.1.60 to 192.168.1.50 port 8765 proto tcp
```

No internet access or package download is involved. Router guest networks or WiFi
client isolation can prevent two computers on the router from communicating.

## 3. Run the Python client

In a terminal on the new computer, change to `playful_learning/client_files/`.

First edit `SERVER_URL` in `sample_remote_client.py` to the old desktop's address
and port, for example `"http://192.168.0.180:8765"`. Then run:

```powershell
py sample_remote_client.py
```

The sample is a linear sequence of API calls with no command-line arguments.
**Running it moves the robot:** joint, endpoint, gripper, trajectory, and head-pan
examples start from current feedback and request small changes. It also reads
the cameras, nods the head, sets lights, and updates the screen. Check the robot
workspace and read the sample before running it.

Neutral/resting poses and fully opening/closing the gripper are commented
examples because they can make large movements. Unsupported head tilt and the
optional Pillow image-file helper are also shown as comments. Active calls wait
for completion; a failed command raises an error and ends the sample.

On Linux/macOS use `python3` in place of `py`. The client requires Python 3.8 or
newer; the browser launcher provides `--help`.

Using the object from your own program:

```python
from BaxterRemoteController_client import BaxterRemoteController

with BaxterRemoteController("http://192.168.1.50:8765") as robot:
    angles = robot.get_joint_angles_rad()
    operation = robot.move_to_joint_angles_rad({"left": angles["left"]})
    reached_targets = operation.wait()
    print(robot.get_end_effector_poses())
```

## API behavior

Method names and argument names follow BaxterController for the exposed methods.
Required arguments work positionally or by name; optional controller arguments
such as `timeout_s`, `tolerance_rad`, seeds, and `goal_time_tolerance_s` are passed
by name. The server rejects unknown methods and unsupported arguments.

| Calls | Return value |
| --- | --- |
| Joint, endpoint, gripper, and movement-status reads | State immediately |
| `get_state()` | One combined snapshot with server timestamp and simulation flag |
| `get_camera_frame`, `get_wrist_camera_frame` | Wrist PNG image bytes; raises `RemoteError` if unavailable |
| `get_head_state`, `get_head_pan_rad` | Head feedback immediately |
| `get_head_tilt_rad()` / `set_head_tilt_rad(...)` | `None` / unsupported-operation error |
| Head pan/nod, lights, screen color/image commands | `Operation` |
| `move_to_joint_angles_rad`, `move_to_neutral`, `move_to_resting`, `move_to_gripper_pose` | `Operation` |
| `move_gripper`, `open_gripper`, `close_gripper`, the three jog methods | `Operation` |
| `run_trajectory(limb_names=None)` | `Operation` |
| `get_joint_angles_rad_for_gripper_pose(...)` | Joint solution or `None`; waits internally |
| The two `build_trajectory_...(...)` methods | Build result; waits internally |
| `trajectory_succeeded(limb_name)` | `True`, `False`, or `None` when no completed result exists |
| `abort_movement(limb_name=None)`, `stop_gripper(limb_name)` | Cancellation acknowledgement |
| `wait_for_movement_completion(limb_name=None, timeout_s=60)` | `True` for idle, `False` on timeout |

`operation.status()` returns its state and any result/error. `operation.wait()`
returns the successful result, raises `RemoteError` for failure/cancellation, and
raises `TimeoutError` if its deadline expires. A network or wait timeout **does
not cancel motion**. Commands are never automatically retried: a lost response
can mean a command was accepted. Use the operation ID/status when available, or
read state and explicitly request cancellation before deciding what to do next.

Neutral/resting, joint-angle, end-effector, and joint/endpoint jog moves share
the same completion check. They allow **30 seconds** by default and require each
requested joint to be within **0.008726646 radians (0.5 degrees)** of its target.
After the motion worker ends, the server allows up to 0.5 seconds for feedback
to settle, without sending another motion command. A target miss reports the
elapsed time, configured timeout, worst joint error, and tolerance. Looking close
to the requested pose does not necessarily mean every joint reached that tolerance.

For a slower or longer move, pass an explicit motion timeout. The client wait
timeout is separate: both `operation.wait()` and `wait_for_movement_completion()`
default to 60 seconds. Give the client enough time to receive the motion result:

```python
motion = robot.move_to_resting("left", timeout_s=60, tolerance_rad=0.008726646)
motion.wait(timeout_s=65)
```

The client's `request_timeout_s` (default 3 seconds) applies to each HTTP request,
not the whole movement. Motion requests return an operation ID promptly; waiting
polls that ID. Trajectories use their waypoint timing; head and gripper commands
retain their separate timeouts.

An idle arm alone does not establish that its commanded target was reached;
prefer the specific operation's result. Stop acknowledgements mean a stop was
requested, not that physical stopping has been confirmed. A software/network
stop is not Baxter's physical emergency stop.

Only one client owns control at a time, acquired by its first operation. There
is one active operation at a time, including IK and trajectory preparation;
conflicting commands receive a busy error. A single operation may target both
arms. State reads remain available while motion is running. Cancelling one arm
of an active two-arm operation cancels that entire operation.
Any connected client can request a stop, even while another client owns
control; only the owner can release its control lease.

The client sends a heartbeat every second. If the owner stops communicating for
five seconds, the server requests cancellation and releases ownership. The next
command must acquire ownership again. `close()` and exiting the context manager
also request cancellation/release; wait for desired operations before exiting.
Long GUI work should run outside the GUI thread so the interface remains responsive.

Trajectory builders replace the currently prepared trajectory for their arm.
They do not move the robot. Call `run_trajectory(["left"])` to execute just that
arm. Prepared trajectories are cleared when control changes owners. Trajectory
timing executes on the old desktop and does not depend on network polling.
The adapter starts from the measured joint position at execution; uploaded
timestamps are measured from that start. It does not first move to the first
uploaded waypoint, so allow enough time to reach that waypoint.

Joint dictionaries use full names (`left_s0`, `right_w2`, etc.). Seven-element
angle arrays follow `s0, s1, e0, e1, w0, w1, w2`. Angles are radians, torques are
Nm, endpoint positions are meters in Baxter's `base` frame, and quaternions are
`[w, x, y, z]`. `get_end_effector_poses()` returns, for each arm,
`{"position_m": [x, y, z], "orientation_wijk": [w, x, y, z]}`.
Gripper position and force use the SDK's 0–100 percentages.
The remote API limits requested gripper force to 30% (default 15%).

Additional jog calls are:

```python
robot.jog_joint("left", "left_w2", 0.01).wait()
robot.jog_endpoint("left", "x", 0.002).wait()      # 2 mm along base X
robot.jog_endpoint("left", "yaw", 0.01).wait()    # rotation about base Z
robot.jog_gripper("left", 1.0).wait()             # 1 percentage point
```

For endpoint jogging, `x/y/z` translate in meters; `roll/pitch/yaw` rotate in
radians about the base frame's X/Y/Z axes. The server reads the current state
when executing each jog and uses IK for endpoint targets. IK can fail even for
a small step. Joint-space movement to an IK solution does not guarantee a
straight Cartesian path or collision-free motion.
Each jog is limited to 5 degrees for joint/rotation steps, 2 cm for Cartesian
translation, or 10 percentage points for gripper opening. These bounds limit
individual jogs; ordinary target/trajectory calls can request larger movements.

## Camera setup and API

Baxter supports **two active cameras simultaneously**. This interface exposes
only the two wrist cameras; close the head camera before opening both wrists.
640x400 is a supported resolution suitable for these snapshots. [Rethink camera documentation](https://github.com/RethinkRobotics/sdk-docs/wiki/Using-the-Cameras).

On the old desktop, in a configured `baxter.sh` shell, enable both wrists:

```bash
rosrun baxter_tools camera_control.py -l
rosrun baxter_tools camera_control.py -c head_camera
rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 640x400
rosrun baxter_tools camera_control.py -o right_hand_camera -r 640x400
```

These flags match the bundled SDK script and the [Rethink camera-control example](https://github.com/RethinkRobotics/sdk-docs/wiki/Camera-Control-Example).
The getters and browser do not open, close, or reconfigure cameras. An inactive
camera remains unavailable until opened from the old desktop.

The client exposes the same camera getters as the separate `BaxterCameras` class:

```python
with BaxterRemoteController("http://192.168.1.50:8765") as robot:
    left_png = robot.get_wrist_camera_frame("left")
    # Equivalent general form: robot.get_camera_frame("left_hand_camera")
    with open("left_wrist.png", "wb") as image_file:
        image_file.write(left_png)
```

For a standalone script on the old desktop, initialize a ROS node once and use
the camera class directly (no remote server or BaxterController is needed).
Skip `rospy.init_node` if an existing BaxterController already initialized it:

```python
import rospy
from BaxterCameras import BaxterCameras

rospy.init_node("camera_reader")
cameras = BaxterCameras()
left_png = cameras.get_wrist_camera_frame("left")
```

`get_camera_frame(camera_name)` accepts `left_hand_camera` or
`right_hand_camera`. Head-camera requests are rejected. Every getter returns PNG **bytes**, ready to save or display;
it does not return a pixel array. A modern client application can optionally use
its preferred image library to decode those bytes. No image library is required
to save files or use the browserInterface.

Frames use HTTP `GET /camera/<camera_name>.png` separately from
JSON-RPC. The old desktop waits up to one second for a ROS image on the camera's
`/cameras/<camera_name>/image` topic and encodes it with the standard library.
No `cv_bridge`, OpenCV, Pillow, new catkin dependency, or package installation is
needed. An unavailable frame raises `RemoteError`; camera requests run separately
from state and motion handling, so camera errors do not disable robot control.
Supported ROS encodings are the 8-bit formats `mono8`, `rgb8`, `bgr8`, `rgba8`, and
`bgra8`; other encodings produce a clear error. PNG encoding preserves alpha when
present and handles row padding and BGR channel ordering.

## Head, lights, and screen API

`get_head_state()` returns `pan_rad`, `tilt_rad`, `tilt_supported`, `panning`,
`nodding`, `feedback_age_s`, and `feedback_stale`. The same dictionary appears as
`get_state()["head"]`. Pan is measured in radians. Baxter's SDK provides a pan
angle and a discrete nod gesture; it does not provide a continuously commanded
tilt angle. Accordingly, `get_head_tilt_rad()` returns `None`,
`tilt_supported` is `False`, and `set_head_tilt_rad(...)` raises `RemoteError`.
Use `nod_head()` for up/down nodding. [Rethink head-joint API](https://github.com/RethinkRobotics/sdk-docs/wiki/API-Reference#head-joints).

```python
with BaxterRemoteController("http://192.168.1.50:8765") as robot:
    print(robot.get_head_pan_rad())
    robot.set_head_pan_rad(0.0, speed_percent=25, timeout_s=10).wait()
    robot.nod_head(times=1, internod_delay_s=0).wait()
    robot.set_halo_led(red_percent=0, green_percent=25).wait()
    robot.set_sonar_leds("auto").wait()
    robot.show_screen_color([24, 32, 48]).wait()
```

All supported setters return an operation and use the same ownership/busy rules
as arm movement. `abort_movement()` requests cancellation of head pan/nod as well
as the arms and grippers. An SDK nod that has already begun may finish despite
cancellation; software/network cancellation has no guaranteed physical stop time.
LED and display operation success confirms publication of the command, not
verified feedback from the lights or screen.

Halo intensities are percentages. Sonar LEDs accept `"auto"`, `"on"`, `"off"`,
a list of twelve 0/1 states, or a list of LED indices from 0 through 11.
For example, `robot.set_sonar_leds([0, 3, 6, 9]).wait()` lights selected LEDs.
Screen colors are `[red, green, blue]` integers from 0 to 255.

For application-generated images, pass packed RGB bytes without extra packages:

```python
rgb_data = bytes([20, 80, 120]) * (640 * 400)
robot.show_screen_image_rgb(640, 400, rgb_data).wait()
```

The RGB buffer must contain exactly `width * height * 3` bytes. Dimensions must
be positive integers no larger than 1024 by 600. The client base64-encodes the
buffer for JSON transport; the old desktop publishes it as an RGB ROS image.
The request limit is 3 MiB, enough for a full 1024-by-600 RGB screen image.

For JPEG, PNG, or other local image files, optionally install Pillow **on the new
computer only**:

```powershell
python -m pip install Pillow
```

Then call `robot.show_screen_image("picture.png").wait()`. This helper applies
EXIF orientation, preserves aspect ratio, composites transparency over black,
and centers the image on a black 1024-by-600 canvas before sending RGB bytes.
The resizing and orientation use [Pillow's ImageOps](https://pillow.readthedocs.io/en/stable/reference/ImageOps.html).
The browser handles file decoding and resizing itself with canvas, so browser
uploads need no Pillow installation. The old desktop uses only its existing ROS
packages and Python's standard library for all head and display functions.

## 4. Run the browserInterface on the new computer

In a terminal in `playful_learning/client_files/` on the new computer:

```powershell
python browser_interface/BaxterRemoteController_browserInterface.py --server http://192.168.1.50:8765 --port 8000
```

On Windows, use `py` instead of `python` if that is your Python launcher.
The two ports serve different purposes: `:8765` in `--server` is the old
desktop's HTTP port; `--port 8000` is the browserInterface's local port on the new
computer. Include the server port explicitly: `http://192.168.1.50` alone uses
port 80. Substitute your old desktop's actual address, for example:

```powershell
py browser_interface/BaxterRemoteController_browserInterface.py --server http://192.168.0.180:8765 --port 8000
```

Open **http://127.0.0.1:8000** in a browser on that computer. The browser server binds
only to localhost. The page displays each arm's joint angles and
torques, end-effector position/orientation, and gripper position/force/status.
It provides individual jog buttons for joints, Cartesian translation, base-axis
rotation, and grippers, plus Neutral and Resting pose buttons for each arm and a
stop control. Enable the Control buttons toggle to use them; each pose button
moves only its arm and waits for the current command to finish before another
can be sent. Trajectory building remains in the Python API.
Each gripper also has **Open (100%)** and **Close (0%)** buttons alongside its
jog controls. These use the same control toggle and wait for each operation to finish.

After an ordinary command failure, the error stays visible and controls become
available again once a subsequent state update confirms fresh, idle, fault-free
feedback. A disconnect, stale feedback, controller fault, or unknown command
status still turns the control toggle off. Failed commands are never retried
automatically.

Two camera cards show both wrists, refreshing available frames
about once per second. A closed camera is marked unavailable; the other images
and motion controls continue working. The head panel shows pan and nod status,
with controls for pan, nodding, halo/sonar LEDs, screen color, and image upload.
Continuous tilt is shown as unsupported.

The page's state polling renews its control lease; stale/disconnected state
disables movement controls. Closing the page stops that polling, so its lease
expires and the server requests cancellation. Keep the page in the foreground during control because browsers can
throttle background tabs. Stop the programmatic client before controlling from
the browser, or its control lease will cause busy/ownership errors.

## Try everything without Baxter

On the new computer, keep `server_files/` and `client_files/` beside each other,
as in the layout above. In the first terminal, from `playful_learning/`:

```powershell
python server_files/BaxterRemoteController_server.py --simulate --host 127.0.0.1 --port 8765
```

In another terminal, from `playful_learning/client_files/`:

```powershell
python browser_interface/BaxterRemoteController_browserInterface.py --server http://127.0.0.1:8765 --port 8000
```

To run the Python sample against this simulator, set its `SERVER_URL` to
`"http://127.0.0.1:8765"` and run `python sample_remote_client.py`. Close the sample
before issuing commands in the browser so the two clients do not compete for control.

Open http://127.0.0.1:8000. Simulation requires neither ROS nor Baxter modules.
Its pose/IK behavior is illustrative and does not model Baxter's physics,
workspace, or collisions. Trajectories use accelerated timing of about 0.4 seconds
per waypoint, rather than their uploaded timestamps. Successful simulation checks communication and interface
behavior; real Python 2.7/ROS/hardware operation must still be verified on Baxter.
Simulation also provides generated images for both wrists and simulated head
feedback. These are illustrative; they do not establish hardware motion timing.

## Troubleshooting and shutdown

The real server tracks incoming ROS joint, endpoint, and gripper feedback. If
feedback is older than two seconds, new operations are rejected and active
operations receive cancellation. The browser shows stale feedback separately
from a lost HTTP connection. A cancellation that cannot be acknowledged latches
a controller fault; inspect the robot and restart the server before continuing.

To run the included hardware-free regression checks on the new computer,
from `playful_learning/` (the parent of `client_files/`):

```bash
python -B client_files/tests/run_tests.py
```

- **Connection refused or timed out:** confirm the server is running, use its
  router-facing IP and port (including `:8765` in `--server`), check the firewall,
  and check router client isolation. On Windows, test reachability with
  `Test-NetConnection 192.168.0.180 -Port 8765` in PowerShell, substituting the
  actual address.
- **WinError 10053 when the browserInterface writes a response:** the local browser connection
  was aborted before the response could be delivered. This does not identify
  why the Baxter connection failed. The browserInterface handles this disconnect
  quietly; restart it after copying the updated client files. If the page reports
  a connection error, verify the server URL and port as above.
- **Busy or another owner:** finish/cancel the active operation and close the
  other client, or wait for its five-second lease to expire after it disconnects.
- **ROS initialization waits:** verify `rostopic echo -n 1 /robot/state` works and
  both trajectory action servers are running in a configured `baxter.sh` shell.
- **IK or target failure:** inspect the returned error and current pose before
  choosing another target. Do not automatically replay failed movement commands.

Stop/release control from the client, then press Ctrl+C in the browser/server
terminals. The remote service requests cancellation on shutdown. It does not
provide remote enabling/disabling methods. For the existing explicit disable
command, use the old desktop's configured ROS terminal:

```bash
rosrun baxter_tools enable_robot.py -d
```
