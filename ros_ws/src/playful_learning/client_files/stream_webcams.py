"""Show two USB webcams on Windows/Linux; press Q or Esc to quit.

Install: python -m pip install --upgrade pip
         python -m pip install opencv-python cv2-enumerate-cameras
Run:     python stream_webcams.py

Built-in cameras are excluded by name; adjust INTERNAL_NAMES if needed.
"""

import sys

import cv2
from cv2_enumerate_cameras import enumerate_cameras


FRAME_SIZE = (640, 480)
INTERNAL_NAMES = ("integrated", "internal", "built-in", "built in", "builtin")


def find_usb_cameras():
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
    selected = []
    for info in enumerate_cameras(backend):
        # Laptop cameras can also use USB, so exclude their names explicitly.
        external = (info.vid is not None and
                    not any(word in info.name.lower() for word in INTERNAL_NAMES))
        print("{}: {} ({})".format(
            info.index, info.name, "selected" if external else "skipped"))
        if external:
            selected.append(info)
    if len(selected) != 2:
        raise SystemExit("Expected 2 external USB cameras; found {}. "
                         "Check the names above and INTERNAL_NAMES.".format(len(selected)))
    return sorted(selected, key=lambda info: info.path or info.name)


def main():
    devices = find_usb_cameras()
    cameras = []
    window = "USB webcams - Q or Esc to quit"
    try:
        for info in devices:
            camera = cv2.VideoCapture(info.index, info.backend)
            cameras.append(camera)
            if not camera.isOpened():
                raise SystemExit("Could not open {}.".format(info.name))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])

        while True:
            frames = []
            for number, (info, camera) in enumerate(zip(devices, cameras), 1):
                ok, frame = camera.read()
                if not ok:
                    raise SystemExit("Could not read {}.".format(info.name))
                frame = cv2.resize(frame, FRAME_SIZE)
                cv2.putText(frame, "{}: {}".format(number, info.name), (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                frames.append(frame)

            cv2.imshow(window, cv2.hconcat(frames))
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        for camera in cameras:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
