"""On-demand camera snapshots using ROS and Python 2.7 standard libraries.

Initialize rospy first (BaxterController already does this). Open the desired
wrist cameras with baxter_tools camera_control.py; this class never opens or
closes them. The head camera is disabled in this project. Getters return PNG bytes.
"""
import struct
import zlib


CAMERA_NAMES = ('left_hand_camera', 'right_hand_camera')


def image_to_png(message):
    """Encode common 8-bit ROS Image formats without cv_bridge or Pillow."""
    formats = {'mono8': (1, 0), 'rgb8': (3, 2), 'bgr8': (3, 2),
               'rgba8': (4, 6), 'bgra8': (4, 6)}
    if message.encoding not in formats:
        raise ValueError('Unsupported camera encoding: ' + message.encoding)
    channels, color_type = formats[message.encoding]
    width, height, step = message.width, message.height, message.step
    row_size = width * channels
    if (not 0 < width <= 4096 or not 0 < height <= 4096 or step < row_size
            or step * height > 32 * 1024 * 1024 or len(message.data) != step * height):
        raise ValueError('Invalid camera image dimensions, row stride, or data length')

    # Strip row padding before swapping BGR to RGB. Alpha, if present, is kept.
    raw = bytes(bytearray(message.data))
    pixels = bytearray(b''.join(raw[y * step:y * step + row_size] for y in range(height)))
    if message.encoding in ('bgr8', 'bgra8'):
        pixels[0::channels], pixels[2::channels] = pixels[2::channels], pixels[0::channels]
    pixels = bytes(pixels)
    rows = b''.join(b'\x00' + pixels[y * row_size:(y + 1) * row_size] for y in range(height))
    header = struct.pack('!IIBBBBB', width, height, 8, color_type, 0, 0, 0)
    png = [b'\x89PNG\r\n\x1a\n']
    for kind, data in ((b'IHDR', header), (b'IDAT', zlib.compress(rows, 1)), (b'IEND', b'')):
        png.append(struct.pack('!I', len(data)) + kind + data +
                   struct.pack('!I', zlib.crc32(kind + data) & 0xffffffff))
    return b''.join(png)


class BaxterCameras(object):
    def get_camera_frame(self, camera_name):
        """Wait up to one second for a frame; raise RuntimeError if unavailable.

        A temporary subscription is removed after every request, so cameras do
        not send raw frames to this process while nobody is requesting images.
        """
        if camera_name not in CAMERA_NAMES:
            raise ValueError('Only left_hand_camera and right_hand_camera are enabled')
        import rospy
        from sensor_msgs.msg import Image
        try:
            message = rospy.wait_for_message('/cameras/' + camera_name + '/image', Image, timeout=1.0)
        except rospy.ROSException as exc:
            raise RuntimeError('%s unavailable: check that the camera is open and ROS is connected (%s)' %
                               (camera_name, exc))
        return image_to_png(message)

    def get_wrist_camera_frame(self, limb_name):
        if limb_name not in ('left', 'right'):
            raise ValueError('limb_name must be left or right')
        return self.get_camera_frame(limb_name + '_hand_camera')
