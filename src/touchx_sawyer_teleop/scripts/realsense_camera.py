"""Intel RealSense (color + aligned depth) via the realsense2_camera ROS node.

The camera itself is driven by `realsense2_camera` (apt: ros-noetic-realsense2-camera,
started from touchx_teleop.launch with align_depth:=true). This class only
subscribes to its topics and caches the latest frame of each stream.

Mirrors the BaslerCameraManager API (start_bg / get_color / get_depth / stop)
so the recorder treats every camera the same way. Depth is the
`aligned_depth_to_color` stream, so colour pixel (u,v) and depth pixel (u,v)
correspond.

Color: (HEIGHT, WIDTH, 3) uint8 BGR
Depth: (HEIGHT, WIDTH)    uint16 millimetres
"""

import threading

import numpy as np


class RealSenseCamera:
    def __init__(self,
                 color_topic="/camera/color/image_raw",
                 depth_topic="/camera/aligned_depth_to_color/image_raw"):
        import rospy                       # lazy import
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        self._rospy = rospy
        self._Image = Image
        self._bridge = CvBridge()
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self._subs = []
        self._color = None
        self._depth = None
        self._lock = threading.Lock()

    def start_bg(self, wait_timeout=10.0):
        rospy = self._rospy
        self._subs = [
            rospy.Subscriber(self.color_topic, self._Image, self._on_color,
                             queue_size=1, buff_size=2 ** 24),
            rospy.Subscriber(self.depth_topic, self._Image, self._on_depth,
                             queue_size=1, buff_size=2 ** 24),
        ]
        # Wait for the first color frame so a missing camera is reported early.
        try:
            rospy.wait_for_message(self.color_topic, self._Image, timeout=wait_timeout)
            print("[realsense] receiving %s + %s" % (self.color_topic, self.depth_topic))
        except rospy.ROSException:
            print("[realsense] no frames on %s after %.0fs — is realsense2_camera running?"
                  % (self.color_topic, wait_timeout))

    def _on_color(self, msg):
        try:
            color = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self._color = color

    def _on_depth(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return
        with self._lock:
            self._depth = np.asarray(depth, dtype=np.uint16)

    def get_color(self):
        with self._lock:
            return self._color.copy() if self._color is not None else None

    def get_depth(self):
        with self._lock:
            return self._depth.copy() if self._depth is not None else None

    def has_data(self):
        with self._lock:
            return self._color is not None

    def stop(self):
        for s in self._subs:
            try:
                s.unregister()
            except Exception:
                pass
        self._subs = []
