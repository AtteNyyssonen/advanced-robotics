#!/usr/bin/env python3
import rclpy, cv2, numpy as np, time
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge

def rvec_to_rpy(rvec):
    """Convert a 3x1 rotation vector (rvec) to Roll-Pitch-Yaw (RPY) angles."""
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    return np.array([roll, pitch, yaw])

class ArucoView(Node):
    def __init__(self):
        super().__init__('aruco_view')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dictionary', 'DICT_6X6_250')
        self.declare_parameter('marker_id', -1)
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('pose_topic', '/command')

        it = self.get_parameter('image_topic').value
        ct = self.get_parameter('camera_info_topic').value
        dict_name = self.get_parameter('dictionary').value
        self.want_id = int(self.get_parameter('marker_id').value)
        self.mlen = float(self.get_parameter('marker_length_m').value)
        pt = self.get_parameter('pose_topic').value

        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_6X6_250)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.detector_params = cv2.aruco.DetectorParameters_create()
        self.use_new = hasattr(cv2.aruco, 'ArucoDetector')
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params) if self.use_new else None

        self.K = None; self.D = None
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, ct, self.on_info, 1)
        self.create_subscription(Image, it, self.on_img, 10)
        self.pose_pub = self.create_publisher(Float64MultiArray, pt, 1)
        self.state_pub = self.create_publisher(Bool, '/aruco_active', 1)

        self.marker_visible = False
        self.last_seen_time = 0.0
        self.visibility_timeout = 0.5
        self.create_timer(0.1, self.visibility_check)

        cv2.namedWindow('aruco', cv2.WINDOW_NORMAL)
        self.get_logger().info('aruco_view running – publishing only when marker is detected')

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if msg.d else np.zeros((5, 1))

    def on_img(self, msg: Image):
        if self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self.use_new:
            corners, ids, _ = self.detector.detectMarkers(img)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.detector_params)

        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            idx = int(np.where(ids == self.want_id)[0][0]) if (self.want_id in ids and self.want_id >= 0) else 0
            mc = [corners[idx]]

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(mc, self.mlen, self.K, self.D)
            rvec, tvec = rvec[0][0], tvec[0][0]
            rpy = rvec_to_rpy(rvec)

            pose_msg = Float64MultiArray()
            pose_msg.data = [tvec[0], tvec[1], tvec[2], rpy[0], rpy[1], rpy[2]]
            self.pose_pub.publish(pose_msg)
            if not self.marker_visible:
                self.get_logger().info('Marker detected - enabling ArUco control')
                self.state_pub.publish(Bool(data=True))
                self.marker_visible = True
                self.last_seen_time = time.time()
            self.get_logger().info(f"Published ArUco pose (tvec): {tvec} (rpy): {rpy}")

            cv2.aruco.drawDetectedMarkers(img, mc, np.array([[ids[idx]]]))
            cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, self.mlen * 0.5)
        else:
            self.marker_visible = False
            self.state_pub.publish(Bool(data=False))
        cv2.imshow('aruco', img)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            rclpy.shutdown()

    def visibility_check(self):
        if time.time() - self.last_seen_time > self.visibility_timeout and self.marker_visible:
            self.marker_visible = False
            self.get_logger().warn('Marker lost – pausing pose publication')

def main():
    rclpy.init()
    rclpy.spin(ArucoView())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()