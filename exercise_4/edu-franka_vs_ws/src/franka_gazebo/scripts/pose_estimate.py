#!/usr/bin/env python3
import rclpy, cv2, numpy as np, time
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix, euler_from_matrix
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge

def rvec_tvec_to_matrix(rvec, tvec):
    """Convert OpenCV rvec/tvec to 4x4 homogeneous matrix."""
    R, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec, dtype=np.float64).reshape(3)
    return T

def transform_stamped_to_matrix(ts: TransformStamped):
    """Convert geometry_msgs/TransformStamped to 4x4 matrix."""
    q = ts.transform.rotation
    t = ts.transform.translation
    q_arr = [q.x, q.y, q.z, q.w]
    T = quaternion_matrix(q_arr)
    T[:3, 3] = [t.x, t.y, t.z]
    return T

def matrix_to_t_rpy(T):
    """Extract translation (3) and RPY (3) from 4x4 matrix."""
    t = T[:3, 3].tolist()
    rpy = euler_from_matrix(T[:3, :3], axes='sxyz')
    return t, rpy

class ArucoView(Node):
    def __init__(self):
        super().__init__('aruco_view')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dictionary', 'DICT_6X6_250')
        self.declare_parameter('marker_id', -1)
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('pose_topic', '/command')
        self.declare_parameter('target_frame', 'panda_link0')

        it = self.get_parameter('image_topic').value
        ct = self.get_parameter('camera_info_topic').value
        dict_name = self.get_parameter('dictionary').value
        self.want_id = int(self.get_parameter('marker_id').value)
        self.mlen = float(self.get_parameter('marker_length_m').value)
        pt = self.get_parameter('pose_topic').value
        self.target_frame = self.get_parameter('target_frame').value

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.K = None; self.D = None
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, ct, self.on_info, 1)
        self.create_subscription(Image, it, self.on_img, 10)
        self.pose_pub = self.create_publisher(Float64MultiArray, pt, 1)
        self.state_pub = self.create_publisher(Bool, '/aruco_active', 1)

        self.marker_visible = False
        self.last_seen_time = 0.0
        self.visibility_timeout = 1

        cv2.namedWindow('aruco', cv2.WINDOW_NORMAL)
        self.get_logger().info('aruco_view running – publishing only when marker is detected')

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if msg.d else np.zeros((5, 1))
        self.camera_frame = msg.header.frame_id

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
            T_cam_marker = rvec_tvec_to_matrix(rvec, tvec)

            camera_frame = self.camera_frame if self.camera_frame else msg.header.frame_id
            stamp = None
            try:
                if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
                    stamp = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
                else:
                    stamp = rclpy.time.Time()
            except Exception:
                stamp = rclpy.time.Time()

            try:
                ts = self.tf_buffer.lookup_transform(self.target_frame, camera_frame, stamp, timeout=rclpy.duration.Duration(seconds=0.5))
                T_target_camera = transform_stamped_to_matrix(ts)
            except Exception as e:
                try:
                    ts = self.tf_buffer.lookup_transform(self.target_frame, camera_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
                    T_target_camera = transform_stamped_to_matrix(ts)
                    self.get_logger().warn(f"TF lookup with image stamp failed, used latest transform. Exception: {e}")
                except Exception as e2:
                    self.get_logger().warn(f"TF lookup failed: {e2}")
                    self.state_pub.publish(Bool(data=False))
                    self.marker_visible = False
                    return

            T_target_marker = np.dot(T_target_camera, T_cam_marker)
            t_target, rpy_target = matrix_to_t_rpy(T_target_marker)

            pose_msg = Float64MultiArray()
            pose_msg.data = [float(t_target[0]), float(t_target[1]), float(t_target[2]),
                             float(rpy_target[0]), float(rpy_target[1]), float(rpy_target[2])]
            self.pose_pub.publish(pose_msg)

            if not self.marker_visible:
                self.get_logger().info('Marker detected - enabling ArUco control')
                self.state_pub.publish(Bool(data=True))
                self.marker_visible = True
                self.last_seen_time = time.time()
            
            cv2.aruco.drawDetectedMarkers(img, mc, np.array([[ids[idx]]]))
            cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, self.mlen * 0.5)
        else:
            if time.time() - self.last_seen_time > self.visibility_timeout and self.marker_visible:
                self.marker_visible = False
                self.state_pub.publish(Bool(data=False))

        cv2.imshow('aruco', img)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(ArucoView())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()