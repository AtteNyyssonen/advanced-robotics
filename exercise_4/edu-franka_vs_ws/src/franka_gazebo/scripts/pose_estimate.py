#!/usr/bin/env python3
import rclpy, cv2, numpy as np, time
from math import pi
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from tf_transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix, euler_matrix
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray, Bool



def matrix_to_t_rpy(T):
    """Extract translation (3) and RPY (3) from 4x4 matrix."""
    t = T[:3, 3].tolist()
    rpy = euler_from_matrix(T[:3, :3], axes='sxyz')
    return t, rpy

class ArucoView(Node):
    def __init__(self):
        super().__init__('aruco_view')
        # params
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dictionary', 'DICT_6X6_250')
        self.declare_parameter('marker_id', -1)
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('marker_frame', 'marker')
        self.declare_parameter('goal_frame', 'goal')
        self.declare_parameter('ee_frame', 'end_effector_py')
        self.declare_parameter('camera_frame', 'camera')

        self.declare_parameter('pose_topic', '/command')
        
        self.declare_parameter('marker_visibility_timeout', 1.0)
        self.declare_parameter('state_topic', 'aruco_state')

        # publishers for pose & detection state
        pose_topic = self.get_parameter('pose_topic').value
        state_topic = self.get_parameter('state_topic').value
        self.pose_pub = self.create_publisher(Float64MultiArray, pose_topic, 10)
        self.state_pub = self.create_publisher(Bool, state_topic, 10)
        # visibility / publish state
        self.marker_visible = False
        self.last_seen_time = 0.0
        self.visibility_timeout = float(self.get_parameter('marker_visibility_timeout').value)


        it = self.get_parameter('image_topic').value
        ct = self.get_parameter('camera_info_topic').value
        dict_name = self.get_parameter('dictionary').value
        self.want_id = int(self.get_parameter('marker_id').value)
        self.mlen = float(self.get_parameter('marker_length_m').value)
        
        self.base_frame = self.get_parameter('base_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        pt = self.get_parameter('pose_topic').value


        # camera frame from end effector
        tx, ty, tz = 0.0, 0.0, 0.15
        roll, pitch, yaw = 0.0, 0.0, 0.0
        # build rotation matrix from rpy and place into 4x4 homogeneous matrix
        T = euler_matrix(roll, pitch, yaw)  # returns 4x4
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz
        # store as T_ee_cam: maps points in camera frame -> points in ee frame if you choose that convention
        # (use consistently when chaining transforms)
        self.T_ee_cam = T

        self.endEffector_pos = [0.0]*6
        # subscribe to end_effector_pose published as std_msgs/Float64MultiArray
        self.ee_sub = self.create_subscription(Float64MultiArray, '/end_effector_pose', self.on_end_effector, 10)

        self.T_base_ee = np.eye(4)
        self.T_base_camera = np.eye(4)


        self.camera_frame = self.get_parameter('camera_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # dictionary (compat)
        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_6X6_250)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)

        # detector params (compat)
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.detector_params = cv2.aruco.DetectorParameters_create()

        # try new detector, else fallback flag
        self.detector = None
        self.use_new = False
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new = True

        # intrinsics
        self.K = None; self.D = None
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, ct, self.on_info, 1)
        self.create_subscription(Image, it, self.on_img, 10)

        cv2.namedWindow('aruco', cv2.WINDOW_NORMAL)
        self.get_logger().info('aruco_view running – press q/Esc to quit')


    def broadcast_matrix(self, T, parent_frame: str, child_frame: str):
        """
        Fill a TransformStamped from 4x4 homogeneous matrix T and broadcast it.
        - T: 4x4 numpy array (homogeneous transform)
        - parent_frame: header.frame_id
        - child_frame: header.child_frame_id
        - stamp: optional rclpy.time.Time; if None uses self.get_clock().now()
        Returns the TransformStamped that was sent.
        """

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = parent_frame
        ts.child_frame_id = child_frame

        ts.transform.translation.x = float(T[0, 3])
        ts.transform.translation.y = float(T[1, 3])
        ts.transform.translation.z = float(T[2, 3])

        quat = quaternion_from_matrix(T)  # returns [x,y,z,w]
        ts.transform.rotation.x = float(quat[0])
        ts.transform.rotation.y = float(quat[1])
        ts.transform.rotation.z = float(quat[2])
        ts.transform.rotation.w = float(quat[3])

        # broadcast and return the message for convenience
        self.tf_broadcaster.sendTransform(ts)
        return ts



    # callback for end_effector_pose subscriber
    # calculates the 4x4 homogeneous transform from base to end-effector
    # NOT ideal – should get this from TF directly
    def on_end_effector(self, msg: Float64MultiArray):
        # expect [x y z roll pitch yaw]
        if msg.data and len(msg.data) >= 6:
            self.endEffector_pos = list(msg.data[:6])

            # build 4x4 transform from [x,y,z,roll,pitch,yaw] (angles in radians)
            x, y, z, roll, pitch, yaw = self.endEffector_pos
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)

            # rotation matrices about x,y,z
            Rx = np.array([[1, 0, 0],
                           [0, cr, -sr],
                           [0, sr,  cr]])
            Ry = np.array([[ cp, 0, sp],
                           [  0, 1,  0],
                           [-sp, 0, cp]])
            Rz = np.array([[cy, -sy, 0],
                           [sy,  cy, 0],
                           [ 0,   0, 1]])

            # convention: R = Rz * Ry * Rx  (yaw->pitch->roll)
            R = Rz @ Ry @ Rx

            T = np.eye(4)
            T[:3, :3] = R
            T[0, 3] = x
            T[1, 3] = y
            T[2, 3] = z -0.5

            self.T_base_ee = T
            self.T_base_camera = T @ self.T_ee_cam
        else:
            self.get_logger().warn('end_effector_pose message has <6 elements')


        


    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1,1) if msg.d else np.zeros((5,1))

    def on_img(self, msg: Image):
        if self.K is None: return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # if camera_frame not provided as param, use image header frame
        if not self.camera_frame:
            self.camera_frame = msg.header.frame_id

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

            # Convert rotation vector → rotation matrix
            R, _ = cv2.Rodrigues(np.array(rvec).reshape(3,))

            # Ensure translation is column vector
            t = np.array(tvec).reshape(3, 1)

            # Build 4x4 matrix
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = R
            T_cam_marker[:3, 3:] = t

            # to get the transformation from camera to marker, we need to invert T_cam_marker
            T_cam_marker = np.linalg.inv(T_cam_marker)



            # Compute T_base_marker = T_base_ee * T_ee_cam * T_cam_marker
            # Assume T_ee_cam is identity (i.e., camera is at end-effector)
            T_base_marker = self.T_base_ee @ T_cam_marker

            #test
            self.broadcast_matrix(self.T_base_camera, self.base_frame, self.camera_frame)
            self.broadcast_matrix(self.T_base_ee, self.base_frame, self.ee_frame)

            goal_offset_transform = euler_matrix(np.pi, 0.0, np.pi/2, 'sxyz')
            goal_offset_transform[0,3] = 0.0
            goal_offset_transform[1,3] = 0.0
            goal_offset_transform[2,3] = 0.7

            T_base_goal = T_base_marker @ goal_offset_transform

            # publish goal frame using helper
            self.broadcast_matrix(T_base_goal, self.base_frame, self.goal_frame)


            # compute target pose expressed in the marker frame (translation + rpy)
            t_target, rpy_target = matrix_to_t_rpy(T_base_goal)

            # compute target pose in marker frame
            pose_msg = Float64MultiArray()
            pose_msg.data = [float(t_target[0]), float(t_target[1]), float(t_target[2]),
                             float(rpy_target[0]), float(rpy_target[1]), float(rpy_target[2])]
            self.get_logger().info(f"pose message {pose_msg.data}")





            # for test do not bublish
            """
            self.pose_pub.publish(pose_msg)

            if not self.marker_visible:
                self.get_logger().info('Marker detected - enabling ArUco control')
                self.state_pub.publish(Bool(data=True))
                self.marker_visible = True
                self.last_seen_time = time.time()
            """
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

