#!/usr/bin/env python
import rospy
import PyKDL

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from kdl_parser_py.urdf import treeFromParam
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_matrix,
)
from std_msgs.msg import Float32MultiArray

class BodyPoseEstimator(object):
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base_footprint")

        self.joint_topic = rospy.get_param("~joint_topic", "/joint_states_real")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")

        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.filter_alpha = float(rospy.get_param("~filter_alpha", 0.75))
        self.foot_ground_offset = float(rospy.get_param("~foot_ground_offset", 0.0))

        self.roll_sign = float(rospy.get_param("~roll_sign", 1.0))
        self.pitch_sign = float(rospy.get_param("~pitch_sign", 1.0))
        self.yaw_sign = float(rospy.get_param("~yaw_sign", 1.0))
        self.use_imu_yaw = bool(rospy.get_param("~use_imu_yaw", False))
        
        # self.lock_world_yaw = bool(rospy.get_param("~lock_world_yaw", True)) # trying to fix the model slowly turning
        # self.freeze_yaw_when_idle = bool(rospy.get_param("~freeze_yaw_when_idle", True))
        # self.yaw_motion_threshold = float(rospy.get_param("~yaw_motion_threshold", 0.01))
        # self.last_fixed_yaw = 0.0


        self.min_support_feet = int(rospy.get_param("~min_support_feet", 3))
        self.support_threshold = float(rospy.get_param("~support_threshold", 0.015))
        
        self.foot_links = rospy.get_param("~foot_links", [
            "calf_Link_1",
            "calf_Link_2",
            "calf_Link_3",
            "calf_Link_4",
        ])

        # Approximate foot-tip offsets in each calf link frame.
        # These will need calibration.
        self.foot_offsets = rospy.get_param("~foot_offsets", {
            "calf_Link_1": [0.11, 0.0, 0.0],
            "calf_Link_2": [0.0, 0.11, 0.0],
            "calf_Link_3": [0.11, 0.0, 0.0],
            "calf_Link_4": [0.0, 0.11, 0.0],
        })

        self.joint_positions = {}
        self.latest_quat = None
        self.last_z = None
        self.imu_zero = None
        self.latest_foot = []

        self.fk_solvers = {}

        self.odom_pub = rospy.Publisher("/digital_twin/odom", Odometry, queue_size=10)
        self.foot_pub = rospy.Publisher("/digital_twin/feet", Float32MultiArray, queue_size=10)

        rospy.loginfo("Waiting for /robot_description...")
        while not rospy.is_shutdown() and not rospy.has_param("/robot_description"):
            rospy.sleep(0.2)

        self.build_fk_solvers()

        rospy.Subscriber(self.joint_topic, JointState, self.joint_cb, queue_size=10)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("Body pose estimator started.")
        rospy.loginfo("Subscribing joint states: %s", self.joint_topic)
        rospy.loginfo("Subscribing IMU: %s", self.imu_topic)
        rospy.loginfo("Publishing odom: /digital_twin/odom")

    def angle_diff(self, a, b):
        d = a - b
        while d > 3.141592653589793:
            d -= 2.0 * 3.141592653589793
        while d < -3.141592653589793:
            d += 2.0 * 3.141592653589793
        return d

    def build_fk_solvers(self):
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            raise RuntimeError("Failed to parse /robot_description into KDL tree")

        for link_name in self.foot_links:
            chain = tree.getChain(self.base_frame, link_name)
            solver = PyKDL.ChainFkSolverPos_recursive(chain)

            joint_names = []
            for i in range(chain.getNrOfSegments()):
                joint = chain.getSegment(i).getJoint()
                if joint.getTypeName() != "None":
                    joint_names.append(joint.getName())

            self.fk_solvers[link_name] = {
                "chain": chain,
                "solver": solver,
                "joint_names": joint_names,
            }

            rospy.loginfo(
                "FK chain %s -> %s has %d joints: %s",
                self.base_frame,
                link_name,
                len(joint_names),
                joint_names,
            )

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def imu_cb(self, msg):
        q = msg.orientation
        raw_q = [q.x, q.y, q.z, q.w]

        # roll, pitch, yaw = euler_from_quaternion(raw_q)

        # roll *= self.roll_sign
        # pitch *= self.pitch_sign
        imu_roll, imu_pitch, imu_yaw = euler_from_quaternion(raw_q)
        mapped_roll = imu_roll
        mapped_pitch = imu_yaw
        mapped_yaw = imu_pitch

        if self.imu_zero is None:
            self.imu_zero = {
                "roll": mapped_roll,
                "pitch": mapped_pitch,
                "yaw": mapped_yaw,
            }
            rospy.loginfo(
                "IMU zero set: roll=%.3f pitch=%.3f yaw=%.3f",
                mapped_roll,
                mapped_pitch,
                mapped_yaw,
            )

        #roll = self.roll_sign * imu_roll
        #pitch = self.pitch_sign * imu_yaw
        roll = self.roll_sign * self.angle_diff(mapped_roll, self.imu_zero["roll"])
        pitch = self.pitch_sign * self.angle_diff(mapped_pitch, self.imu_zero["pitch"])


        if self.use_imu_yaw:
            # yaw = self.yaw_sign * imu_pitch
            yaw = self.yaw_sign * self.angle_diff(mapped_yaw, self.imu_zero["yaw"])
        else:
            yaw = 0.0

        # self.latest_quat = quaternion_from_euler(roll, pitch, yaw)
        fixed_roll = yaw
        fixed_pitch = roll
        fixed_yaw = pitch
        

        self.latest_quat = quaternion_from_euler(fixed_roll, fixed_pitch,fixed_yaw)

    def compute_foot_point_base(self, link_name):
        data = self.fk_solvers[link_name]
        chain = data["chain"]
        solver = data["solver"]
        joint_names = data["joint_names"]

        q = PyKDL.JntArray(len(joint_names))

        for i, joint_name in enumerate(joint_names):
            q[i] = self.joint_positions.get(joint_name, 0.0)

        frame = PyKDL.Frame()
        solver.JntToCart(q, frame)

        offset = self.foot_offsets.get(link_name, [0.0, 0.0, 0.0])
        local_tip = PyKDL.Vector(
            float(offset[0]),
            float(offset[1]),
            float(offset[2]),
        )

        p = frame * local_tip

        return [p.x(), p.y(), p.z()]

    def estimate_base_z(self):
        if self.latest_quat is None:
            return None

        rot = quaternion_matrix(self.latest_quat)

        raw_feet = []

        for link_name in self.foot_links:
            p = self.compute_foot_point_base(link_name)

            # Only need world Z component:
            # z_world = R[2,0]*x + R[2,1]*y + R[2,2]*z
            x = rot[0][0] * p[0] + rot[0][1] * p[1] + rot[0][2] * p[2]
            y = rot[1][0] * p[0] + rot[1][1] * p[1] + rot[1][2] * p[2]
            z = rot[2][0] * p[0] + rot[2][1] * p[1] + rot[2][2] * p[2]

            raw_feet.append({
                "name": link_name,
                "x": x,
                "y": y,
                "z_no_base": z
            })

        if not raw_feet: 
            return None

        # Put the lowest estimated foot on ground z=0.
        # min_foot_z = min(f["z_no_base"] for f in raw_feet)
        
        sorted_feet = sorted(raw_feet, key=lambda f: f["z_no_base"])
        min_foot_z = sorted_feet[0]["z_no_base"]

        support_feet = [
            f for f in sorted_feet
            if (f["z_no_base"] - min_foot_z) <= self.support_threshold
        ]

        if (len(support_feet) < self.min_support_feet):
            support_feet = sorted_feet[:self.min_support_feet]

        support_z_mean = sum(f["z_no_base"] for f in support_feet) / float(len(support_feet))
        
        target_z = -support_z_mean + self.foot_ground_offset
        support_names = set(f["name"] for f in support_feet)


        if self.last_z is None:
            self.last_z = target_z
        else:
            self.last_z = (
                self.filter_alpha * self.last_z +
                (1.0 - self.filter_alpha) * target_z
            )

        self.latest_foot= []

        for f in raw_feet:
            contact = 1.0 if f["name"] in support_names else 0.0
            self.latest_foot.append({
                "name": f["name"],
                "x": f["x"],
                "y": f["y"],
                "z": f["z_no_base"] + self.last_z,
                "contact": contact,
            })

        return self.last_z

    def publish_foot(self):
        if not self.latest_foot:
            return

        msg = Float32MultiArray()

        data = []

        # Same order as self.foot_links
        by_name = {f["name"]: f for f in self.latest_foot}

        for link_name in self.foot_links:
            f = by_name.get(link_name)
            if f is None:
                data.extend([0.0, 0.0, 0.0, 0.0])
            else:
                data.extend([
                    float(f["x"]),
                    float(f["y"]),
                    float(f["z"]),
                    float(f["contact"]),
                ])

        msg.data = data
        self.foot_pub.publish(msg)

    def timer_cb(self, event):
        if self.latest_quat is None:
            return

        z = self.estimate_base_z()
        if z is None:
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.child_frame

        # For now x/y are fixed. Later replace these with real odom/SLAM/AprilTag.
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = z

        odom.pose.pose.orientation.x = self.latest_quat[0]
        odom.pose.pose.orientation.y = self.latest_quat[1]
        odom.pose.pose.orientation.z = self.latest_quat[2]
        odom.pose.pose.orientation.w = self.latest_quat[3]

        self.publish_foot()
        self.odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("body_pose_estimator")
    BodyPoseEstimator()
    rospy.spin()
