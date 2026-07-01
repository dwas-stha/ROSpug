#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from ros_robot_controller.srv import GetBusServoState, GetBusServoStateRequest
from ros_robot_controller.msg import GetBusServoCmd

JOINT_ORDER = [
    "rf_joint", "rf_thigh", "rf_calf",
    "lf_joint", "lf_thigh", "lf_calf",
    "rb_joint", "rb_thigh", "rb_calf",
    "lb_joint", "lb_thigh", "lb_calf",
]

# servo_id: (joint_name, neutral_raw, radians_per_tick, sign)
#
# Neutral values are from your latest scan.
# Scale assumes approx 0-1000 ticks over 240 degrees:
# 240 deg = 4.18879 rad, so 4.18879 / 1000 = 0.00418879 rad/tick.
#
# Signs may need correction after visual testing.
SCALE = 0.00418879

SERVO_MAP = {
    # IDs 1-3: right back
    1:  ("rf_joint",  500, SCALE,  1),
    2:  ("rf_thigh",  319, SCALE, -1),
    3:  ("rf_calf",   521, SCALE, -1),

    # IDs 4-6: left back
    4:  ("lf_joint",  501, SCALE,  1),
    5:  ("lf_thigh",  681, SCALE, -1),
    6:  ("lf_calf",   479, SCALE, -1),

    # IDs 7-9: right front
    7:  ("rb_joint",  505, SCALE,  1),
    8:  ("rb_thigh",  313, SCALE, -1),
    9:  ("rb_calf",   520, SCALE, -1),

    # IDs 10-12: left front
    10: ("lb_joint",  500, SCALE,  1),
    11: ("lb_thigh",  687, SCALE, -1),
    12: ("lb_calf",   476, SCALE, -1),
}

def make_cmd(servo_id):
    cmd = GetBusServoCmd()
    cmd.id = servo_id
    cmd.get_id = 0
    cmd.get_position = 1
    cmd.get_offset = 0
    cmd.get_voltage = 0
    cmd.get_temperature = 0
    cmd.get_position_limit = 0
    cmd.get_voltage_limit = 0
    cmd.get_max_temperature_limit = 0
    cmd.get_torque_state = 0
    return cmd

def raw_to_rad(raw, neutral, scale, sign):
    return sign * (float(raw) - float(neutral)) * float(scale)

def main():
    rospy.init_node("bus_servo_joint_state_bridge")

    service_name = "/ros_robot_controller/bus_servo/get_state"
    rospy.wait_for_service(service_name)
    get_state = rospy.ServiceProxy(service_name, GetBusServoState)

    # pub = rospy.Publisher("/joint_states_real", JointState, queue_size=10)
    pub_topic = rospy.get_param("~publish_topic", "/joint_states")
    pub = rospy.Publisher(pub_topic, JointState, queue_size=10)
    mirror_pub = rospy.Publisher("/joint_states_real", JointState, queue_size=10)

    last_joint_positions = {name: 0.0 for name in JOINT_ORDER}
    
    rate_hz = rospy.get_param("~rate", 20)
    rate = rospy.Rate(rate_hz)

    servo_ids = sorted(SERVO_MAP.keys())

    rospy.loginfo("Publishing /joint_states_real from bus servo positions")

    while not rospy.is_shutdown():
        req = GetBusServoStateRequest()
        req.cmd = [make_cmd(sid) for sid in servo_ids]

        try:
            res = get_state(req)
        except Exception as e:
            rospy.logwarn("Servo state service failed: %s", e)
            rate.sleep()
            continue

        #joint_positions = {name: 0.0 for name in JOINT_ORDER}

        for servo_id, state in zip(servo_ids, res.state):
            if not state.position:
                rospy.logwarn_throttle(2.0, "No position returned for servo id %s", servo_id)
                continue

            joint_name, neutral, scale, sign = SERVO_MAP[servo_id]
            raw = state.position[0]
            #joint_positions[joint_name] = raw_to_rad(raw, neutral, scale, sign)
            last_joint_positions[joint_name] = raw_to_rad(raw, neutral, scale, sign)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_ORDER
        #msg.position = [joint_positions[name] for name in JOINT_ORDER]
        msg.position = [last_joint_positions[name] for name in JOINT_ORDER]

        msg.velocity = []
        msg.effort = []

        pub.publish(msg)
        mirror_pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
