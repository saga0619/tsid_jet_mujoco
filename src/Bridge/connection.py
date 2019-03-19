import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from mujoco_ros_msgs.msg import SensorState as MujocoSensorState
from mujoco_ros_msgs.msg import JointState as MujocoJointState
from mujoco_ros_msgs.msg import JointSet as MujocoJointSet

from Bridge import configuration as cf

import numpy as np


class mujoco:
    def __init__(self):
        print('mj python bride initializing')
        rospy.init_node('jet_python_mujoco')

        self.mode = 'position'

        self.setjoint = rospy.Publisher(
            '/mujoco_ros_interface/joint_set', MujocoJointSet, queue_size=1)
        self.simcommandpub = rospy.Publisher(
            '/mujoco_ros_interface/sim_command_con2sim', String, queue_size=1)
        self.simcommandsub = rospy.Subscriber(
            '/mujoco_ros_interface/sim_command_sim2con', String, self.simCommandCallback)

        self.jointStateSub = rospy.Subscriber(
            '/mujoco_ros_interface/joint_states', JointState, self.jointStateCallback, tcp_nodelay=True)
        self.simTimeSub = rospy.Subscriber(
            '/mujoco_ros_interface/sim_time', Float32, self.simTimeCallback, tcp_nodelay=True)
        self.sensorStateSub = rospy.Subscriber(
            '/mujoco_ros_interface/sensor_states', MujocoSensorState, self.sensorStateCallback, tcp_nodelay=True)

        self.jointSet_msg = MujocoJointSet()
        self.jointSet_msg.MODE = 0  # Position mode 0 Torque mode 1
        self.jointSet_msg.position = [0.0] * cf.dof
        self.jointSet_msg.torque = [0.0] * cf.dof

        self.q = np.array(np.zeros(cf.dof))
        self.qdot = np.array(np.zeros(cf.dof))
        self.torque = np.array(np.zeros(cf.dof))
        self.joint_names_mj = [''] * cf.dof

        self.mujoco_ready = False
        self.mujoco_init_receive = False
        print('sim ready ? ')
        self.sim_ready()
        print('PYTHON BRIDGE READY ! ')

    def sim_ready(self):
        r = rospy.Rate(1)
        while ((not self.mujoco_ready) & (not rospy.is_shutdown())):
            r.sleep()
            print('waiting : please reset mujoco with backspace')
        self.mujoco_ready = False

    def simCommandCallback(self, msg):
        print(msg)
        self.test = msg
        if msg.data == "RESET":
            # controller_param_init_here
            print('reset check')
            self.mujoco_ready = True
            self.simcommandpub.publish('RESET')
            r = rospy.Rate(100)
            while ((not self.mujoco_init_receive) & (not rospy.is_shutdown())):
                r.sleep()
        if msg.data == "INIT":
            print('init check')
            self.mujoco_init_receive = True
            self.simcommandpub.publish('INIT')

    def jointStateCallback(self, msg):
        for i in range(cf.dof):
            for j in range(cf.dof):
                if cf.joint_names[i] == msg.name[j+6]:
                    self.q[i] = msg.position[j+6]
                    self.qdot[i] = msg.velocity[j+6]
                    #self.torque[i] = msg.effort[j+6]
            self.joint_names_mj[i] = msg.name[i + 6]

    def simTimeCallback(self, msg):
        self.mj_time = msg.data

    def sensorStateCallback(self, msg):
        len(msg.sensor)

    def setMototState(self, qdes):
        if self.mode == 'position':
            for i in range(cf.dof):
                for j in range(cf.dof):
                    if cf.joint_names[i] == self.joint_names_mj[j]:
                        self.jointSet_msg.position[j] = qdes[i]
        self.jointSet_msg.header.stamp = rospy.Time.now()
        self.jointSet_msg.time = self.mj_time
        self.setjoint.publish(self.jointSet_msg)

    def simTogglePlay(self):
        self.simcommandpub.publish('pause')

    def simReset(self):
        self.simcommandpub.publish('mjreset')
