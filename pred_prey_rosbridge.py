#!/usr/bin/env python

## Python bridge to ROS which the predator and prey can both use

import sys, getopt, types
  
PKG = 'pred_prey_turtles' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose
from turtlesim.msg import Velocity
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.srv import SpawnRequest
from turtlesim.srv import KillRequest

from py_interface import erl_node, erl_opts, erl_eventhandler, erl_term

SELF_NODE_NAME = "pred_prey_rosbridge@localhost"
ERLANG_COOKIE = "pred_prey_erlang_cookie"
SELF_NODE_REGISTERED_PROCESS = "pred_prey_erlang_mailbox"
ROS_NODE_NAME = 'pred_prey_rosbridge'
ROS_SERVICE_SPAWN = "spawn"
ROS_SERVICE_KILL = "kill"
VERBOSE = True

class RosBridge:

    def __init__(self):

        self.evhand = None
        self.mailbox = None
        self.subscriber_registry = {}  # topic -> remote_pid
        self.publisher_registry = {}   # topic -> rospy.publisher instance

        self.init_ros()
        node = self.init_erlang_node()
        self.init_erlang_mailbox(node)
        self.run_erlang_event_handler() # blocks
        
    def ros_receive_prey_pose_message(self, data):
        self.ros_receive_pose_message("/prey/pose", data)
    
    def ros_receive_predator_pose_message(self, data):
        self.ros_receive_pose_message("/predator/pose", data)

    def ros_receive_pose_message(self, topic, data):

        if VERBOSE:
            rospy.loginfo("Ros " + topic + " messsage " + rospy.get_caller_id() + " x: %s y: %s", data.x, data.y)
    
        subscribers = self.subscriber_registry[topic]
        for remote_pid in subscribers:
            self.send_turtle_pose_erlang(remote_pid, topic, data)


    def erlang_node_receive_message(self, msg, *k, **kw):
        if VERBOSE:
            print "Incoming msg=%s (k=%s, kw=%s)" % (`msg`, `k`, `kw`)

        remote_pid = msg[0]
        msg_type = msg[1]

        if str(msg_type) == "stop":
            print "Exiting"
            self.evhand.StopLooping()
        elif str(msg_type) == ROS_SERVICE_KILL:
            turtle_type = msg[2]
            self.kill_turtle(remote_pid, turtle_type)
        elif str(msg_type) == ROS_SERVICE_SPAWN:
            spawn_params_tuple = msg[2]
            self.spawn_turtle(remote_pid, spawn_params_tuple)
        elif str(msg_type) == "subscribe":
            # TODO: the topic name should come in the message, and based on
            # topic name, figure out the message type and callback handler
            # rospy.Subscriber(ROS_TOPIC_POSE, Pose, self.ros_receive_topic_message)
            topic_name = msg[2]
            self.subscribe_process_to_topic(remote_pid, topic_name)
        elif str(msg_type) == "command_velocity":
            turtle_type = msg[2]
            velocity_tuple = msg[3]
            # velocity = Velocity(velocity_tuple)
            if VERBOSE:
                print "Moving the turtle"
            topic = "/%s/command_velocity" % turtle_type
            publisher = self.publisher_registry[topic]
            publisher.publish(velocity_tuple[0], velocity_tuple[1])

    def subscribe_process_to_topic(self, remote_pid, topic_name):

        # does our subscriber_registry dictionary already contain that topic?
        # if not, we need to create it in ros
        # add this process to the list of processes that will have topic messages forwarded
        
        if not self.subscriber_registry.has_key(topic_name):
            subscribers = [remote_pid]
            self.subscriber_registry[topic_name] = subscribers
            if topic_name == "/prey/pose":
                rospy.Subscriber(topic_name, Pose, self.ros_receive_prey_pose_message)
            elif topic_name == "/predator/pose":
                rospy.Subscriber(topic_name, Pose, self.ros_receive_predator_pose_message)

        else:
            subscribers = self.subscriber_registry[topic_name]
            subscribers.append(remote_pid)
            self.subscriber_registry[topic_name] = subscribers


    def create_command_velocity_publisher(self, remote_pid, turtle_type):
        topic = "/%s/command_velocity" % turtle_type
        self.publisher_registry[topic] = rospy.Publisher(topic, Velocity) 

    def spawn_turtle(self, remote_pid, spawn_params_tuple):
        print "Spawning a turtle"
        rospy.wait_for_service(ROS_SERVICE_SPAWN)
        try:
            spawn = rospy.ServiceProxy(ROS_SERVICE_SPAWN, Spawn)
            spawn_response = spawn.call(SpawnRequest(*spawn_params_tuple))
            turtle_type = spawn_params_tuple[3]
            self.create_command_velocity_publisher(remote_pid, turtle_type)
            print spawn_response
        except rospy.ServiceException, e:
            print "Service call failed to spawn turtle: %s" % e        
            self.mailbox.Send(remote_pid, erl_term.ErlAtom("stop"))

    def kill_turtle(self, remote_pid, turtle_type):
        print "Killing turtle %s" % turtle_type
        rospy.wait_for_service(ROS_SERVICE_KILL)
        try:
            kill = rospy.ServiceProxy(ROS_SERVICE_KILL, Kill)
            kill_response = kill.call(KillRequest(turtle_type))
            print kill_response
        except rospy.ServiceException, e:
            print "Service call failed to kill turtle: %s" % e        
            self.mailbox.Send(remote_pid, erl_term.ErlAtom("stop"))


    def send_turtle_pose_erlang(self, remote_pid, topic, data):
        topic_atom = erl_term.ErlAtom("%s" % topic)
        msg_tuple = [topic_atom,
                     erl_term.ErlNumber(data.x), 
                     erl_term.ErlNumber(data.y),
                     erl_term.ErlNumber(data.theta),
                     erl_term.ErlNumber(data.linear_velocity),
                     erl_term.ErlNumber(data.angular_velocity)]
        msg_data = erl_term.ErlTuple(msg_tuple)
        self_node_name = erl_term.ErlAtom("%s" % SELF_NODE_NAME)
        self_reg_process = erl_term.ErlAtom("%s" % SELF_NODE_REGISTERED_PROCESS)
        return_addr = erl_term.ErlTuple([self_node_name, self_reg_process])
        msg = erl_term.ErlTuple([return_addr, msg_data])
        self.mailbox.Send(remote_pid, msg)
        if VERBOSE:
            print "Sent %s message to %s" % (topic, remote_pid)

    def init_erlang_node(self):
        node = erl_node.ErlNode(SELF_NODE_NAME, erl_opts.ErlNodeOpts(cookie=ERLANG_COOKIE))
        node.Publish()
        return node

    def init_erlang_mailbox(self, node):
        self.mailbox = node.CreateMBox(self.erlang_node_receive_message)
        self.mailbox.RegisterName(SELF_NODE_REGISTERED_PROCESS)

    def run_erlang_event_handler(self):
        if VERBOSE:
            print "Waiting for Erlang node messages.. kill this process via 'kill -9'"
        self.evhand = erl_eventhandler.GetEventHandler()
        self.evhand.Loop()    

    def init_ros(self):
        rospy.init_node(ROS_NODE_NAME, anonymous=True)

if __name__ == '__main__':
    rosbridge = RosBridge()
