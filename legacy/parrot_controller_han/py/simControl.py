#!/usr/bin/env python

"""
This Python executable is part of the ram ROS package. Its function is to update the setpoints of multiple drones at the same time. The application is designed to work under Ubuntu Linux (and just that).
For documentation, check the Github repository. http://github.com/ceesietopc/ram_ba_package
"""

# System imports
import sys
import time
from gi.repository import Gtk, cairo, Pango, PangoCairo, GObject, Gdk   # We use glade for UI
import threading
import os
import xmlrpclib
import socket
import subprocess

# Ros related imports
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf.transformations
import rospkg
from ram.msg import nonlinearity

# RosConnector class to handle all communication with ROS
class RosConnector:
    def __init__(self, prefixes):
        # Set up dictionaries to store the current setpoint, the setpoint subscriber and the setpoint publisher per quadcopter.
        self.setpointDict = {}
        self.setpointSubDict = {}
        self.setpointPubDict = {}
        
        # Prefixes are directly deducted from executable arguments. 
        del prefixes[0] # First arg is the filename
        if len(prefixes) > 0:
            for prefix in prefixes:
                self.initPrefix(prefix)

    def initPrefix(self, prefix):
        # Initiate the handling of a new prefix. So: current setpoint is zero valued, initiate subscriber and publisher.
        self.setpointDict[prefix] = geometry_msgs.msg.Pose()
        self.setpointSubDict[prefix] = rospy.Subscriber("/"+prefix+"/setpoint", geometry_msgs.msg.Pose, self.setpointCB, callback_args=prefix)
        self.setpointPubDict[prefix] = rospy.Publisher("/"+prefix+"/setpoint", geometry_msgs.msg.Pose)

    def setpointCB(self, msg, prefix):
        # One callback is used for all subscribers. This callback just stores the pose received from the subscriber.
        self.setpointDict[prefix] = msg
   
    def clean(self):
        pass

    def publish(self):
        # Publish the new setpoints
        for (i, prefix) in enumerate(self.setpointPubDict):
            self.setpointPubDict[prefix].publish(self.setpointDict[prefix])

    # Button handler per button. This can implemented in a beautiful way, since this is dirty. It works though.
    def btnXL(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.x += 0.5
        self.publish()

    def btnXNL(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.x -= 0.5
        self.publish()

    def btnXS(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.x += 0.1
        self.publish()

    def btnXNS(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.x -= 0.05
        self.publish()

    def btnYL(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.y += 0.5
        self.publish()

    def btnYNL(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.y -= 0.5
        self.publish()

    def btnYS(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.y += 0.1
        self.publish()

    def btnYNS(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.y -= 0.05
        self.publish()

    def btnZL(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.z += 0.5
        self.publish()

    def btnZNL(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.z -= 0.5
        self.publish()

    def btnZS(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.z += 0.1
        self.publish()

    def btnZNS(self, btn):
        for setpoint in self.setpointDict:
            self.setpointDict[setpoint].position.z -= 0.05
        self.publish()

def btnClose(widget, event):
    # If the close button is pressed, clean up.
    ros.clean()
    # Exit application, go to terminal
    Gtk.main_quit(widget, event)

# Main function
if __name__ == "__main__":
    # Check if roscore is running
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('controller_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()

    # Init node
    rospy.init_node('bulk_setpoint_change')

    # Init ros communication, just pass on info from arguments
    ros = RosConnector(sys.argv)

    # Start threads
    GObject.threads_init()
    Gdk.threads_init()

    # Get location of Glade file and use for UI
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    gladefile = package_location+"/py/bulk_setpoint_change.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)

    # Connect signals
    handlers = {
        "quit": btnClose,
        "btnXL": ros.btnXL,
        "btnXNL": ros.btnXNL,
        "btnXS": ros.btnXS,
        "btnXNS": ros.btnXNS,
        "btnYL": ros.btnYL,
        "btnYNL": ros.btnYNL,
        "btnYS": ros.btnYS,
        "btnYNS": ros.btnYNS,
        "btnZL": ros.btnZL,
        "btnZNL": ros.btnZNL,
        "btnZS": ros.btnZS,
        "btnZNS": ros.btnZNS,
    }
    builder.connect_signals(handlers)

    # Get window and open it
    window = builder.get_object("window1")
    window.set_title("Bulk setpoint edit")
    window.show_all()

    # Start main loop
    Gdk.threads_enter()
    Gtk.main()
    Gdk.threads_leave()