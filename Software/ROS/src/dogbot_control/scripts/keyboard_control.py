#!/usr/bin/env python
#
# Controls a single motor from keyboard strokes
# Motors are position-controlled via controllers of type position_controllers/JointPositionController
#
# up/down arrows (or s and d keys) move the motor clockwise/anticlockwise
# 
# Janaury 2018, N Greenway, React AI Ltd 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import sys, getopt, curses


DEFAULT_CONTROLLER_PATTERN = "{ns}{joint}_position_controller/command"
DEFAULT_STATE_TOPIC  ="{ns}joint_states"
NODE_NAME = "keyboard_motor_controller"


class keyboardcontroller(object):

    def __init__(self, screen, joint, ns):
      self.screen = screen
      rospy.init_node(NODE_NAME, anonymous=True)
      
      #ns will be blank if fired up from a launch file
      self.ns = "/" + ns + "/" if ns else ""
      self.callerID = rospy.get_caller_id()
      self.realNamespace = ns if ns else rospy.get_namespace()
      self.jointnames = []
      self.errorstring = ""
      
      # subscribe to the current positions
      rospy.Subscriber(DEFAULT_STATE_TOPIC.format(ns=self.ns), JointState, self.onStateMsg)
      
      self.setJoint(joint)
      rospy.loginfo("Startup of keyboardcontroller {} for {} in namespace {}".format(self.callerID, joint, self.realNamespace))
    
    def setJoint(self, jointname):      
      self.jointReady = False
      self.jointname = jointname
      self.jointindex = -1
      self.controllerTopic = DEFAULT_CONTROLLER_PATTERN.format(joint = jointname, ns = self.ns)
      self.controlPublisher = rospy.Publisher(self.controllerTopic, Float64, queue_size=1)
      #rospy.loginfo("Switching to sending commands to {}".format(self.controllerTopic))
      self.basetopstring = "Keyboard control of {}.".format(self.jointname)
      self.topstring = self.basetopstring + " Ctrl-C to exit \n \n"
      
      
    def onStateMsg(self, data):
      #check the requested joint is there and note the position
      self.jointnames = list(data.name)
      try:
        self.jointindex = self.jointnames.index(self.jointname)
      except ValueError:
        self.jointReady = False
        self.errorstring = "Can't locate joint {} in joint state names: {}".format(self.jointname, data.name)
        print self.errorstring
        return;
      self.currentJointPosition = data.position[self.jointindex]
      self.desiredJointPosition = self.currentJointPosition
      self.errorstring = ""
      self.topstring = self.basetopstring + "  Current position: {}. Ctrl-C to exit \n".format(self.currentJointPosition)
      self.redraw()
      
    def incrementDesiredPosition(self, increment):
      self.desiredJointPosition += increment;
      
    def publish(self):
      if not self.jointReady: 
        self.redraw("not ready")
        return;
      msg = Float64()
      msg.data = self.desiredJointPosition
      print "publishing", msg
      self.controlPublisher.publish(msg)
      
      
    def switchJoint(self, move):
      if len(self.jointnames)==0: return;
    
      if self.jointindex == -1:
        self.jointindex=0
      else:
        self.jointindex += move
        if self.jointindex < 0: self.jointindex = len(self.jointnames)-1;
        if self.jointindex > len(self.jointnames)-1: self.jointindex=0;
        
      self.setJoint(self.jointnames[self.jointindex])
      
    def redraw(self, extra=""):
      #print "in redraw", self.topstring
      self.screen.erase()
      self.screen.addstr(self.topstring)
      if self.errorstring: self.screen.addstr(self.errorstring)
      if extra: self.screen.addstr(extra)



def usage():
    print('rosrun dogbot_control keyboard_control front_left_knee dogbot');


def main(argv):
    joint = argv[1] if len(argv) > 1 and argv[1][0:2] != "__" else ""
    if joint != "" and joint[-6:] != "_joint": joint += "_joint"
    
    ns = argv[2] if len(argv) > 2 and argv[2][0:2] != "__" else ""    
    
    #if not passed on command line, get the joint from the param server - TODO
    
    stdscr = curses.initscr()
    node = keyboardcontroller(stdscr, joint, ns)
    
    curses.noecho(); curses.cbreak(); stdscr.keypad(0)
    
    stdscr.addstr(node.topstring)
    
    while not rospy.is_shutdown():
      try:
        ch = stdscr.getch()
        newMsg = False

        # left arrow
        if ch == 68 or ch == ord('a'):
          node.switchJoint(-1)
        # up arrow
        elif ch == 65 or ch == ord('w'):
          node.incrementDesiredPosition(1.0)
          newMsg = True;
          
        # right arrow
        elif ch == 67 or ch == ord('d'):
          node.switchJoint(1)
        # down arrow
        elif ch == 66 or ch == ord('s'):
          node.incrementDesiredPosition(-1.0)
          newMsg = True;
        
        #TODO - enter multiple-digit values
        
        elif ch == ord('h'):
          node.redraw("h: help \n"
            "a/w/d/s or arrow keys to control \n"
            "up/down or w/s move the joint \n"
            "left/right ot a/d switch the joint under control \n")
        
        else:
          newMsg = False
        
        node.redraw()
        if newMsg: node.publish()

        
      except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt, exiting {} node".format(NODE_NAME))
        break;
        
      except Exception, e:
        node.redraw( "Error in {} node: {}".format(NODE_NAME, str(e)) )
        break
      
    resetScreen(stdscr)
    return;
  
      
def resetScreen(stdscr=None):     
    if not stdscr:  stdscr = curses.initscr()
    curses.nocbreak(); stdscr.keypad(0); curses.echo()
    curses.endwin()      
    
    
if __name__ == '__main__':
    try:
      main(sys.argv)
      
    except rospy.ROSInterruptException:
      resetScreen()
    except Exception, e:
      resetScreen()
      print "Error", str(e);     
    
    
    
    
    