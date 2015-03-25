#!/usr/bin/python
import threading
from tornado.httpserver import HTTPServer

__author__ = 'Moon Kwon Kim <mkdmkk@gmail.com>'

import rospy
import geometry_msgs.msg as geometry_msgs
import tornado.ioloop
import tornado.web
from kobuki_msgs.msg import MotorPower

NODE_NAME = "turtlebot_control_websvc"


class TurtleBotControlWebSvc(tornado.web.RequestHandler):
    exposed = True
    cnt = 0

    def initialize(self, pub):
        self.pub = pub

    def get(self):
        linear = self.get_argument("linear", '0')
        angular = self.get_argument("angular", '0')

        ctr_msg = geometry_msgs.Twist()
        ctr_msg.linear.y = 0.0
        ctr_msg.linear.z = 0.0
        ctr_msg.angular.x = 0.0
        ctr_msg.angular.y = 0.0

        try:
            ctr_msg.linear.x = float(linear)
            print("Linear: "+str(linear))
        except:
            ctr_msg.linear.x = 0.0

        try:
            ctr_msg.angular.z = float(angular)
            print("Angular: "+str(angular))
        except:
            ctr_msg.angular.z = 0.0

        self.pub.publish(ctr_msg)

        self.write("Hello.")


class TurtleBotControlServer(threading.Thread):
    def __init__(self, pub):
        threading.Thread.__init__(self)
        self.pub = pub
        self.mot_pub = mot_pub

    def run(self):
        print("Starting web server...")
        application = tornado.web.Application([
            (r"/", TurtleBotControlWebSvc, dict(pub=self.pub)),
        ])
        server = HTTPServer(application)
        server.listen(8080)
        tornado.ioloop.IOLoop.instance().start()

    def stop(self):
        self.__stop = True


def echo(msg):
    print("echo!")


if __name__ == '__main__':
    rospy.loginfo("Starting the turtlebot control web service...")

    rospy.init_node(NODE_NAME)

    # Motor On
    mot_pub = rospy.Publisher('/mobile_base/commands/motor_power',MotorPower, queue_size=10)
    mot_pub.publish(MotorPower(MotorPower.ON))

    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", geometry_msgs.Twist, queue_size=10)
    sub = rospy.Subscriber("/cmd_vel_mux/input/teleop", geometry_msgs.Twist, echo)

    server = TurtleBotControlServer(pub)
    server.start()

    rospy.loginfo("Web service started.")

    rospy.spin()

    if rospy.is_shutdown():
        mot_pub.publish(MotorPower(MotorPower.OFF))
        server.stop()
