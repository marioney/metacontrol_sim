#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import roslib
import rospy
import rostopic

import argparse
import importlib
import sys

from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String
from threading import Lock

"""
@author: enriquefernandez
Original file: https://github.com/ros/ros_comm/blob/noetic-devel/tools/topic_tools/scripts/transform
CHANGES:
    - added Lock to original TopicOp class
    - added a service to change transform function at run-time
"""


class TopicOp:

    def __init__(self):
        parser = argparse.ArgumentParser(
            formatter_class=argparse.RawTextHelpFormatter,
            description='Apply a Python operation to a topic.\n\n'
                        'A node is created that subscribes to a topic,\n'
                        'applies a Python expression to the topic (or topic\n'
                        'field) message \'m\', and publishes the result\n'
                        'through another topic.\n\n'
                        'Usage:\n\trosrun topic_tools transform '
                        '<input> <output topic> <output type> '
                        '[<expression on m>] [--import numpy tf]\n\n'
                        'Example:\n\trosrun topic_tools transform /imu/orientation '
                        '/norm std_msgs/Float64 '
                        '\'sqrt(sum(array([m.x, m.y, m.z, m.w])))\'')
        parser.add_argument('input', help='Input topic or topic field.')
        parser.add_argument('output_topic', help='Output topic.')
        parser.add_argument('output_type', help='Output topic type.')
        parser.add_argument(
            'expression', default='m',
            help='Python expression to apply on the input message \'m\'.'
        )
        parser.add_argument(
            '-i', '--import', dest='modules', nargs='+', default=['numpy'],
            help='List of Python modules to import.'
        )
        parser.add_argument(
            '--wait-for-start', action='store_true',
            help='Wait for input messages.'
        )
        parser.add_argument(
            '--latch', action='store_true',
            help='Set publisher to latched.'
        )

        # get and strip out ros args first
        argv = rospy.myargv()
        args = parser.parse_args(argv[1:])

        self.modules = {}
        for module in args.modules:
            try:
                mod = importlib.import_module(module)
            except ImportError:
                print('Failed to import module: %s' % module, file=sys.stderr)
            else:
                self.modules[module] = mod

        self.expression = args.expression

        input_topic_in_ns = args.input
        if not input_topic_in_ns.startswith('/'):
            input_topic_in_ns = rospy.get_namespace() + args.input

        input_class, input_topic, self.input_fn = rostopic.get_topic_class(
            input_topic_in_ns, blocking=args.wait_for_start)
        if input_topic is None:
            print('ERROR: Wrong input topic (or topic field): %s' % input_topic_in_ns, file=sys.stderr)
            sys.exit(1)

        self.output_class = roslib.message.get_message_class(args.output_type)

        self.pub = rospy.Publisher(args.output_topic, self.output_class, queue_size=1, latch=args.latch)
        self.sub = rospy.Subscriber(input_topic, input_class, self.callback)

        self.transform_fn = rospy.get_param('~transform_fn', '[0 for x in m.ranges]')
        print(self.transform_fn)
        self.lock = Lock()

    def callback(self, m):
        with self.lock:
            if self.input_fn is not None:
                m = self.input_fn(m)

        try:
            res = eval("{}".format(self.expression), self.modules, {'m': m})
        except NameError as e:
            print("Expression using variables other than 'm': %s" % e.message, file=sys.stderr)
        except UnboundLocalError as e:
            print('Wrong expression:%s' % e.message, file=sys.stderr)
        except Exception:
            raise
        else:
            if not isinstance(res, (list, tuple)):
                res = [res]
            self.pub.publish(*res)


class FakeLaserscanner(TopicOp):

    def __init__(self):
        TopicOp.__init__(self)
        rospy.Service('transform_range', SetBool, self.handle_transform_range)

    def handle_transform_range(self, req):
        func = self.transform_fn if req.data else "m.ranges"
        with self.lock:
            start = self.expression.find("ranges=") + len("ranges=")
            ranges_str = self.expression[start:]
            end = ranges_str.find(",")
            self.expression = self.expression.replace(self.expression[start:start+end], func)
            print(self.expression[start:start+end])
            print(func)
            return SetBoolResponse(True, "Data transformed successfully")

if __name__ == '__main__':
    rospy.init_node('fake_laserscanner', anonymous=True)
    app = FakeLaserscanner()
    rospy.spin()
