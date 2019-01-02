import uuid
from enum import Enum

import rospy


class Topic(Enum):
    sub = 1
    pub = 2


class Node:

    def __init__(self):
        self.topics = []
        self.topic_subs = {}

    def topic(self, mode, topic_g, msg_type, rate=10):
        def cb_decorator(callback, *args):
            try:
                from_part, imp_part = msg_type.rsplit('.', 1)
                mod = __import__(from_part, fromlist=[imp_part])
                imported_dtype = getattr(mod, imp_part)
            except ImportError:
                print('Cant import given dtype!')
                raise

            if mode is Topic.sub:
                self.topic_subs[topic_g] = (topic_g, imported_dtype, callback)

            def wrapper(*args, **kwargs):
                cb_ret = callback(*args, **kwargs)
                if mode is Topic.pub:
                    pub = rospy.Publisher(topic_g, imported_dtype, queue_size=10)
                    pub.publish(cb_ret)
            self.topics.append((mode, msg_type, rate, wrapper, topic_g))
            return wrapper    
        return cb_decorator    
    
    def run(self, node_name=None):
        if not node_name:  # needed to avoid variable referenced b4 assignment error
            node_name_init = 'node_' +  uuid.uuid4().hex.upper()[0:6]
        else:
            node_name_init = node_name

        print('-'*80)
        print('created node: {}'.format(node_name_init))
        rospy.init_node(node_name_init, anonymous=True)

        for topic_ in self.topics:
            if topic_[0] is Topic.sub:
                sub_args = self.topic_subs[topic_[4]]
                rospy.Subscriber(sub_args[0], sub_args[1], sub_args[2])
                print('subscribed to: {}'.format(sub_args[0]))

        ros_rate = rospy.Rate(self.main_runner[1])
        while not rospy.is_shutdown():
            self.main_runner[0]()
            ros_rate.sleep()

        self.main_runner()
        
    def main(self, rate=10):
        def wrapper(callback):
            self.main_runner = callback, rate
        return wrapper

