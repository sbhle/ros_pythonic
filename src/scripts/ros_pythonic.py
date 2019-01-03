import uuid
import rospy
import functools


class Node:

    def __init__(self):
        self.topic_subs = []

    @staticmethod
    def _import_from_str(import_str):
        try:
            from_part, imp_part = import_str.rsplit('.', 1)
            mod = __import__(from_part, fromlist=[imp_part])
            return getattr(mod, imp_part)
        except ImportError:
            print('Cant import given dtype!')
            raise

    def topic_sub(self, sub_topic, msg_type):
        def wrapper(callback, *args):
            imported_dtype = self._import_from_str(msg_type)
            self.topic_subs.append((sub_topic,
                                    imported_dtype,
                                    callback))
        return wrapper

    def topic_pub(self, pub_topic, msg_type, rate=10, queue_size=10):
        # two decorators to enable arg/no arg for pub
        def cb_decorator(callback, *args):
            imported_dtype = self._import_from_str(msg_type)
            @functools.wraps(callback)
            def wrapper(*args, **kwargs):
                cb_ret = callback(*args, **kwargs)
                pub = rospy.Publisher(pub_topic,
                                      imported_dtype,
                                      queue_size=queue_size)
                pub.publish(cb_ret)
            return wrapper    
        return cb_decorator    

    def main(self, rate=10):
        def wrapper(callback):
            self.main_runner = callback, rate
        return wrapper

    def run(self, node_name=None):
        if not node_name:  
            node_name_init = 'node_' +  uuid.uuid4().hex.upper()[0:6]
        else:
            node_name_init = node_name

        print('-'*80)
        print('created node: {}'.format(node_name_init))
        rospy.init_node(node_name_init, anonymous=True)

        for topic, dtype, cb in self.topic_subs:
            rospy.Subscriber(topic, dtype, cb)
            print('subscribed to: {}'.format(topic))

        try:
            # only check main perodically if there is something to do
            ros_rate = rospy.Rate(self.main_runner[1])
            while not rospy.is_shutdown():
                self.main_runner[0]()
                ros_rate.sleep()
        except AttributeError:    
            if self.topic_subs:
                # node only listens on topics - nothing else to do
                rospy.spin()

