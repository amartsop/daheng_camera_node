import rospy
from std_msgs.msg import Bool

class RecordingStateListener:

    def __init__(self):
        # Name of node
        self.node_name = "recording_state_listener"

        # Name of the topic to be subscribed to
        self.subscribed_topic_name = "camera_recording_state"

        # Initialize recording status
        self.recording_on = False

        rospy.init_node(self.node_name, anonymous=True)

        # Subscribe to recording state
        rospy.Subscriber(self.subscribed_topic_name, Bool,
            self.recording_status_callback)

    def get_recording_state(self):
        return self.recording_on

    def recording_status_callback(self, res):
        self.recording_on = res.data

