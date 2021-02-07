#!/usr/bin/env python
import rospy
from recording_state_listener import RecordingStateListener
from daheng_camera import DahengCamera


def camera_control_main():

    # Recording state handle
    recording_state = RecordingStateListener()

    # Camera 1 handle
    camera1 = DahengCamera()

    while(not rospy.is_shutdown()):
        camera1.raw_img_acquisition(recording_state.get_recording_state())

    print(camera1.get_total_frames())

    rospy.spin()


if __name__ == '__main__':
    try:
        camera_control_main()
    except rospy.ROSInterruptException:
        pass

