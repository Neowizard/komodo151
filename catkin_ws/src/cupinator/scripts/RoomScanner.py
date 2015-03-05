#!/usr/bin/env python
import rospy

import os
import cv2
import cv_bridge
from cupinator.srv import SingleCupRecQuery
from cupinator.msg import Contour
from cupinator.msg import SnapshotData
from cupinator.msg import RoomScannerCommand


class RoomScanner:

    def __init__(self):
        rospy.Subscriber("cupinator/room_scanner/command", RoomScannerCommand, self.scanner_cb)
        pass

    def scanner_cb(self, scanner_command):
        """
        Dispatcher of the few RoomScannerCommand types to their respective handlers.
        :param scanner_command: RoomScannerCommand message to dispatch
        :type scanner_command: RoomScannerCommand
        """
        if (scanner_command.command != "start"):
            rospy.logdebug("Command %s not implemented, ignoring message" % scanner_command.command)
        if (scanner_command.full):
            self.room_scan()
        else:
            self.stationary_scan()

    def capture_image(self):
        #TODO: implement real camera capture
        DB_path = os.path.join(os.path.dirname(__file__), "cupRec/DB/")
        captured_image = cv2.imread(DB_path+"green1.jpg")
        msg_image = cv_bridge.CvBridge().cv2_to_imgmsg(captured_image)
        return msg_image

    def room_scan(self):
        pass

    def stationary_scan(self):
        rospy.wait_for_service("/cupinator/cup_recognizer/single_cupRec", 10)  # Waite up to 10 seconds for service
        try:
            cupRec_srv = rospy.ServiceProxy("cupinator/cup_recognizer/single_cupRec", SingleCupRecQuery)
            message_img = self.capture_image()
            best_match_contour = cupRec_srv(message_img)
            print best_match_contour.x_coords[0]
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed. Exception: %s" % e)



if __name__ == '__main__':
    try:
        rospy.init_node("RoomScanner", anonymous=False, log_level=rospy.DEBUG)
        roomScanner = RoomScanner()
        rospy.spin()

    except rospy.ROSInterruptException, e:
        pass
