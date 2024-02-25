import rospy
from sensor_msgs.msg import Image
from usb_cam.srv import *

def check_camera_connection():
    rospy.init_node('camera_connection_checker', anonymous=True)
    
    # USB 카메라 연결 상태를 확인하기 위한 서비스 호출
    rospy.wait_for_service('usb_cam/check_connection')
    try:
        check_connection = rospy.ServiceProxy('usb_cam/check_connection', Empty)
        check_connection()
        rospy.loginfo("USB 카메라가 연결되어 있습니다.")
    except rospy.ServiceException as e:
        rospy.loginfo("USB 카메라 연결 상태를 확인할 수 없습니다. 오류 메시지: %s" % e)

if __name__ == '__main__':
    check_camera_connection()
