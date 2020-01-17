import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import tf2_ros
import yaml
print(cv2.__version__)

cal_data = yaml.load(open('./boards/board.yaml', 'r'), Loader=yaml.Loader)
board = cal_data['board']
print board.getSquareLength()

imboard = board.draw((1000, 1400))
cv2.imwrite('charuco.jpg', imboard)

cal_data = yaml.load(open('./calib_data/cam.yaml', 'r'), Loader=yaml.Loader)
mtx = np.asarray(cal_data['camera_matrix'])
dist = np.asarray(cal_data['dist_coefficients'])
p_rvec = None
p_tvec = None

cap = cv2.VideoCapture(0)
decimator = 0

bridge = CvBridge()
tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

raw_image_publisher = rospy.Publisher('camera/raw_image', Image, queue_size=0)
detected_marker_image_publisher = rospy.Publisher('camera/detected_image', Image, queue_size=0)
marker_pose_publisher = rospy.Publisher('charuco/marker_pose', Pose, queue_size=0)
robot_pose_publisher = rospy.Publisher('charuco/rover_pose', Pose, queue_size=0)
marker_detected_publisher = rospy.Publisher('charuco/marker_deteced', Bool, queue_size=0)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters =  cv2.aruco.DetectorParameters_create()

    marker_detected = Bool()
    marker_detected.data = False

    res = cv2.aruco.detectMarkers(gray, board.dictionary, parameters=parameters)
    cv2.aruco.refineDetectedMarkers(gray, board, res[0], res[1], res[2])

    if len(res[0])>0:
        for r in res[0]:
            cv2.cornerSubPix(
                gray, 
                r,
                winSize = (3,3),
                zeroZone = (-1,-1),
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
            )
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)

        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            cv2.aruco.drawDetectedCornersCharuco(gray,res2[1],res2[2])
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(res2[1], res2[2], board, mtx, dist, rvec=p_rvec, tvec=p_tvec, useExtrinsicGuess=False)
            if retval:
                print 'rvec:', rvec
                print 'tvec:', tvec
                p_rvec = rvec
                p_tvec = tvec

                # Compute pose of Camera relative to world
                dst, _ = cv2.Rodrigues(rvec)
                R = dst.T
                tvec = np.dot(-R, tvec)
                rvec, _ = cv2.Rodrigues(R)

                marker_detected.data = True
                marker_pose = Pose()
                quat = tf.transformations.quaternion_from_euler(rvec[1, 0], rvec[0, 0], -rvec[2, 0])
                marker_pose.position.x = tvec[2, 0]
                marker_pose.position.y = tvec[0, 0]
                marker_pose.position.z = tvec[1, 0]
                marker_pose.orientation.x = quat[0]
                marker_pose.orientation.y = quat[1]
                marker_pose.orientation.z = quat[2]
                marker_pose.orientation.w = quat[3]
                marker_pose_publisher.publish(marker_pose)
                robot_pose = Pose()
                trans = tfBuffer.lookup_transform(robot_frame, world_frame, rospy.Time(0))
                robot_pose.position.x = trans.translation.x
                robot_pose.position.y = trans.translation.y
                robot_pose.position.z = trans.translation.z
                robot_pose.orientation.x = trans.rotation.x
                robot_pose.orientation.y = trans.rotation.y
                robot_pose.orientation.z = trans.rotation.z
                robot_pose.orientation.w = trans.rotation.w
                robot_pose_publisher.publish(robot_pose)
                detected_marker_image_publisher.publish(bridge.cv2_to_imgmsg(frame, 'rgb8'))

    cv2.imshow('frame', gray)
    decimator+=1

    marker_detected_publisher.publish(marker_detected)
    raw_image_publisher.publish(bridge.cv2_to_imgmsg(frame, 'rgb8'))

cap.release()
cv2.destroyAllWindows()
