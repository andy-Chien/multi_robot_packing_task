#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from task_msgs.srv import GQCNNGraspPlanner, GQCNNGraspPlannerRequest
from cv_bridge import CvBridge, CvBridgeError

color_img = None
depth_img = None
camera_inf = None
mouse_x = 0
mouse_y = 0
px_x = 0
px_y = 0
minx = 542
miny = 264
maxx = 938
maxy = 632

def color_img_cb(data):
    global color_img
    color_img = data

def grasp_plan(req):
    rospy.wait_for_service('/gqcnn/grasp_planner')
    try:
        gqcnn_client = rospy.ServiceProxy('/gqcnn/grasp_planner', GQCNNGraspPlanner)
        res = gqcnn_client(req)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def on_mouse(event, x, y, flag, param):
    global mouse_x, mouse_y
    mouse_x = x
    mouse_y = y
    
if __name__ == '__main__':
    rospy.init_node('img_adapter', anonymous=True)
    rospy.Subscriber("/rgb/image_raw", Image, color_img_cb)

    cv_brige = CvBridge()
    req = GQCNNGraspPlannerRequest()
    loop_rate = rospy.Rate(1)
    cv2.namedWindow("Image window 1", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("Image window 1", on_mouse, 0)

    while not rospy.is_shutdown():
        loop_rate.sleep()
        res = grasp_plan(req)
        if res is None:
            continue
        print(res.grasp.pose)
        print('q_value = {}'.format(res.grasp.q_value))
        print(res.grasp.angle)
        (px_x, px_y) = (int(res.grasp.center_px[0]), int(res.grasp.center_px[1]))
        img = cv_brige.imgmsg_to_cv2(color_img, "bgr8")
        img = cv2.circle(img, (int(res.grasp.center_px[0]), int(res.grasp.center_px[1])), 4, (255, 255, 255), 2)
        cv2.putText(img, str(mouse_x) + ', ' + str(mouse_y), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow("Image window 1", img)
        cv2.imshow("Image window 2", cv_brige.imgmsg_to_cv2(res.grasp.thumbnail, "32FC1"))
        cv2.waitKey(1)
    cv2.destroyAllWindows()

    # 542, 264, 938, 632

