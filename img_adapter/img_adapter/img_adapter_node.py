#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from img_adapter.msg import BoundingBox
from img_adapter.srv import GQCNNGraspPlannerBoundingBox, GQCNNGraspPlannerBoundingBoxRequest
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

def depth_img_cb(data):
    global depth_img
    depth_img = data

def camera_inf_cb(data):
    global camera_inf
    camera_inf = data

def grasp_plan(req):
    rospy.wait_for_service('/gqcnn/grasp_planner_bounding_box')
    try:
        gqcnn_client = rospy.ServiceProxy('/gqcnn/grasp_planner_bounding_box', GQCNNGraspPlannerBoundingBox)
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
    rospy.Subscriber("/depth_to_rgb/image_raw", Image, depth_img_cb)
    rospy.Subscriber("/rgb/camera_info", CameraInfo, camera_inf_cb)

    cv_brige = CvBridge()
    req = GQCNNGraspPlannerBoundingBoxRequest()
    loop_rate = rospy.Rate(1)
    cv2.namedWindow("Image window 1", cv2.WINDOW_AUTOSIZE)

    while color_img is None or depth_img is None or camera_inf is None and not rospy.is_shutdown():
        print('some body still are None')
        loop_rate.sleep()
        
    while not rospy.is_shutdown():
        try:
            depth_img_mm = cv_brige.imgmsg_to_cv2(depth_img, "32FC1")
            req.depth_image = cv_brige.cv2_to_imgmsg(depth_img_mm * 0.001, "32FC1")
        except CvBridgeError as e:
            req.depth_image = Image()
            print(e)
        req.color_image = color_img
        req.camera_info = camera_inf
        req.bounding_box = BoundingBox(minx, miny, maxx, maxy)
        # req.camera_info.roi.x_offset = 740
        # req.camera_info.roi.y_offset = 448
        # req.camera_info.roi.height = 368
        # req.camera_info.roi.width = 396
        # req.camera_info.roi.do_rectify = True
                
        res = grasp_plan(req)
        print(res.grasp.pose)
        print('q_value = {}'.format(res.grasp.q_value))
        print(res.grasp.angle)
        (px_x, px_y) = (int(res.grasp.center_px[0]), int(res.grasp.center_px[1]))
        cv2.setMouseCallback("Image window 1", on_mouse, 0)
        depth_img_mm *= 0.001
        depth_img_mm = cv2.circle(depth_img_mm, (int(res.grasp.center_px[0]), int(res.grasp.center_px[1])), 4, (255, 255, 255), 2)
        cv2.rectangle(depth_img_mm, (minx, miny), (maxx, maxy), (0, 255, 0), 1)
        cv2.putText(depth_img_mm, str(mouse_x) + ', ' + str(mouse_y), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow("Image window 1", depth_img_mm)
        cv2.imshow("Image window 2", cv_brige.imgmsg_to_cv2(res.grasp.thumbnail, "32FC1"))
        cv2.waitKey(1)
        loop_rate.sleep()
    cv2.destroyAllWindows()

    # 542, 264, 938, 632

