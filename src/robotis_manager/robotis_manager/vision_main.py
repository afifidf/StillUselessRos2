from robotis_manager.vision_image import Vision
from robotis_manager.vision_utils import *
# from yolov3 import Yolov3

import cv2
import os
from ament_index_python.packages import get_package_share_directory

# import concurrent.futures

# executor = concurrent.futures.ThreadPoolExecutor()

data_dir = os.path.join(get_package_share_directory('robotis_manager'), 'data')
os.chdir(data_dir)
cam = Vision()
# cam.writeConfig("bismillah.avi")

colorbase = ColorBased()

# field
# colorbase.createTrackbar("Field", "FLD")
val_green_field = colorbase.load("robotis_green_field")
# val_green_field = [[11, 36, 24], [170, 146, 193]]

# ball
# colorbase.createTrackbar("Ball", "HSV")
val_ball_orange = colorbase.load("robotis_orange_ball")
# val_ball_orange = [[11, 47, 19], [28, 172, 132]]
colorbase.blobSetParams()

# goal
# colorbase.createTrackbar("Goal", "GOL")
val_goal_pos = colorbase.load("robotis_goal_pos")
# val_goal_pos = [[0, 0, 0], [119, 109, 160]]

# global variables
# ball_real_pos = {}
# goal_real_pos = {}

# Global Vision
configurator = load_json_configurator("robotis_main_vision_configurator.json")

# Access the specific parameters
hdrMode = configurator.get("hdrMode", False)  # default to False if the key is not found

hdr_processor1 = None

if hdrMode:
    hdrOptions = configurator.get("hdrOptions", {})

    gamma = hdrOptions.get("Gamma", 1.0)  # default to 1.0 if the key is not found
    saturation = hdrOptions.get("Saturation", 0.7)  # default to 0.7 if the key is not found
    hdr_processor1 = HDRProcessor(gamma, saturation)


def BallDetectorNode(data, rh):
    # global ball_real_pos
    global val_green_field, val_ball_orange, hdr_processor1
    frame = rh.imgMsg2Frame(data)

    if hdr_processor1 is not None:
        resize1 = cam.resize(frame, 450)
        hdr_image1 = hdr_processor1.enhance_image2(resize1)
        frame = cam.blur(hdr_image1)
    else :
        frame = cam.blur(cam.resize(frame, 450))

    # bismillah
    # _, frame = cap.read()
    hsv = colorbase.color2(frame, TO_HSV)
    # cam.show(colorbase.color2(cam.resize(frame, 450), TO_HSV), "hsv asli", show_fps=False)


    try:
        # load param -> for calibration only
        val_green_field = colorbase.load("robotis_green_field")
        val_ball_orange = colorbase.load("robotis_orange_ball")

        # field_mask = colorbase.calibrate(colorbase.color2(frame, TO_YUV), "robotis_green_field")
        field_mask = colorbase.mask(colorbase.color2(frame, TO_YUV), val_green_field)
        field_cnts = colorbase.getContours(field_mask, RET_TREE)
        field_mask = colorbase.fill(frame, field_cnts)

        # ros image publish
        # ros_field_mask = rh.bridge.cv2_to_imgmsg(field_mask, encoding="mono8")
        # rh.maskingFieldFrame.publish(ros_field_mask)

        ball_frame = colorbase.And(frame, frame, mask=field_mask)
        # ball_mask = colorbase.calibrate(colorbase.color2(ball_frame, TO_HSV), "robotis_orange_ball")
        ball_mask = colorbase.mask(colorbase.color2(ball_frame, TO_HSV), val_ball_orange)
        ball_mask = colorbase.morph(ball_mask, MORPH_CLOSE, colorbase.getElement(ELE_ELLIPSE, 2))
        ball_blob_pos = colorbase.blobPos(colorbase.blob(~ball_mask))
        ball_color_pos = colorbase.circlePos(colorbase.getContours(ball_mask))
        ball_real_pos = ball_blob_pos if ball_blob_pos != {} else ball_color_pos

        colorbase.drawPoints(frame, ball_real_pos, shape=DRAW_CIRCLE, label="ball", disp_coordinates=True)
        # colorbase.drawPoints(frame, goal_real_pos, shape=DRAW_RECT, label="goal", disp_coordinates=True)
        # colorbase.enableReferenceLine(frame)

        rh.ball2ros(ball_real_pos, frame.shape)

        cam.show(ball_frame, "ball_frame")
        cam.show(hsv, "hsv")
        cam.show(field_mask, "field_mask")
        cam.show(ball_mask, "ball_mask")
        cam.show(frame, "frame_ball", show_fps=False)
        # cam.show(frame1, "frame_ball HDR 1", show_fps=False)
        # cam.show(frame2, "frame_ball HDR 2", show_fps=False)
        # cam.write(frame)
        cam.wait(1)

    except Exception as e:
        print(f"[BallDetectorNode] Exception: {e}")


def GoalPosDetector(data, rh):
    global val_goal_pos
    frame = rh.imgMsg2Frame(data)
    # frame = rh.compressedImgMsg2Frame(data)
    frame = cam.blur(cam.resize(frame, 450))
    hsv = colorbase.color2(frame, TO_HSV)
    try:
        val_green_field = colorbase.load("robotis_green_field")
        field_mask = colorbase.mask(colorbase.color2(frame, TO_YUV), val_green_field)
        field_cnts = colorbase.getContours(field_mask, RET_TREE)
        field_mask = colorbase.fill(frame, field_cnts)

        goal_frame = colorbase.And(frame, frame, mask=~field_mask)

        # load param -> for calibration only
        # val_green_field = colorbase.load("robotis_green_field")
        # val_ball_orange = colorbase.load("robotis_orange_ball")
        # val_goal_pos = colorbase.load("robotis_goal_pos")

        goal_mask = colorbase.mask(colorbase.color2(goal_frame, TO_HSV), val_goal_pos)
        goal_real_pos = colorbase.rectPos(colorbase.getContours(~goal_mask))

        colorbase.drawPoints(frame, goal_real_pos, shape=DRAW_RECT, label="goal", disp_coordinates=True)

        # cam.show(goal_frame, "goal_frame")
        # cam.show(~goal_mask, "goal_mask")
        # cam.show(hsv, "hsv")
        # cam.show(frame, "frame_goal", show_fps=True)
        cam.wait(1)
    except Exception as e:
        pass

# TODO :: to run the code, run the node_handler
