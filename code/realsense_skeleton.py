import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

mppose = mp.solutions.pose
def get_landmark(landmarks, part_name):
    try:
      return [
          landmarks[mppose.PoseLandmark[part_name].value].x,
          landmarks[mppose.PoseLandmark[part_name].value].y,
          landmarks[mppose.PoseLandmark[part_name].value].z,
      ]
    except KeyError:
       print("no {} found".format(part_name))

# Pyrealsense
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

align_to = rs.stream.color
align = rs.align(align_to)

pipeline.start(config)


# For static images:
IMAGE_FILES = []
BG_COLOR = (192, 192, 192) # gray
with mp_pose.Pose(
    static_image_mode=True,
    model_complexity=2,
    enable_segmentation=True,
    min_detection_confidence=0.5) as pose:
  for idx, file in enumerate(IMAGE_FILES):
    image = cv2.imread(file)
    image_height, image_width, _ = image.shape
    # Convert the BGR image to RGB before processing.
    results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    if not results.pose_landmarks:
      continue
    print(
        f'Nose coordinates: ('
        f'{results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x * image_width}, '
        f'{results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].y * image_height})'
    )

    annotated_image = image.copy()
    # Draw segmentation on the image.
    # To improve segmentation around boundaries, consider applying a joint
    # bilateral filter to "results.segmentation_mask" with "image".
    condition = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
    bg_image = np.zeros(image.shape, dtype=np.uint8)
    bg_image[:] = BG_COLOR
    annotated_image = np.where(condition, annotated_image, bg_image)
    # Draw pose landmarks on the image.
    mp_drawing.draw_landmarks(
        annotated_image,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
    cv2.imwrite('/tmp/annotated_image' + str(idx) + '.png', annotated_image)
    # Plot pose world landmarks.
    mp_drawing.plot_landmarks(
        results.pose_world_landmarks, mp_pose.POSE_CONNECTIONS)

# numpy data declare
x_shoulder_data = []
y_shoulder_data = []
z_shoulder_data = []
x_wrist_data = []
y_wrist_data = []
z_wrist_data = []


# For webcam input:
#cap = cv2.VideoCapture(0)
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
    """   while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue """
    while True:
      frames = pipeline.wait_for_frames()
      aligned_frames = align.process(frames)

      depth_frame = aligned_frames.get_depth_frame()
      color_frame = frames.get_color_frame()
  
      image = np.asanyarray(color_frame.get_data())
  
      # To improve performance, optionally mark the image as not writeable to
      # pass by reference.
      image.flags.writeable = False
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      results = pose.process(image)

      if not results.pose_landmarks:
        cv2.imshow('MediaPipe Pose', image)
        cv2.waitKey(1)
        continue
      # Draw the pose annotation on the image.
      image.flags.writeable = True
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

      #get Distance 
      #print(depth_frame.get_distance(320,240))

      
      # choose point right shoulder and wrist
      x_coordinate_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * 640
      y_coordinate_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * 480
      x_coordinate_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * 640
      y_coordinate_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * 480

      x_shoulder_data.append(x_coordinate_shoulder)
      y_shoulder_data.append(y_coordinate_shoulder)
      x_wrist_data.append(x_coordinate_wrist)
      y_wrist_data.append(y_coordinate_wrist)

    # take only 100 data and shutdown automatically
      #if(len(x_shoulder_data) == 100):
        #break

      # initialized, in case not detect any body frame
      if not depth_frame.get_distance(int(x_coordinate_shoulder), int(y_coordinate_shoulder)) or not depth_frame.get_distance(int(x_coordinate_wrist), int(y_coordinate_wrist)):
        print("NOT Detected")
        cv2.waitKey(1)
        continue
      else: 
        dist_shoulder = depth_frame.get_distance(int(x_coordinate_shoulder), int(y_coordinate_shoulder)) # meter
        z_shoulder_data.append(dist_shoulder)
        dist_wrist = depth_frame.get_distance(int(x_coordinate_wrist), int(y_coordinate_wrist))
        z_wrist_data.append(dist_wrist)
        # print(x_coordinate_shoulder, y_coordinate_shoulder)
        print(dist_shoulder, dist_wrist)
        print(abs(dist_shoulder - dist_wrist))
        # cv2.circle(image,(x_coodinate, y_coodinate),5,(0,0,255),-1)
        # cv2.circle(image,(320,240),5,(0,0,255),-1)

      mp_drawing.draw_landmarks(
          image,
          results.pose_landmarks,
          mp_pose.POSE_CONNECTIONS,
          landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
      # Flip the image horizontally for a selfie-view display.
      
      cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
      key = cv2.waitKey(5)
      if key == 27: # 27 == key esc
        break

# 填補缺失值
miss = len(y_wrist_data)-len(z_wrist_data)
for i in range(miss):
  z_wrist_data.append(z_wrist_data[len(z_wrist_data)-1])

miss = len(y_shoulder_data)-len(z_shoulder_data)
for i in range(miss):
  z_shoulder_data.append(z_shoulder_data[len(z_shoulder_data)-1])

# 使用線性插值計算缺失值
# known_indices = list(range(len(z_wrist_data)))
# for i in range(len(x_wrist_data)):
#     if i not in known_indices:
#         # 找到相鄰的兩個已知值的索引
#         left_idx = max([idx for idx in known_indices if idx < i])
#         right_idx = min([idx for idx in known_indices if idx > i])
#         
#         # 進行線性插值
#         left_val = z_wrist_data[left_idx]
#         right_val = z_wrist_data[right_idx]
#         z_value = left_val + (right_val - left_val) * (i - left_idx) / (right_idx - left_idx)
#         
#         # 插入補償值
#         z_wrist_data.insert(i, z_value)


# 彌補缺失直 使用平均值
# z_mean = np.mean(z_wrist_data)
# num_missing_values = len(x_wrist_data) - len(z_wrist_data)
# for _ in range(num_missing_values):
#   z_wrist_data.append(z_mean)

# 3D
x_data = [ a - b for a, b in zip(x_wrist_data, x_shoulder_data) ]
y_data = [ a - b for a, b in zip(y_wrist_data, y_shoulder_data) ]
z_data = [ a - b for a, b in zip(z_wrist_data, z_shoulder_data) ]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x_data = [abs(i) for i in x_data]
y_data = [abs(i) for i in y_data]
z_data = [abs(i) for i in z_data] 
ax.scatter(x_data, y_data, z_data)
ax.set_xlabel("X (pixel)")
ax.set_ylabel("Y (pixel)")
ax.set_zlabel("Z (meter)")
plt.show() 
print(len(x_wrist_data))

# 2D
# plt.scatter(x_shoulder_data, y_shoulder_data, color='blue', label='shoulder')
# plt.scatter(x_wrist_data, y_wrist_data, color='green', label='wrist')
# plt.xlabel("X (pixel)")
# plt.ylabel("Y (pixel)")
# plt.xlim(0, 640)
# plt.ylim(0, 480)
# plt.legend()
# plt.show()