# In settings.json first activate computer vision mode:
# https://github.com/Microsoft/AirSim/blob/main/docs/image_apis.md#computer-vision-mode

import airsim
import cv2
import time, sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cameraTypeMap = {
 "depth": airsim.ImageType.DepthVis,
 "segmentation": airsim.ImageType.Segmentation,
 "seg": airsim.ImageType.Segmentation,
 "scene": airsim.ImageType.Scene,
 "disparity": airsim.ImageType.DisparityNormalized,
 "normals": airsim.ImageType.SurfaceNormals
}

if (cameraType not in cameraTypeMap):
  printUsage()
  sys.exit(0)

print (cameraTypeMap[cameraType])

client = airsim.MultirotorClient(ip="172.23.80.1")

print("Connected: now while this script is running, you can open another")
print("console and run a script that flies the drone and this script will")
print("show the depth view while the drone is flying.")

help = False

fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
thickness = 2
textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)
print(textSize)
textOrg = (10, 10 + textSize[1])
frameCount = 0
startTime = time.time()
fps = 0

# print("Taking off...")
# client.armDisarm(True)
# client.takeoffAsync().join()

while True:
    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    rawImages = client.simGetImages([
       airsim.ImageRequest("high_res", airsim.ImageType.Scene, False, False),
    ])
    rawImage = rawImages[0]
    img1d = np.frombuffer(rawImage.image_data_uint8, dtype=np.uint8) 
    img_rgb = img1d.reshape(rawImage.height, rawImage.width, 3)
    img_rgb = np.copy(img_rgb)
    img_rgb = cv2.resize(img_rgb, (640, 480))
    if (rawImage == None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    else:
        cv2.putText(img_rgb, 'FPS ' + str(fps), textOrg, fontFace, fontScale, (255, 0, 255), thickness)
        cv2.imshow("Scene", img_rgb)

    frameCount = frameCount  + 1
    endTime = time.time()
    diff = endTime - startTime
    if (diff > 1):
        fps = frameCount
        frameCount = 0
        startTime = endTime

    key = cv2.waitKey(1) & 0xFF
    if (key == 27 or key == ord('q') or key == ord('x')):
        break
