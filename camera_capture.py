import pyzed.sl as sl
import cv2
import numpy as np


def take_photo():

   zed = sl.Camera()
   init_params = sl.InitParameters()
   init_params.camera_resolution = sl.RESOLUTION.AUTO 
   init_params.camera_fps = 15
   init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT
   err = zed.open(init_params)
   if (err > sl.ERROR_CODE.SUCCESS) :
      exit(-1)

   i = 0
   image = sl.Mat()
   runtime_parameters = sl.RuntimeParameters()
   lastime = 0
   while i < 10000:
       if zed.grab(runtime_parameters) <= sl.ERROR_CODE.SUCCESS:
           zed.retrieve_image(image, sl.VIEW.LEFT)
           timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
           i = i + 1
           
           curTime = int(timestamp.get_milliseconds())
           print(curTime - lastime)
           lastime = curTime
           

   zed.close()


take_photo()
