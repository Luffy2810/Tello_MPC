import os
import sys
import time
import csv
from pathlib import Path
import glob
import cv2
import torch
import torch.backends.cudnn as cudnn

import csv
# sys.path.append('.') ## add current directory path to python path
# from load_model import LoadYolov8nModel
from detection.yolov8n.load_model import LoadYolov8nModel


class DetectYolov8n(object):
    def __init__(self):
        # self.model = Yolov8nModel.load_model()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

    def detect(self, model, batch_frame, dnn_model):    
        bboxes_cls_list = []
        b_xywh_list = []
        print ("shape: ",len(batch_frame))
        print("------ Running detect.py on ------")
        print(self.device)
        print("------------------------")
        # max_value, _ = torch.max(batch_frame[0], dim=0)
        # print (batch_frame[0])
        output = model.predict(batch_frame[0], conf = 0.5)
        
        bboxes = output[0].boxes
        print (output[0].boxes.conf)
        for box in bboxes:
            b_xywh_list.append(box.xywh.squeeze().tolist())
            bboxes_cls_list.append([box.cls.squeeze().tolist(), box.xyxy.squeeze().tolist()])

        if dnn_model == "DISTANCE_ESTIMATION_VIP":
            result = 0.0
            print ("b_xywh list = ", b_xywh_list)
            print("bboxes_cls_list = ", bboxes_cls_list)
            for item in b_xywh_list:
                bbox_w = item[2]
                bbox_h = item[3]
                bbox_area = bbox_w * bbox_h
                result = bbox_w * -2.144 + bbox_h * -1.767 + bbox_area * 0.0050
            return result
        result = bboxes_cls_list
        return result



        #class_id = output[0].class_id
        #b = torch.tensor([0,0,0,0])
        #b_y = torch.tensor([0,0,0,0])
        #for box in boxes:
        #    b = box.xywh[0]
        #    b_y = box.xyxy[0]


        #if dnn_model == "":

#yolov5_model = Loadv5Model().load_model()
#detect = Detectv5(yolov5_model)
#for i in range(10):
    #detect.detect([1])




