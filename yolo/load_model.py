import os
import sys
#sys.path.append('../../detection/yolov5')
import time
import argparse
import csv
from pathlib import Path
import glob
import cv2
import torch
import torch.backends.cudnn as cudnn

from ultralytics import YOLO
#sys.path.append('models')
#sys.path.append('weights')

#import csv
#from models.common import DetectMultiBackend
#from models import yolo
#from detection.yolov5.utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
#from utils_yolov5.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
#from detection.yolov5.utils.plots import Annotator, colors, save_one_box
#from detection.yolov5.utils.torch_utils import select_device, time_sync


class LoadYolov8nModel(object):
    def __init__(self):
        pass

    def load_model(self):
        #load model
        model = YOLO('/root/scripts/yolo/best.pt')

        #weights = ['/home/dream-nano6/badri/badri_ccgrid/ocularone/ccgridPythonFiles/cv_tasks/detection/yolov5/models/yolov5s.pt']
        #model = DetectMultiBackend(weights,device,False)
        #model = yolo.Modelv5().to(device)
        #ckpt = torch.load(weights,map_location=device)
        #model.append(ckpt['ema' if ckpt.get('ema') else 'model'].float().fuse().eval())  # FP32 model

        #stride = model.stride
        #imgsz = check_img_size(imgsz,s=stride)
        return model

#Loadv5Model().load_model()




"""
    #do some dummy inferencing for GPU to warmup
    #print("doing some dummy inferencing for batch size 1..........")
    dummy_data = torch.zeros(1,3,640,640).to(device)
    #starter,ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
    for i in range(10):
        t1 = time_sync() 
        _ = model(dummy_data,False,False)
        t2 = time_sync()
        #inference_time, fps = t2-t1, 1/(t2-t1)

    chunk_duration = opt.chunk_dur
    chunks = glob.glob(source)
    chunks = chunks[:100]
    batches = [1,2,3,4,5]
    batches = [i*chunk_duration for i in batches]

    for chunk in chunks:
        for batch in batches:
            cap = cv2.VideoCapture(chunk)
            dummy_data = torch.zeros(batch,3,640,640).to(device)
            #frame_start,frame_end,e2e_start,e2e_end = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True),\
             #       torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)

            while True:
                read_start = time.time()
                ret, frame = cap.read()
                read_end = time.time()

                if ret:
                    start_time = time_sync()
                    outs = model(dummy_data,augment=False)[0]
                    end_time = time_sync()
                    inf_time = end_time-start_time

                    #post processing
                    #pred = non_max_suppression(outs,0.75,0.45,None,False,1)

                    inf_dict = {'inference_time': inf_time, 'batch_size':batch, \
                                    'chunk_size':chunk_duration,'read_time':read_end-read_start}
                    print(inf_dict)
                    with open(output_path, 'a', newline='') as csv_file:
                        dict_writer = csv.DictWriter(csv_file, inf_dict.keys())
                        dict_writer.writerow(inf_dict)
                else:
                    break

            cap.release()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='weights/best_overall.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.75, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', type=str, help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--cfg', type=str, default='', help='*.cfg path')
    parser.add_argument('--names', type=str, default='data/coco.names', help='*.cfg path')
    parser.add_argument('--chunk-dur', type=int, default=1, help='time duration of each chunk')
    opt = parser.parse_args()

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['']:
                strip_optimizer(opt.weights)
        else:
            detect(opt)
"""
