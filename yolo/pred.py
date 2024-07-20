from load_model import LoadYolov8nModel

l = LoadYolov8nModel()
model = l.load_model()
output = model.predict(batch_frame[0], conf = 0.5)
        
bboxes = output[0].boxes
print (output[0].boxes.conf)
for box in bboxes:
    b_xywh_list.append(box.xywh.squeeze().tolist())
    bboxes_cls_list.append([box.cls.squeeze().tolist(), box.xyxy.squeeze().tolist()])
