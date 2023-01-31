import os
import glob
import cv2
import numpy as np
import torch
import time
import math
import yolo_net

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def get_ypre(ypre,input_size,anchors):
    anchor_gird = np.reshape(anchors, (1, 1, 1, 1, 2))
    mesh_x = np.reshape(np.tile(np.arange(0, ypre.shape[2]), [ypre.shape[1]]),
                           (1, ypre.shape[1], ypre.shape[2], 1, 1))
    mesh_y = np.reshape(np.tile(np.arange(0, ypre.shape[1]), [ypre.shape[2]]),
                           (1, ypre.shape[2], ypre.shape[1], 1, 1))
    mesh_y = np.transpose(mesh_y, (0,2,1,3,4))
    mesh_xy = np.concatenate([mesh_x, mesh_y], -1)

    pre_con = sigmoid(ypre[..., 4])
    pre_cla = sigmoid(ypre[..., 5:])
    pre_xy = (sigmoid(ypre[..., :2]) + mesh_xy) * 64#64倍下采样
    pre_wh_half = np.exp(ypre[..., 2:4]) * anchor_gird / 2
    pre_mins = pre_xy - pre_wh_half
    pre_maxes = pre_xy + pre_wh_half

    return np.concatenate([pre_mins, pre_maxes, np.expand_dims(pre_con, axis=-1), pre_cla],-1)

def get_IOU(box1,box2):
    intersect_mins  =np.maximum(box1[:2],  box2[:2])
    intersect_maxes =np.minimum(box1[2:4],  box2[2:4])

    intersect_wh    = np.maximum(intersect_maxes - intersect_mins, 0.)
    intersect_areas = intersect_wh[..., 0] * intersect_wh[..., 1]

    box1_areas = (box1[2]-box1[0]) * (box1[3]-box1[1])
    box2_areas = (box2[2]-box2[0]) * (box2[3]-box2[1])

    union_areas = box1_areas + box2_areas - intersect_areas
    IOU  = intersect_areas/union_areas

    return IOU

def nms_boxes(box,iou):
    boxes = sorted(box.tolist(),key=(lambda x:x[5]),reverse=True)
    new_boxes = []
    l = 0
    while len(boxes)>0:
        new_boxes.append(boxes[0])
        del boxes[0]
        i = 0
        for _ in range(len(boxes)):
            IOU = get_IOU(np.array(new_boxes[l][:4]), np.array(boxes[i][:4]))
            if IOU>iou:
                del boxes[i]
                i -= 1
            i += 1
        l += 1
    return new_boxes

def draw_result(boxes,img,input_size):
    for box in boxes:
        xmin = max(int(box[0] * img.shape[1] / input_size[1]),0)
        ymin = max(int(box[1] * img.shape[0] / input_size[0]),0)
        xmax = min(int(box[2] * img.shape[1] / input_size[1]),img.shape[1])
        ymax = min(int(box[3] * img.shape[0] / input_size[0]),img.shape[0])

        img[ymin:ymax, xmin] = (0,255,0)
        img[ymin:ymax, xmax] = (0,255,0)
        img[ymin, xmin:xmax] = (0,255,0)
        img[ymax, xmin:xmax] = (0,255,0)

    # cv2.imshow('',cv2.resize(img,(800,500)))
    result_path = os.getcwd() + '/det-{:.10}'.format(time.time() / 1e9)
    if not os.path.exists(result_path):
        os.makedirs(result_path)
    res_dir = result_path + '/result.png'
    cv2.imwrite(res_dir, img)
    # cv2.waitKey(0)
    # reshape_time1 = time.time()
    # print('reshape{:.2f}ms'.format(1000*(reshape_time1-reshape_time0)))
    return res_dir

def get_img(fname):
    try:

        image = cv2.cvtColor(fname,cv2.COLOR_BGR2GRAY)
        input_width = math.ceil(image.shape[1] / 128) * 128
        input_height = math.ceil(image.shape[0] / 128) * 128
        img = cv2.resize(image,(input_width,input_height))

        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image,np.expand_dims(img,axis=0)
    except:
        return [],[]

class detect():
    def __init__(self,model_path):
        self.iou = 0.3
        self.score = 0.8
        self.classesname = ['switch']
        self.anchors = np.array([[150, 150]])
        self.model_path = model_path
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = torch.load(self.model_path,map_location=self.device).eval()

    def start(self,img):
        start_time0 = time.time()
        image,img = get_img(img)
        if len(image) == 0:
            print('wrong img dir')
        else:
            input = torch.autograd.Variable(torch.from_numpy(img),requires_grad=False).to(self.device)
            end_time0 = time.time()

            output = self.model(torch.unsqueeze(input/255.,0)).to('cpu').detach().numpy()
            end_time1 = time.time()

            boxes= []
            ypre = np.squeeze(np.transpose(np.expand_dims(output,-1), (0,4,2,3,1)),0)
            ypre = np.reshape(ypre, (-1, ypre.shape[1], ypre.shape[2], 1,ypre.shape[3]))
            new_ypre = np.squeeze(get_ypre(ypre,img.shape,self.anchors),0)

            for n in range(int(new_ypre.shape[1])):
                for m in range(int(new_ypre.shape[0])):
                    for i in range(int(new_ypre.shape[2])):
                        object_prob = new_ypre[m,n,i,4]
                        if object_prob > self.score:
                            classes = (new_ypre[m,n,i,5:]*object_prob).tolist()
                            # index = classes.index(max(classes))
                            new_ypre[m,n,i,5:] = max(classes)
                            boxes.append(new_ypre[m,n,i,:6])

            boxes = nms_boxes(np.array(boxes),self.iou)
            end_time2 = time.time()

            res_dir = draw_result(boxes,image,img.shape[1:])
            end_time3 = time.time()

            # print('前处理{:.2f}ms'.format(1000*(end_time0-start_time0)),
            #       '推理{:.2f}ms'.format(1000*(end_time1-end_time0)),
            #       '后处理{:.2f}ms'.format(1000*(end_time2-end_time1)),
            #       '保存{:.2f}ms'.format(1000*(end_time3-end_time2)),
            #       'FPS={:.2f}'.format(1/(end_time3-start_time0)))
            return len(boxes),res_dir





