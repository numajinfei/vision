import os
import glob

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from shared_interfaces.msg import Defect

import cv2
import time
import numpy as np
import torch
from torch.autograd import Variable
# from tqdm import tqdm

from torch import cat,flatten,sigmoid,relu
from torch.nn import Module,Conv2d,Linear,BatchNorm2d
import torch.nn.functional as F


class predict():
    def __init__(self,input_size,model_dir):
        print('--------------- Init Fissure System ----------------')
        self.num = 0
        self.input_size = input_size
        self.model_dir = model_dir
        print(self.model_dir)
        self.dev = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    
        self.model = torch.load(self.model_dir,map_location=str(self.dev))
        print('Use ' + str(self.dev) + '\n' +
              'Model Loaded Successfully!')
        # print('\nModel Test Successfully!')
        print('--------------- Fissure System Ready ---------------')
        self.model.eval()

    def normalize(self,img):
        out_min = 0
        out_max = 255
        in_min = np.min(img)
        in_max = np.max(img)
        a = float(out_max - out_min) / (in_max - in_min)
        b = out_min - a * in_min
        img_norm = img * a + b

        return img_norm.astype(np.uint8)

    def convert(self,img, a=1.1):
        y = float(a) * img
        y[y > 255] = 255
        img_bright = np.round(y)

        return img_bright.astype(np.uint8)

    def gamma(self,img, b=0.39):
        img_norm = img / 255.0
        img_gamma = np.power(img_norm, b) * 255.0

        return img_gamma.astype(np.uint8)

    # def get_img(self,fname,input_size):
    #     image = cv2.imread(fname)
    #     if image.shape[-1] == 3:
    #         image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #     image = cv2.resize(image,(input_size,input_size))
    #     image = self.normalize(image)
    #     image = self.gamma(image)
    #     image = self.convert(image)
    #     image = np.asarray(image, dtype='float32')/255.
    #
    #     return np.expand_dims(np.expand_dims(image,axis=0),axis=0)
    #
    # def test(self,test_fname):
    #     Neg00_fnames = glob.glob(os.path.join(test_fname+'0/', '*.bmp'))
    #     Pos05_fnames = glob.glob(os.path.join(test_fname+'mixup/', '*.bmp'))
    #     Pos10_fnames = glob.glob(os.path.join(test_fname+'1/', '*.bmp'))
    #
    #     FP = 0
    #     FP_pic = []
    #     for Neg_fname in tqdm(Neg00_fnames,desc='Neg'):
    #         input = self.get_img(Neg_fname, self.input_size)
    #         input = Variable(torch.from_numpy(input)).to(self.dev)
    #         output = self.model(input)[0]
    #         if output > 0.5:
    #             print(Neg_fname,output)
    #             FP += 1
    #             FP_pic.append(Neg_fname)
    #     for Pos_fname in tqdm(Pos05_fnames+Pos10_fnames,desc='Pos'):
    #         input = self.get_img(Pos_fname, self.input_size)
    #         input = Variable(torch.from_numpy(input)).to(self.dev)
    #         output = self.model(input)[0]
    #         if output < 0.5:
    #             print(Pos_fname, output)
    #             FP += 1
    #             FP_pic.append(Pos_fname)
        # print('FP = ',FP)
        # with open('./FP/'+str(self.model_dir.split('ep')[1][:4])+'FP.txt', 'w') as f:
        #     for x in FP_pic:
        #         f.write(str(x))
        #         f.write('\n')

    def pred(self,imgs):
        inputss = []
        for images in imgs:
            inputs = []
            for image in images:
                image = self.normalize(image)
                inputs.append(image)
            inputss.append(inputs)

        inputss = self.convert(self.gamma(np.asarray(inputss)))
        input = np.reshape(inputss, (-1,1,64,64))
        input = Variable(torch.from_numpy(np.asarray(input)/255.)).to(self.dev).to(torch.float32)
        output = self.model(input).to(self.dev).detach().numpy()
        labels = np.reshape(output,(inputss.shape[0],-1))

        return np.asarray(labels)

class concat():
    def __init__(self,input_size=64,model_dir='',test=False):
        self.input_size = input_size
        self.model_dir = model_dir

        self.p = predict(input_size=self.input_size, model_dir=self.model_dir)
        # if test:
        #     self.p.test('./aug/data/cutdata/')

    def cutimg(self,img,draw=False):
        size = img.shape
        g = 1 if len(size) == 2 else 3
        img = cv2.resize(img, (size[1] // self.input_size * self.input_size,
                               size[0] // self.input_size * self.input_size))
        size0 = img.shape
        cut0 = np.reshape(img, (-1, self.input_size, size0[1], g))  # 行划分行，不改变size0[1]（列数）
        cut1 = np.transpose(np.reshape(cut0, (cut0.shape[0], self.input_size, -1, self.input_size, g)),
                            (0, 2, 1, 3, 4))  # 划分列
        if draw:
            return cut1
        else:
            return np.squeeze(cut1, -1)

    def draw(self,img, label):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        r = np.asarray([80, 80, 255])
        lr = np.asarray([220, 220, 255])
        imgs = self.cutimg(img, draw=True)
        con = 0
        for i in range(imgs.shape[0]):
            for j in range(imgs.shape[1]):
                if label[i][j] >=  0.9:
                    imgs[i][j] = imgs[i][j] * 0.5 + r * 0.5
                    con += 1
                # elif label[i][j] >= 0.5:
                #     imgs[i][j] = imgs[i][j] * 0.5 + lr * 0.5

        imgs = np.transpose(imgs, (0, 2, 1, 3, 4))
        size = imgs.shape
        con1 = np.reshape(imgs, (size[0], size[1], -1, size[-1]))
        size1 = con1.shape
        con2 = np.reshape(con1, (-1, size1[-2], size1[-1]))

        result_path = os.getcwd() + '/cla-{:.10}'.format(time.time() / 1e9)
        # if not os.path.exists(result_path):
            # os.makedirs(result_path)
        dir = '/home/ubuntu/work/ros1_ws/chassis/install/share/manager_server/map'
        map_name = 'detect_result_{:.1f}.webp'.format(time.time())
        res_dir = dir + map_name
        cv2.imwrite(res_dir, con2,[cv2.IMWRITE_WEBP_QUALITY, 50])

        return con,res_dir


    def start(self,img):
        t0 = time.time()
        img = np.asarray(img)
        img = np.expand_dims(img,axis=-1)
        img = np.reshape(img,(1544,2064))
        # cv2.imwrite('/home/ubuntu/tmp/1.png',img)

        # imgg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        t1 = time.time()
        imgs = self.cutimg(img)
        t2 = time.time()
        labels = self.p.pred(imgs)
        t3 = time.time()
        con,res_dir = self.draw(img, labels)
        t4 = time.time()
        # print('\nread:{:.2f}ms'.format((t1 - t0)*1000),
        #       'cut:{:.2f}ms'.format((t2 - t1)*1000),
        #       'pred:{:.2f}ms'.format((t3 - t2)*1000),
        #       'draw:{:.2f}ms'.format((t4 - t3)*1000),
        #       'all:{:.2f}ms'.format((t4 - t0)*1000))

        return con,res_dir

class DefectDetection(Node):

    def __init__(self):
        super().__init__('defect_detection_node')

        self.model_dir = '/home/ubuntu/am_v2_ws/src/defect_detection/logs/ep314-lr0.000010-loss0.000056-val_loss1.054761-acc=83.740%.pth'
        self.concatImage = concat(model_dir = self.model_dir)

        self.subImage = self.create_subscription(
            Image,
            '~/image',
            self.ImageProcess,
            10)
        self.pubResult = self.create_publisher(Defect, '~/result', 10)
        # self.pubStatus = self.create_publisher(String, '/condition_monitoring/status', 10)
        print('defect_detection_node initialize successfully')

    def ImageProcess(self, image):
        self.get_logger().info('subscrip aa image, frame_id is "%s"' % image.header.frame_id)
        value, imagePath = self.concatImage.start(image.data)

        result = Defect()
        result.value = value
        result.picture = imagePath
        print(value,imagePath)
        self.pubResult.publish(result)
        self.get_logger().info('publish result, value is '+ str(value)+ ' picture is' + str(imagePath))


def main(args=None):
    ''' ros node spin '''
    rclpy.init(args=args)
    defect_detection = DefectDetection()
    rclpy.spin(defect_detection)
    defect_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()