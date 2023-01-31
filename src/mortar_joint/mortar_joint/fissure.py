import glob
import os
import time
import cv2
import torch
import numpy as np
from torch.autograd import Variable
from torch import cuda,from_numpy
from U_net import U_plus

import rclpy
from rclpy.node import Node
from shared_interfaces.msg import ImageC
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class detect():
    def __init__(self,model_path):
        self.input_size = (2048//8,1536//8)
        self.model_path = model_path
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = torch.load(self.model_path,map_location=self.device).eval()
        # with open('/home/ubuntu/tmp/load model.txt','w') as f:
        #     f.write('load model')


    def start(self,img):
        img = self.get_img(img, self.input_size)

        inputs = from_numpy(img).to(self.device)
        t1 = time.time()
        output = self.model(inputs)
        t2 = time.time()
        
        # with open('/home/ubuntu/tmp/time.txt','w') as f:
        #     f.write( str(t2-t1))

        img = output.to('cpu').detach().numpy() * 255

        img = np.squeeze(img, 0)
        img = np.squeeze(img, 0)
        img = cv2.resize(img,(2064,1544))
        img = (img > 100) * 255
        img = img.astype(np.uint8)

        return img

    def get_img(self,image,input_size):
        image = cv2.resize(image,input_size)
        image = np.array(image, dtype='float32')
        image /= 255.

        return np.expand_dims(np.expand_dims(image,axis=0),axis=0)

class Fissure(Node):
    def __init__(self):
        super().__init__('mortar_joint_node')

        self.model_dir = '/home/ubuntu/am_v2_ws/src/mortar_joint/logs/fissure.pth'
        self.detectImage = detect(model_path=self.model_dir)
        self.subImage = self.create_subscription(
            ImageC,
            '~/image_subscribe',
            self.ImageProcess,
            10)

        self.pubImage = self.create_publisher(
            ImageC,
            '~/image_publish', 10)

        print('mortar_joint_node initialize successfully')

    def ImageProcess(self, image):
        self.get_logger().info('subscribe an image, frame_id is "%s"' % image.frame_id)
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image.image)
        # img = self.detectImage.start(image.image.data[1])

        img = self.detectImage.start(cv_image)   
        with open('/home/ubuntu/tmp/type.txt','w') as f:
            f.write( str(img.dtype) )

        imageC = ImageC()
        imageC.header = image.header
        imageC.id = image.id
        imageC.engineering = image.engineering
        imageC.option = image.option
        imageC.angle = image.angle
        imageC.frame_id = image.frame_id
        imageC.image = bridge.cv2_to_imgmsg(img)
        # imageC.image.height = img.shape[0]
        # imageC.image.width = img.shape[1]
        # imageC.image.encoding = 'mono8'
        # imageC.image.is_bigendian = False
        # imageC.image.step = img.shape[1]
        # imageC.image.data = np.array(img).tostring()
        self.pubImage.publish(imageC)
        self.get_logger().info('publish an image, frame_id is "%s"' % imageC.frame_id)

        res_dir = "/home/ubuntu/tmp/images/origin_image.png"
        cv2.imwrite(res_dir, cv_image)

        res_dir = "/home/ubuntu/tmp/images/mortar_joint.png"
        cv2.imwrite(res_dir, img)


def main(args=None):
    ''' ros node spin '''
    rclpy.init(args=args)
    fissure = Fissure()
    rclpy.spin(fissure)
    fissure.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()