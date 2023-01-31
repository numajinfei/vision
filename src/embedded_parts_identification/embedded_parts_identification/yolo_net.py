import torch
from torchsummary import summary
from torch.nn import Module,Conv2d,BatchNorm2d,Sequential,LeakyReLU,Upsample
import torch.nn.functional as F


class network(Module):
    def __init__(self,num_anchors,num_classes):
        super().__init__()
        start_c = 8
        self.conv1 = Conv2d(1, start_c, kernel_size=(3,3),stride=(1,1),padding=(1,1))
        self.BN1 = BatchNorm2d(start_c)

        self.conv2 = self.CBLS(start_c)

        self.conv3 = self.CBLS(start_c*2)
        self.conv3_1 = self.CBLSD(start_c*2)

        self.conv4 = self.CBLS(start_c*4)
        self.conv4_1 = self.CBLSD(start_c*4)

        self.conv5 = self.CBLS(start_c*8)
        self.conv5_1 = self.CBLSD(start_c*8)

        self.conv6 = self.CBLL(start_c*8)
        self.conv6_1 = self.CBLLD(start_c*8)

        self.conv7 = self.CBLL(start_c*4)
        self.conv7_1 = self.CBLLD(start_c*4)

        self.conv8 = self.CBLL(start_c*2)
        self.conv8_1 = self.CBLLD(start_c*2)

        self.Upsample1 = Upsample(scale_factor=2)
        self.conv9 = self.CBLLD(start_c*2)

        self.conv9_1 = Conv2d(start_c*6, start_c*2, kernel_size=(3, 3), stride=(1,1),padding=(1,1))
        self.BN9_1 = BatchNorm2d(start_c*2)
        self.conv9_2 = Conv2d(start_c*2, num_anchors * (num_classes + 5), kernel_size=(3,3),stride=(1,1),padding=(1,1))

    def CBLS(self,c):
        return Sequential(
            Conv2d(c,c*2, kernel_size=(3, 3), stride=(2,2),padding=(1,1)),
            BatchNorm2d(c*2),
            LeakyReLU(0.1,inplace=True),
        )

    def CBLSD(self,c):
        return Sequential(
            Conv2d(c*2,c*2, kernel_size=(1,1),stride=(1,1)),
            BatchNorm2d(c*2),
            LeakyReLU(0.1,inplace=True),
            # Conv2d(c,c*2, kernel_size=(3,3),stride=(1,1),padding=(1,1)),
            # BatchNorm2d(c*2),
            # LeakyReLU(0.1,inplace=True)
        )

    def CBLL(self,c):
        return Sequential(
            Conv2d(c*2,c, kernel_size=(3, 3), stride=(2,2),padding=(1,1)),
            BatchNorm2d(c),
            LeakyReLU(0.1,inplace=True),
        )

    def CBLLD(self,c):
        return Sequential(
            Conv2d(c,c, kernel_size=(1,1),stride=(1,1)),
            BatchNorm2d(c),
            LeakyReLU(0.1,inplace=True),
            # Conv2d(c*2,c, kernel_size=(3,3),stride=(1,1),padding=(1,1)),
            # BatchNorm2d(c),
            # LeakyReLU(0.1,inplace=True),
        )

    def head(self,x):
        x9_1 = F.leaky_relu_(self.BN9_1(self.conv9_1(x)))
        x9_2 = self.conv9_2(x9_1)

        return x9_2

    def neck(self,s,l):
        x = self.conv9(l)
        x = self.Upsample1(x)
        x = torch.cat((x, s), dim=1)

        return x

    def backbone(self,x):
        x1 = F.leaky_relu_(self.BN1(self.conv1(x)))
        x2 = self.conv2(x1)
        x3 = self.conv3(x2)
        x3_1 = self.conv3_1(x3)
        x3_2 = torch.add(x3_1,x3)
        x4 = self.conv4(x3_2)
        x4_1 = self.conv4_1(x4)
        x4_2 = torch.add(x4_1,x4)
        x5 = self.conv5(x4_2)
        x5_1 = self.conv5_1(x5)
        x5_2 = torch.add(x5_1,x5)
        x6 = self.conv6(x5_2)
        x6_1 = self.conv6_1(x6)
        x6_2 = torch.add(x6_1,x6)
        x7 = self.conv7(x6_2)
        x7_1 = self.conv7_1(x7)
        x7_2 = torch.add(x7_1,x7)
        x8 = self.conv8(x7_2)
        x8_1 = self.conv8_1(x8)
        x8_2 = torch.add(x8_1,x8)

        return x7_2,x8_2

    def forward(self, input):
        s,l = self.backbone(input)
        x = self.neck(s,l)
        x = self.head(x)

        return x

def main():

    net = network(2,2)
    x = torch.randn([1,1,1920,1408])
    summary(net, (1,1920,1408))
    # torch.save(net, './origin.pth')

    # net.eval()
    # # Export the model
    # torch.onnx.export(net.to('cpu'),  # model being run
    #                   x,  # model input (or a tuple for multiple inputs)
    #                   "net.onnx",  # where to save the model
    #                   export_params=True,  # store the trained parameter weights inside the model file
    #                   opset_version=10,  # the ONNX version to export the model to
    #                   do_constant_folding=True,  # whether to execute constant folding for optimization
    #                   input_names=['modelInput'],  # the model's input names
    #                   output_names=['modelOutput'],  # the model's output names
    #                   dynamic_axes={'modelInput': {0: 'batch_size'},  # variable length axes
    #                                 'modelOutput': {0: 'batch_size'}})
    #

if __name__ == '__main__':
    main()











