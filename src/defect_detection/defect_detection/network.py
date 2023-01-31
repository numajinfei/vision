from torch import cat,flatten,sigmoid,relu
from torch.nn import Module,Conv2d,Linear,BatchNorm2d
import torch.nn.functional as F

class body(Module):
    def __init__(self):
        super().__init__()
        self.conv1 = Conv2d(1, 8, kernel_size=(3,3), padding=(1,1))
        self.BN1 = BatchNorm2d(8)
        self.conv2 = Conv2d(8, 8, kernel_size=(3, 3), stride=(2,2), padding=(1, 1))
        self.BN2 = BatchNorm2d(8)
        self.conv3 = Conv2d(8, 16, kernel_size=(3, 3), padding=(1, 1))
        self.BN3 = BatchNorm2d(16)
        self.conv4 = Conv2d(16, 16, kernel_size=(3, 3), stride=(2,2), padding=(1, 1))
        self.BN4 = BatchNorm2d(16)
        self.conv5 = Conv2d(16, 32, kernel_size=(3, 3), padding=(1, 1))
        self.BN5 = BatchNorm2d(32)
        self.conv6 = Conv2d(32, 32, kernel_size=(3, 3), stride=(2,2), padding=(1, 1))
        self.BN6 = BatchNorm2d(32)
        self.conv7 = Conv2d(32, 16, kernel_size=(3, 3), padding=(1, 1))
        self.BN7 = BatchNorm2d(16)
        self.conv8 = Conv2d(16, 16, kernel_size=(3, 3), stride=(2,2), padding=(1, 1))
        self.BN8 = BatchNorm2d(16)
        self.conv9 = Conv2d(16, 8, kernel_size=(3, 3), padding=(1, 1))
        self.BN9 = BatchNorm2d(8)
        self.conv10 = Conv2d(8, 8, kernel_size=(4, 4), padding=(0, 0))
        self.BN10 = BatchNorm2d(8)
        self.linear = Linear(8,1)

    def forward(self, input):
        x = F.leaky_relu(self.BN1(self.conv1(input)))
        x = F.leaky_relu(self.BN2(self.conv2(x)))
        x = F.leaky_relu(self.BN3(self.conv3(x)))
        x = F.leaky_relu(self.BN4(self.conv4(x)))
        x = F.leaky_relu(self.BN5(self.conv5(x)))
        x = F.leaky_relu(self.BN6(self.conv6(x)))
        x = F.leaky_relu(self.BN7(self.conv7(x)))
        x = F.leaky_relu(self.BN8(self.conv8(x)))
        x = F.leaky_relu(self.BN9(self.conv9(x)))
        x = F.leaky_relu(self.BN10(self.conv10(x)))
        x = flatten(x,1)
        x = sigmoid(self.linear(x))

        return x

from torchsummary import summary
def main():
    net = body()
    summary(net,(1,64,64))

if __name__ == '__main__':
    main()