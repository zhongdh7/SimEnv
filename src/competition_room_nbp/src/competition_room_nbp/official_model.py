"""NBP network definition from shiyao-li/NextBestPath commit 3a745a1.

This is the inference-only architecture in next_best_path/networks/nbp_model.py.
It intentionally imports no simulator, mesh, depth, or ground-truth utilities.
"""

import torch
import torch.nn as nn


class ConvBlock(nn.Module):
    def __init__(self, ch_in, ch_out):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(ch_in, ch_out, 3, 1, 1, bias=True),
            nn.BatchNorm2d(ch_out),
            nn.ReLU(inplace=True),
            nn.Conv2d(ch_out, ch_out, 3, 1, 1, bias=True),
            nn.BatchNorm2d(ch_out),
            nn.ReLU(inplace=True),
        )

    def forward(self, value):
        return self.conv(value)


class UpConv(nn.Module):
    def __init__(self, ch_in, ch_out):
        super().__init__()
        self.up = nn.Sequential(
            nn.Upsample(scale_factor=2),
            nn.Conv2d(ch_in, ch_out, 3, 1, 1, bias=True),
            nn.BatchNorm2d(ch_out),
            nn.ReLU(inplace=True),
        )

    def forward(self, value):
        return self.up(value)


class AttentionBlock(nn.Module):
    def __init__(self, f_g, f_l, f_int):
        super().__init__()
        self.W_g = nn.Sequential(
            nn.Conv2d(f_g, f_int, 1, 1, 0, bias=True), nn.BatchNorm2d(f_int)
        )
        self.W_x = nn.Sequential(
            nn.Conv2d(f_l, f_int, 1, 1, 0, bias=True), nn.BatchNorm2d(f_int)
        )
        self.psi = nn.Sequential(
            nn.Conv2d(f_int, 1, 1, 1, 0, bias=True),
            nn.BatchNorm2d(1),
            nn.Sigmoid(),
        )
        self.relu = nn.ReLU(inplace=True)

    def forward(self, g, x):
        return x * self.psi(self.relu(self.W_g(g) + self.W_x(x)))


class NBP(nn.Module):
    def __init__(self, img_ch=5, output_ch1=8, output_ch2=1):
        super().__init__()
        self.Maxpool = nn.MaxPool2d(2, 2)
        self.Conv1 = ConvBlock(img_ch, 64)
        self.Conv2 = ConvBlock(64, 128)
        self.Conv3 = ConvBlock(128, 256)
        self.Conv4 = ConvBlock(256, 512)
        self.Conv5 = ConvBlock(512, 1024)

        self.Up5_1 = UpConv(1024, 512)
        self.Att5_1 = AttentionBlock(512, 512, 256)
        self.Up_conv5_1 = ConvBlock(1024, 512)
        self.Up4_1 = UpConv(512, 256)
        self.Att4_1 = AttentionBlock(256, 256, 128)
        self.Up_conv4_1 = ConvBlock(512, 256)
        self.Final1 = nn.Conv2d(256, output_ch1, 1)

        self.Up5_2 = UpConv(1024, 512)
        self.Att5_2 = AttentionBlock(512, 512, 256)
        self.Up_conv5_2 = ConvBlock(1024, 512)
        self.Up4_2 = UpConv(512, 256)
        self.Att4_2 = AttentionBlock(256, 256, 128)
        self.Up_conv4_2 = ConvBlock(512, 256)
        self.Up3_2 = UpConv(256, 128)
        self.Att3_2 = AttentionBlock(128, 128, 64)
        self.Up_conv3_2 = ConvBlock(256, 128)
        self.Up2_2 = UpConv(128, 64)
        self.Att2_2 = AttentionBlock(64, 64, 32)
        self.Up_conv2_2 = ConvBlock(128, 64)
        self.Final2 = nn.Sequential(nn.Conv2d(64, output_ch2, 1), nn.Sigmoid())
        self.log_vars = nn.Parameter(torch.zeros(2))

    def forward(self, value):
        x1 = self.Conv1(value)
        x2 = self.Conv2(self.Maxpool(x1))
        x3 = self.Conv3(self.Maxpool(x2))
        x4 = self.Conv4(self.Maxpool(x3))
        x5 = self.Conv5(self.Maxpool(x4))

        d5_1 = self.Up5_1(x5)
        d5_1 = self.Up_conv5_1(torch.cat((self.Att5_1(d5_1, x4), d5_1), 1))
        d4_1 = self.Up4_1(d5_1)
        out1 = self.Final1(
            self.Up_conv4_1(torch.cat((self.Att4_1(d4_1, x3), d4_1), 1))
        )

        d5_2 = self.Up5_2(x5)
        d5_2 = self.Up_conv5_2(torch.cat((self.Att5_2(d5_2, x4), d5_2), 1))
        d4_2 = self.Up4_2(d5_2)
        d4_2 = self.Up_conv4_2(torch.cat((self.Att4_2(d4_2, x3), d4_2), 1))
        d3_2 = self.Up3_2(d4_2)
        d3_2 = self.Up_conv3_2(torch.cat((self.Att3_2(d3_2, x2), d3_2), 1))
        d2_2 = self.Up2_2(d3_2)
        out2 = self.Final2(
            self.Up_conv2_2(torch.cat((self.Att2_2(d2_2, x1), d2_2), 1))
        )
        return out1, out2

