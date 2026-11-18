"""Petit reseau PyTorch : image RGB -> fond, rouge, vert, bleu."""
import torch
from torch import nn

class CubeNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers=nn.Sequential(nn.Conv2d(3,16,1),nn.ReLU(),nn.Conv2d(16,32,1),nn.ReLU(),nn.Conv2d(32,4,1))
    def forward(self,x):
        return self.layers(x)
