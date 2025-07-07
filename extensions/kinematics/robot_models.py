import numpy as np
import roboticstoolbox as rtb

from spatialmath.base import transl, trotz


class LBRiiwaR800Model(rtb.DHRobot):
     def __init__(self):

        qlim_one = [np.deg2rad(-170), np.deg2rad(170)]
        qlim_two = [np.deg2rad(-120), np.deg2rad(120)]

        mm = 1e-3
        tool_offset = 230 * mm
        flange = 107 * mm

        links = [
                    rtb.RevoluteDH(alpha=-np.pi/2, 
                                   d=0.34, 
                                   a=0, 
                                   qlim=qlim_one,
                                   m=3.4525,
                                   I=[0.02183, 0, 0, 0.007703, -0.003887, 0.02083],
                                   G=1),
                    rtb.RevoluteDH(alpha=np.pi/2, 
                                   d=0, 
                                   a=0, 
                                   qlim=qlim_two,
                                   m=3.4821,
                                   I=[0.02076, 0, -0.003626, 0.02179, 0, 0.00779],
                                   G=1),
                    rtb.RevoluteDH(alpha=np.pi/2, 
                                   d=0.4, 
                                   a=0, 
                                   qlim=qlim_one,
                                   m=4.05623,
                                   I=[0.03204, 0, 0, 0.00972, 0.006227, 0.03042],
                                   G=1),
                    rtb.RevoluteDH(alpha=-np.pi/2, 
                                   d=0, 
                                   a=0, 
                                   qlim=qlim_two,
                                   m=3.4822,
                                   I=[0.02178, 0, 0, 0.02075, -0.003625, 0.007785],
                                   G=1),
                    rtb.RevoluteDH(alpha=-np.pi/2, 
                                   d=0.4, 
                                   a=0, 
                                   qlim=qlim_one,
                                   m=2.1633,
                                   I=[0.01287, 0, 0, 0.005708, -0.003946, 0.01112],
                                   G=1),
                    rtb.RevoluteDH(alpha=np.pi/2, 
                                   d=0, 
                                   a=0, 
                                   qlim=qlim_two,
                                   m=2.3466,
                                   I=[0.006509, 0, 0, 0.006259, 0.00031891, 0.004527],
                                   G=1),
                    rtb.RevoluteDH(alpha=0,
                                   d=0.126, 
                                   a=0, 
                                   qlim=[np.deg2rad(-175), np.deg2rad(175)],
                                   m=3.129,
                                   I=[0.01464, 0.0005912, 0, 0.01465, 0, 0.002872],
                                   G=1)
                ]

        super().__init__(
            links=links,
            name="LBRiiwaR800",
            manufacturer="KUKA",
        )

        # tool = transl(0, 0, tool_offset) @ trotz(-np.pi / 4)
        tool = transl(0, 0, tool_offset) @ trotz(0)
        self.tool = tool
        self.qr = np.array([0, -0.3, 0, -1.9, 0, 1.5, 0])
        self.qz = np.zeros(7)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        