from . import _orbita2d

class KinematicsModel:
    def __init__(self, ratio_a, ratio_b) -> None:
        self.ratio_a = ratio_a
        self.ratio_b = ratio_b

    def forward_position(self, angle_a, angle_b):
        roll = _orbita2d.ffi.new("double *")
        pitch = _orbita2d.ffi.new("double *")

        _orbita2d.lib.forward_position(self.ratio_a, self.ratio_b, angle_a, angle_b, roll, pitch)

        return roll[0], pitch[0]