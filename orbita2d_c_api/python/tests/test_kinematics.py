from orbita2d import KinematicsModel

def test_kinematics():
    kin = KinematicsModel(1.0, 1.0)
    assert kin.forward_position(0.0, 0.0) == (0.0, 0.0)

    