import numpy as np

from orbita2d import KinematicsModel


def test_kinematics_position():
    ratio_a = np.random.uniform(0.0, 1.0)
    ratio_b = np.random.uniform(0.0, 1.0)
    kin = KinematicsModel(ratio_a, ratio_b)

    motor_a, motor_b = np.random.uniform(-np.pi, np.pi, size=(2,))

    ring, center = kin.forward_position(motor_a, motor_b)
    motor_a_hat, motor_b_hat = kin.inverse_position(ring, center)

    assert np.isclose(motor_a, motor_a_hat)
    assert np.isclose(motor_b, motor_b_hat)


def test_kinematics_velocity():
    ratio_a = np.random.uniform(0.0, 1.0)
    ratio_b = np.random.uniform(0.0, 1.0)
    kin = KinematicsModel(ratio_a, ratio_b)

    motor_a, motor_b = np.random.uniform(-np.pi, np.pi, size=(2,))

    ring, center = kin.forward_velocity(motor_a, motor_b)
    motor_a_hat, motor_b_hat = kin.inverse_velocity(ring, center)

    assert np.isclose(motor_a, motor_a_hat)
    assert np.isclose(motor_b, motor_b_hat)


def test_kinematics_torque():
    ratio_a = np.random.uniform(0.0, 1.0)
    ratio_b = np.random.uniform(0.0, 1.0)
    kin = KinematicsModel(ratio_a, ratio_b)

    motor_a, motor_b = np.random.uniform(-np.pi, np.pi, size=(2,))

    ring, center = kin.forward_torque(motor_a, motor_b)
    motor_a_hat, motor_b_hat = kin.inverse_torque(ring, center)

    assert np.isclose(motor_a, motor_a_hat)
    assert np.isclose(motor_b, motor_b_hat)
