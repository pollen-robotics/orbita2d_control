from . import get_fake_controller


def test_controller_torque():
    fake_controller = get_fake_controller()

    fake_controller.disable_torque()
    assert not fake_controller.is_torque_on()

    fake_controller.set_target_orientation((0.5, 0.25))

    fake_controller.enable_torque(False)
    assert fake_controller.is_torque_on()
    assert fake_controller.get_target_orientation() == (0.5, 0.25)
    assert fake_controller.get_current_orientation() == (0.5, 0.25)

    fake_controller.disable_torque()
    assert not fake_controller.is_torque_on()

    fake_controller.set_target_orientation((0.0, 0.0))

    fake_controller.enable_torque(True)
    assert fake_controller.is_torque_on()
    assert fake_controller.get_target_orientation() == (0.5, 0.25)
    assert fake_controller.get_current_orientation() == (0.5, 0.25)
