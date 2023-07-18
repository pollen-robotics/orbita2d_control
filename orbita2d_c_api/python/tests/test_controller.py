from tempfile import NamedTemporaryFile

from orbita2d import Orbita2dController


def get_fake_controller():
    with NamedTemporaryFile("w") as f:
        f.write("!FakeMotors")
        f.seek(0)
        fake_controller = Orbita2dController.from_config(f.name)
        return fake_controller


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
