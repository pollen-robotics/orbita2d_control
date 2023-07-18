from . import get_fake_controller


def test_multiple_controller():
    controllers = [get_fake_controller() for _ in range(10)]
    uids = [controller.uid for controller in controllers]
    assert len(set(uids)) == len(uids)

    for i, controller in enumerate(controllers):
        controller.set_target_orientation((i, i))

    for i, controller in enumerate(controllers):
        assert controller.get_target_orientation() == (i, i)
