from tempfile import NamedTemporaryFile

from orbita2d import Orbita2dController


def get_fake_controller():
    with NamedTemporaryFile("w") as f:
        f.write("!FakeMotors")
        f.seek(0)
        fake_controller = Orbita2dController.from_config(f.name)
        return fake_controller
