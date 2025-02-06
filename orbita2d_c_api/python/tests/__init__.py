from tempfile import NamedTemporaryFile

from orbita2d import Orbita2dController


def get_fake_controller():
    with NamedTemporaryFile("w") as f:
        f.write(
            """
            !FakeMotors
                inverted_axes:
                    - false
                    - false
                orientation_limits:
                    - {min: -3.14159265, max: 3.14159265}
                    - {min: -3.14159265, max: 3.14159265}
                """
        )
        f.seek(0)
        fake_controller = Orbita2dController.from_config(f.name)
        return fake_controller
