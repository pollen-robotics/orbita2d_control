# Orbita2d python bindings

This folder contains the Python bindings generated using the [orbita2d C-API library](../README.md). It is generated using [maturin and cffi](https://github.com/PyO3/maturin).

## Installation

* Install maturin in your Python virtual environment. For example, using pip: `pip install maturin`

From the orbita2d_c_api folder, run:

```maturin develop```

or

```maturin build && pip install -e .```

Then you should be able to directly import the orbita2d module in your Python code:

```python
import orbita2d
```

## Usage

```python

import numpy as np
import time

from orbita2d import Orbita2dController

orbita = Orbita2dController.from_config("path_to_my_config_file")

orbita.enable_torque(True)

t0 = time.time()
while time.time() - t0 < 15:
    ring = np.deg2rad(20) * np.sin(2 * np.pi * 0.15 * time.time())
    orbita.set_target_orientation((ring, 0.0))

    time.sleep(0.001)

orbita.disable_torque()
```

See [Python API example notebook](./API_example.ipynb) for more complex usage.