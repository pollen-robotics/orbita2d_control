# Orbita2d C-API library

This repository contains the C-API library to control the Orbita2d actuator. It is generated using [cbindgen](https://github.com/mozilla/cbindgen).

This lets you control the actuator from any language that can call C functions. For example, you can use it from Python using the [orbita2d python bindings](./python/README.md).

The full API can be seen in the generated header file: [orbita2d.h](./orbita2d.h).

## Build

* Install rust and cargo: https://www.rust-lang.org/tools/install
* Install cbindgen: ```cargo install cbindgen```
* Build the library: ```cargo build --release``` (both static and dynamic)

The library is generated in the `target/release` folder.