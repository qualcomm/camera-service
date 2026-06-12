# Camera Service

The camera-service is a fundamental component in Qualcomm Linux that operates as a daemon, providing RPC (Remote Procedure Call) APIs for camera control. It exposes helper client APIs that handle RPC between client and server, utilizing HAL3 API to interact with the camera backend (CamX) and camera driver for hardware configuration.

The camera-service supports multiple clients and cameras simultaneously.

The camera-service provides client APIs for communication with the service. Applications such as GStreamer-based apps can use these client APIs to access the camera feed and build end-user solutions.

## Compilation Instructions

Follow the steps below to obtain the cross-compilation toolchain required to build this project.

1. Open the latest nightly build for the **wrynose** branch using the following link:

    [https://github.com/qualcomm-linux/meta-qcom/actions/workflows/nightly-build.yml](https://github.com/qualcomm-linux/meta-qcom/actions/workflows/nightly-build.yml)

1. Within the workflow run results, locate the **"build-nightly / publish_summary summary"** section.

1. Expand the **"Download URLs Details"** section to reveal the available build artifacts.

1. Under the **qcom-distro** distribution for the **qcom-armv8a** target, click on the **"Files"** link.

1. In the file list, use the **"Type"** filter to select `.sh` files, then download the following toolchain installer:

   ```
   qcom-distro-x86_64-qcom-multimedia-proprietary-image-armv8a-qcom-armv8a-toolchain-2.0.sh
   ```

1. Clone the source code from Github:
    ```
    git clone https://github.com/qualcomm/camera-service.git
    cd camera-service
    ```

1. Run CMake to generate the Makefile and build

    ```
    cmake -B build -S . -DCMAKE_INSTALL_PREFIX=/usr && cmake --build build
    ```

## Resources

- [Camera architecture](https://docs.qualcomm.com/doc/80-80022-50/topic/architecture.html#camera-architecture)

## Development

Contributions are welcome! Please refer to the [CONTRIBUTING.md](CONTRIBUTING.md) file for full details on how to contribute to this project, including:

- [Branching strategy](CONTRIBUTING.md#branching-strategy)
- [Submitting a pull request](CONTRIBUTING.md#submitting-a-pull-request)

## License

camera-service is licensed under the [BSD-3-Clause-Clear License](https://spdx.org/licenses/BSD-3-Clause-Clear.html). See [LICENSE.txt](LICENSE.txt) for the full license text.
