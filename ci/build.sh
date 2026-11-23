#!/bin/bash
set -e

echo "Checking disk space"
df -h .

echo "Downloading SDK"
wget -q --header="Authorization:${S3_WEBAPP_TOKEN}" -O sdk-"${BRANCH}.sh" "${SDK_OBJECT_URL}"

echo "Installing SDK"
chmod +x sdk-${BRANCH}.sh
./sdk-${BRANCH}.sh -d ./sdk_install -y

echo "Sourcing toolchain"
source ./sdk_install/environment-setup-armv8a-qcom-linux
echo "Running cmake"
cmake -B build -S . -DCMAKE_INSTALL_PREFIX=/usr

cmake --build build

echo "Creating install artifacts"
mkdir -p artifacts
DESTDIR="$(realpath artifacts)" cmake --install build --prefix /usr

echo "Creating archive"
tar -czf "cam-service-artifacts-${BRANCH}.tar.gz" artifacts

echo "archive created path: $(realpath cam-service-artifacts-${BRANCH}.tar.gz)"
echo "archive size: $(du -sh $(realpath cam-service-artifacts-${BRANCH}.tar.gz))"
    