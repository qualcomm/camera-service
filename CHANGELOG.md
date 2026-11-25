# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]


## [1.0.6]

 - Update udev rules to accommodate the new service name
 - Enable offline JPEG encode support and fix Kodiak build issues
 - Add support for Shikra Soc
 - Update default color space and HDR mode values
 - Restrict camera service launch action to camera driver events
 - Enable downstream Pebble target and associated library names
 - Enable Binder support for Pebble target
 - Add support for device status change event propagation
 - Add locking to protect capture buffer map access
 - Fix race condition in client_cameraid_map handling
 - Skip frame sequence checks in ProviderExtension mode
 - Release buffers after snapshot stream cancellation
