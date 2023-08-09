========================================================================
    CONSOLE APPLICATION : ColorCameraExtrinsicsToROS Project Overview
========================================================================

This is a simple application that prints ColorCamera/Range extrinsics
in the form of ROS launchfile with static_transform_publisher.

Those are extrinsics between raw RGB image and depth.

Note that this is different from device computed RGB texture which is aligned to depth
and extrinsics would be identity in such case.

You will learn how to:

* obtain ROS compatible launchfile publishing ColorCamera/Range extrinsics.

How to build:

1. Copy ColorCameraExtrinsicsToROS folder to a location with Read and Write
   permissions (using the name <source>)
2. Open CMake
   2.1. Set Source code to <source>
   2.2. Set Binaries to <source>/_build or any other writable location
   2.3. Click Configure and Generate
3. Build project

How to use:
1. Set camera parameters (ideally with ROS driver)
2. Run PhoXiControl
   2.1. Connect to a scanner
   2.2. Make sure `Structure->ColorCameraImage` transfer is enabled
3. Run ColorCameraExtrinsicsToROS application and redirect output to launch file
  3.1 e.g. `./ColorCameraExtrinsicsToROS > MyColorCameraRangeExtrinsics.yaml`

The application will print out launchfile publishing ColorCamera/Range extrinsics of the connected scanner.
If not connected to any scanner, it will automatically connect to the first
in PhoXiControl.

/////////////////////////////////////////////////////////////////////////////
