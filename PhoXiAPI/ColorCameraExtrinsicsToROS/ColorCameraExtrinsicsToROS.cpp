/*
* Photoneo's API Example - ColorCameraExtrinsicsToROS.cpp
* Prints ColorCamera/Range extrinsics in the form of ROS launchfile with static_transform_publisher.
* Those are extrinsics between raw RGB image and depth.
*
* Note that this is different from device computed RGB texture which is aligned to depth
* and extrinsics would be identity in such case.
*
* Modified from ExtendRobotics's ColorCameraCalibrationToROS
*
* Mofification Contributors:
* - Bartosz Meglicki <bartosz.meglicki@extendrobotics.com> (2023)
*/

#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include <eigen3/Eigen/Geometry>

#include "PhoXi.h"

//Print out list of device info to standard output
void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList);
//Print out device info to standard output
void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo);
//Print out calibration parameters
void printROSExtrinsics(pho::api::PPhoXi &PhoXiDevice);

int main(int argc, char *argv[])
{
    pho::api::PhoXiFactory Factory;

    //Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning())
    {
        std::cerr << "PhoXi Control Software is not running" << std::endl;
        return 0;
    }

    //Get List of available devices on the network
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
    if (DeviceList.empty())
    {
        std::cerr << "PhoXi Factory has found 0 devices" << std::endl;
        return 0;
    }

    //output ROS launchfile XML declaration
    std::cout << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl << std::endl;

    //start XML comment keeping information how file was generated
    std::cout << "<!--" << std::endl;

    printDeviceInfoList(DeviceList);

    //Try to connect device opened in PhoXi Control, if any
    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice)
    {
        std::cout << "# You have already PhoXi device opened in PhoXi Control, the API Example is connected to device: "
            << (std::string) PhoXiDevice->HardwareIdentification << std::endl;
    }
    else
    {
        std::cout << "# You have no PhoXi device opened in PhoXi Control, the API Example will try to connect to first device in device list" << std::endl;
        PhoXiDevice = Factory.CreateAndConnect(DeviceList.front().HWIdentification);
    }

    //Check if device was created
    if (!PhoXiDevice)
    {
        std::cerr << "Your device was not created!" << std::endl;
        return 0;
    }

    //Check if device is connected
    if (!PhoXiDevice->isConnected())
    {
        std::cerr << "Your device is not connected" << std::endl;
        return 0;
    }

    std::cout << std::endl;

    //Print out ROS launchfile publishing tf2 extrinsics
    printROSExtrinsics(PhoXiDevice);

    //Disconnect PhoXi device
    PhoXiDevice->Disconnect();
    return 0;
}

void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList)
{
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        std::cout << "# Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo)
{
    std::cout << "#  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "#  Hardware Identification: " << DeviceInfo.HWIdentification << std::endl;
    std::cout << "#  Type:                    " << std::string(DeviceInfo.Type) << std::endl;
    std::cout << "#  Firmware version:        " << DeviceInfo.FirmwareVersion << std::endl;
    std::cout << "#  Variant:                 " << DeviceInfo.Variant << std::endl;
    std::cout << "#  IsFileCamera:            " << (DeviceInfo.IsFileCamera ? "Yes" : "No") << std::endl;
    std::cout << "#  Feature-Alpha:           " << (DeviceInfo.CheckFeature("Alpha") ? "Yes" : "No") << std::endl;
    std::cout << "#  Feature-Color:           " << (DeviceInfo.CheckFeature("Color") ? "Yes" : "No") << std::endl;
    std::cout << "#  Status:                  "
        << (DeviceInfo.Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
        << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
        << std::endl << std::endl;
}

void printVector(const std::string &name, const pho::api::Point3_64f &vector)
{
    std::cout << "# " << name << ": ["
        << vector.x << "; "
        << vector.y << "; "
        << vector.z << "]"
        << std::endl;
}

Eigen::Quaterniond rotationAxesToQuaternion(const pho::api::Point3_64f &X, const pho::api::Point3_64f &Y, const pho::api::Point3_64f &Z)
{
  Eigen::Matrix3d m;

  // We need to be careful about the order, as
  // Photoneo vectors form column major rotation matrix
  // while Eigen::Matrix3d expects row-major
  m << X.x, Y.x, Z.x,
       X.y, Y.y, Z.y,
       X.z, Y.z, Z.z;

  Eigen::Quaterniond q(m);

  return q;
}

void printROSExtrinsics(pho::api::PPhoXi& PhoXiDevice)
{
    if (!PhoXiDevice->isAcquiring())
    {
        PhoXiDevice->StartAcquisition();
    }
    int FrameID = PhoXiDevice->TriggerFrame();
    if (FrameID < 0)
    {
        //If negative number is returned trigger was unsuccessful
        std::cerr << "Trigger was unsuccessful! code=" << FrameID << std::endl;
        return;
    }

    pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID);

    if(!Frame)
    {
        std::cerr << "Failed to retrieve the frame!" << std::endl;
        return;
    }

    const pho::api::TextureRGB16 &ColorFrame = Frame->ColorCameraImage;

    if(ColorFrame.Empty())
    {
        std::cerr << "Failed to retrieve the color camera image!" << std::endl
                  << "Make sure that ColorCameraImage transfer is enabled in PhoXiControl" << std::endl
                  << "Make sure that appriopriate settings for color camera are set" << std::endl;
        return;
    }

    std::cout << "# ColorCameraImage (raw RGB, this is not depth aligned RGB texture!)" << std::endl;
    std::cout << "# ColorCameraScale: " << Frame->Info.ColorCameraScale.Width << "x" << Frame->Info.ColorCameraScale.Height << std::endl;
    std::cout << std::endl;

    std::cout << "# 3D sensor should be [0,0,0] with identity rotation (frame of reference)" << std::endl;
    std::cout << "# otherwise this tool will not generate correct extrinsics" << std::endl;
    printVector("3D sensor position", Frame->Info.SensorPosition);
    printVector("3D sensor X axis", Frame->Info.SensorXAxis);
    printVector("3D sensor Y axis", Frame->Info.SensorYAxis);    
    printVector("3D sensor Z axis", Frame->Info.SensorZAxis);

    std::cout << std::endl;

    printVector("color camera position", Frame->Info.ColorCameraPosition);
    printVector("color camera X axis", Frame->Info.ColorCameraXAxis);
    printVector("color camera Y axis", Frame->Info.ColorCameraYAxis);    
    printVector("color camera Z axis", Frame->Info.ColorCameraZAxis);        

    Eigen::Quaterniond Q = rotationAxesToQuaternion(Frame->Info.ColorCameraXAxis, Frame->Info.ColorCameraYAxis, Frame->Info.ColorCameraZAxis);

    std::string parent_frame = "color_camera_optical_frame";
    std::string child_frame = "range_optical_frame";
    std::string node_name = "range_color_camera_link_broadcaster";

    auto T = Frame->Info.ColorCameraPosition;

    //ROS uses meters, Photoneo uses millimeters
    const double MM_TO_M = 0.001;

    //finish XML comment keeping information how file was generated
    std::cout << "-->" << std::endl;

    std::cout << "<launch>" << std::endl;

    std::cout << "  <node pkg=\"tf2_ros\" "
              <<         "type=\"static_transform_publisher\" "
              <<         "name=\"" << node_name <<  "\" "
              <<         "args=\""
              << T.x * MM_TO_M << " " << T.y * MM_TO_M << " " << T.z * MM_TO_M << " "
              << Q.x() << " " << Q.y() << " " << Q.z() << " " << Q.w() << " "
              << child_frame << " " << parent_frame
              << "\" />" << std::endl;

    std::cout << "</launch>" << std::endl;
}
