#include <iostream>
#include <cstring>
#include "MvCameraControl.h"

int main()
{
    std::cout << "=== Hikvision MVS SDK USB Camera Detection ===" << std::endl;
    
    // 枚举USB设备
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList;  // 存储设备列表
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));  // 清空结构体
    
    std::cout << "Scanning for USB cameras..." << std::endl;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    
    if (MV_OK != nRet)
    {
        std::cout << "ERROR: Failed to enumerate devices! Error code: 0x" << std::hex << nRet << std::dec << std::endl;
        return -1;
    }
    
    std::cout << "=== Scan Results ===" << std::endl;
    std::cout << "Found " << stDeviceList.nDeviceNum << " USB device(s)" << std::endl;
    
    // 未找到相机
    if (stDeviceList.nDeviceNum == 0)
    {
        std::cout << "\nNo USB cameras detected!" << std::endl;
        return 0;
    }
    
    // 显示相机信息
    std::cout << "\n=== Detected USB Cameras ===" << std::endl;
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        
        std::cout << "Camera " << i << ":" << std::endl;
        
        if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            std::cout << "  Type: USB Camera" << std::endl;
            std::cout << "  Model: " << pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName << std::endl;
            std::cout << "  Serial: " << pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
            std::cout << "  Vendor: " << pDeviceInfo->SpecialInfo.stUsb3VInfo.chVendorName << std::endl;
        }
        else
        {
            std::cout << "  Type: Unknown (" << pDeviceInfo->nTLayerType << ")" << std::endl;
        }
        std::cout << std::endl;
    }
    
    // 测试连接第一个相机
    if (stDeviceList.nDeviceNum > 0) 
    {
        std::cout << "=== Testing Connection to Camera 0 ===" << std::endl;
        
        void* handle = nullptr;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);  // 创建句柄
        
        if (MV_OK != nRet) 
        {
            std::cout << "ERROR: Failed to create handle! Error: 0x" << std::hex << nRet << std::dec << std::endl;
            return -1;
        }
        
        nRet = MV_CC_OpenDevice(handle);  // 连接相机
        if (MV_OK != nRet) 
        {
            std::cout << "ERROR: Failed to open device! Error: 0x" << std::hex << nRet << std::dec << std::endl;
            MV_CC_DestroyHandle(handle);
            return -1;
        }
        
        std::cout << "SUCCESS: Camera connected and ready!" << std::endl;
        
        // 获取设备信息
        MV_CC_DEVICE_INFO stDeviceInfo;
        memset(&stDeviceInfo, 0, sizeof(MV_CC_DEVICE_INFO));
        nRet = MV_CC_GetDeviceInfo(handle, &stDeviceInfo);
        if (MV_OK == nRet) 
        {
            std::cout << "Device information retrieved successfully" << std::endl;
        }
        
        // 关闭连接
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        std::cout << "Camera connection closed." << std::endl;
    }
    
    std::cout << "=== Test Completed ===" << std::endl;
    return 0;
}