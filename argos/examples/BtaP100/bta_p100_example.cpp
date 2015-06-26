/**  @file bta_p100_example.cpp
*    @version 1.0.0
*  
*    @brief This example shows a common use case of the SDK
*  
*    BLT_DISCLAIMER
*  
*    @author Alex Falkensteiner
*  
*    @cond svn
*  
*    Information of last commit
*    $Rev::               $:  Revision of last commit
*    $Author::            $:  Author of last commit
*    $Date::              $:  Date of last commit
*  
*    @endcond
*/

#include <stdio.h>
#include <bta.h>

static void BTA_CALLCONV infoEvent(BTA_EventId eventId, int8_t *msg) {
    printf("   Callback: infoEvent (%d) %s\n", eventId, msg);
}


int main() {
    BTA_Config config;
    printf("BTAinitConfig()\n");
    BTA_Status status;
    status = BTAinitConfig(&config);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    config.frameMode = BTA_FrameModeXYZ;
    config.infoEvent = &infoEvent;
    config.verbosity = 5;

    
    //BTA_Status status;
    BTA_Handle btaHandle;
    printf("BTAopen()\n");
    status = BTAopen(&config, &btaHandle);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    
    BTA_DeviceInfo *deviceInfo;
    printf("BTAgetDeviceInfo()\n");
    status = BTAgetDeviceInfo(btaHandle, &deviceInfo);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    printf("Device type: 0x%x\n", deviceInfo->deviceType);
    printf("BTAfreeDeviceInfo()\n");
    BTAfreeDeviceInfo(deviceInfo);
    
    printf("Service running: %d\n", BTAisRunning(btaHandle));
    printf("Connection up: %d\n", BTAisConnected(btaHandle));
    
    uint32_t usValue;
    printf("BTAreadRegister()\n");
    status = BTAreadRegister(btaHandle, 0x81, &usValue, 0);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    
    printf("BTAwriteRegister()\n");
    status = BTAwriteRegister(btaHandle, 0x81, &usValue, 0);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    
    // Actively get a frame
    BTA_Frame *frame;
    printf("BTAgetFrame()\n");
    status = BTAgetFrame(btaHandle, &frame, 3000);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    
    void *xCoordinates, *yCoordinates, *zCoordinates;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    printf("BTAgetXYZcoordinates()\n");
    status = BTAgetXYZcoordinates(frame, &xCoordinates, &yCoordinates, &zCoordinates, &dataFormat, &unit, &xRes, &yRes);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    if (dataFormat == BTA_DataFormatFloat32) {
        if (unit == BTA_UnitMeter) {
            printf("Got 3D data\n");
            // as expected -> process point in space: ( xCoordinates[i], yCoordinates[i], zCoordinates[i] )
        }
    }
    printf("BTAfreeFrame()\n");
    status = BTAfreeFrame(&frame);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    printf("BTAclose()\n");
    status = BTAclose(&btaHandle);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    printf("Hit <Return> to end the example\n");
    fgetc(stdin);
}
