/**  @file bta_p100_example_minimal.cpp
*    @version 1.0.0
*  
*    @brief This example shows the minimum functionalities of the SDK
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

int main() {
    BTA_Config config;
    printf("BTAinitConfig()\n");
    BTA_Status status;
    status = BTAinitConfig(&config);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    config.frameMode = BTA_FrameModeXYZAmp; //BTA_FrameModeDistAmpFlags;


    
    //BTA_Status status;
    BTA_Handle btaHandle;
    printf("BTAopen()\n");
    status = BTAopen(&config, &btaHandle);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    
    // Actively get a frame
    BTA_Frame *frame;
    printf("BTAgetFrame()\n");
    status = BTAgetFrame(btaHandle, &frame,3000);  //3000;
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    
    uint16_t *distances;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    printf("BTAgetDistances()\n");
    status = BTAgetDistances(frame, (void **)&distances, &dataFormat, &unit, &xRes, &yRes); //xRes,yRes rozmery
    printf("xRes: %d, yRes: %d\n",xRes,yRes);
    printf("%d\n",distances[0]);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    if (dataFormat == BTA_DataFormatUInt16) {
        if (unit == BTA_UnitMillimeter) {
            printf("Got distance data\n");
            // as expected -> process amplitude data: amplitudes[i]
        }
    }
    
    uint16_t *amplitudes;
    printf("BTAgetAmplitudes()\n");
    status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
    if (status != BTA_StatusOk) {
        printf("error: %d\n", status);
        return -1;
    }
    if (dataFormat == BTA_DataFormatUInt16) {
        if (unit == BTA_UnitUnitLess) {
            printf("Got amplitude data\n");
            // as expected -> process amplitude data: amplitudes[i]
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
