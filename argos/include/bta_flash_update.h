/**  @file bta_flash_update.h
*    @version 1.3.0
*  
*    @brief This header file contains enums and structs for flash update
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

#ifndef BTA_FLASH_UPDATE_H_INCLUDED
#define BTA_FLASH_UPDATE_H_INCLUDED

#include <stdint.h>

/*  @brief BTA_FlashTarget describes the kind of data transmitted as well as
*          what the intended target (in the device/memory) of the data is          */
typedef enum {
    BTA_FlashTargetBootloader,
    BTA_FlashTargetApplication,
    BTA_FlashTargetGeneric,
    BTA_FlashTargetPixelList,
    BTA_FlashTargetLensCalibration,
    BTA_FlashTargetOtp,
    BTA_FlashTargetFactoryConfig
} BTA_FlashTarget;


/*  @brief BTA_FlashId may be needed to further specify the BTA_FlashTarget     */
typedef enum {
    BTA_FlashIdSpi,
    BTA_FlashIdParallel,
    BTA_FlashIdEmmc,
    BTA_FlashIdSd,
    BTA_FlashIdTim,
    BTA_FlashIdLim
} BTA_FlashId;

/*  @brief This configuration structure contains all the data and parameters needed for a BTAflashUpdate    */
typedef struct {
    BTA_FlashTarget target;         //<<< Type of update, indicating the target where to copy the data to
    BTA_FlashId flashId;            //<<< Parameter to distinguish between different flash modules on the device
    uint32_t address;               //<<< Address within the specified memory
    uint8_t *data;                  //<<< Data to be transmitted and saved
    uint32_t dataLen;               //<<< Size of data in bytes
} BTA_FlashUpdateConfig;

#endif
