/**  @file bta_status.h
*    @version 1.3.0
*  
*    @brief This header file contains the status ID enum used as return value for most functions
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

#ifndef BTA_STATUS_H_INCLUDED
#define BTA_STATUS_H_INCLUDED

#define BTA_StatusDeviceSpecificErrorOffset                 4096

/*    @brief Error code for error handling
*            Not listed Error codes < -4096 are device specific error codes. Please refer to the device manual   */
typedef enum {
    BTA_StatusOk                        = 0,
    BTA_StatusInvalidParameter          = -1,
    BTA_StatusIllegalOperation          = -2,

    BTA_StatusProtocolViolation         = -4,
    BTA_StatusTimeOut                   = -5,
    BTA_StatusDeviceUnreachable         = -6,

    BTA_StatusInvalidAddress            = -10,

    BTA_StatusNotConnected              = -12,

    BTA_StatusInvalidProtocolVersion    = -14,
    BTA_StatusInvalidDataFormat         = -15,
    BTA_StatusRuntimeError              = -16,
    BTA_StatusOutOfMemory               = -17,

    BTA_StatusNotSupported              = -21,
    BTA_StatusFlashingFailed            = -22,
    BTA_StatusInvalidConfiguration      = -23,
    BTA_StatusCommunicationError        = -24,
    BTA_StatusInvalidFile               = -25,
    BTA_StatusUnexpectedDataFormat      = -26,

    BTA_StatusUnknown                   = -1024
} BTA_Status;


#endif
