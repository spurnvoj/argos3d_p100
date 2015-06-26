/**  @file bta_event.h
*    @version 1.3.0
*  
*    @brief This header file contains the event ID enum used in the infoCallback function
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

#ifndef BTA_EVENT_H_INCLUDED
#define BTA_EVENT_H_INCLUDED

#include <stdint.h>

/*  @brief A set of values which describe the kind of an event  */
typedef enum {
    BTA_EventIdConnectionEstablished            = 3,

    BTA_EventIdOverTemperature                  = 6,
    BTA_EventIdCalibrationError                 = 7,
    BTA_EventIdInformation                      = 8,
    BTA_EventIdSuccess                          = 9,


    BTA_EventIdGenericError                     = -1,
    BTA_EventIdFatalError                       = -2,
    BTA_EventIdDataInterfaceError               = -3,

    BTA_EventIdControlInterfaceError            = -5,

    BTA_EventIdConnectionError                  = -7,

    BTA_EventIdOutOfMemoryError                 = -10
} BTA_EventId;

#endif
