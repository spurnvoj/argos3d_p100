/**  @file bta.h
*    @version 1.3.0
*  
*    @brief The main header for BltTofApi. Includes all interface functions, the config struct and
*           the declaration of the config struct organisation
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

#ifndef BTA_H_INCLUDED
#define BTA_H_INCLUDED

#include "bta_status.h"
#include "bta_frame.h"
#include "bta_discovery.h"
#include "bta_flash_update.h"
#include "bta_event.h"

#define PLAT_LINUX
#ifdef PLAT_WINDOWS
    #define BTA_CALLCONV __stdcall
    // for matlab compatibility
    #define BTA_PRAGMA_ALIGN __declspec(align(BTA_CONFIG_STRUCT_STRIDE))
    #ifdef COMPILING_DLL
        #define DLLEXPORT __declspec(dllexport)
    #else
        #define DLLEXPORT __declspec(dllimport)
    #endif
#elif defined PLAT_LINUX
    //must be empty
    #define DLLEXPORT
    //must be empty
    #define BTA_CALLCONV
    #define BTA_PRAGMA_ALIGN
#else
    #error "Please define PLAT_WINDOWS or PLAT_LINUX"
#endif


#define BTA_Handle unsigned long

/**     @brief Callback function to report on informative events
*              The implementation of this function in the application must copy the relevant data and return immediately
*              For information on the generation of infoEvents, please refer to the Bluetechnix support wiki
*              The parameter 'verbosity' in BTA_Config can be used to turn on/off certain events.
*       @param eventId An identifier for the event that the library is reporting on
*       @param msg A string containing the information for the user        */
typedef void (BTA_CALLCONV *FN_BTA_InfoEvent)(BTA_EventId eventId, int8_t *msg);

/**     @brief Callback function to report on informative events
*              The implementation of this function in the application must copy the relevant data and return immediately
*              For information on the generation of infoEvents, please refer to the Bluetechnix support wiki
*              The parameter 'verbosity' in BTA_Config can be used to turn on/off certain events.
*       @param handle The handle as identification for the device
*       @param eventId An identifier for the event that the library is reporting on
*       @param msg A string containing the information for the user        */
typedef void (BTA_CALLCONV *FN_BTA_InfoEventEx)(BTA_Handle handle, BTA_EventId eventId, int8_t *msg);

/**     @brief Callback function to report on data frames from the sensor
*              The implementation of this function in the application must copy the relevant data and return immediately
*              The BTA_Frame may NOT be altered!
*              Do not call BTAfreeFrame on frame, because it is free'd in the lib
*       @param frame A pointer to the structure containing the data frame        */
typedef void (BTA_CALLCONV *FN_BTA_FrameArrived)(BTA_Frame *frame);

/**     @brief Callback function to report on data frames from the sensor
*              The implementation of this function in the application must copy the relevant data and return immediately
*              The BTA_Frame may NOT be altered!
*       @param handle The handle as identification for the device
*       @param frame A pointer to the structure containing the data frame        */
typedef void (BTA_CALLCONV *FN_BTA_FrameArrivedEx)(BTA_Handle handle, BTA_Frame *frame);

/**     @brief Enumerator with queuing modes        */
typedef enum {
    BTA_QueueModeDoNotQueue       = 0,                  //<<< No queueing
    BTA_QueueModeDropOldest       = 1,                  //<<< Before an overflow, the oldest item in the queue is removed
    BTA_QueueModeDropCurrent      = 2                   //<<< When full, the queue remains unchanged
} BTA_QueueMode;


#define BTA_CONFIG_STRUCT_STRIDE 8
/**     @brief Configuration structure to be passed with BTAopen        */
typedef struct {
    BTA_PRAGMA_ALIGN uint8_t *udpDataIpAddr;                //<<< The IP address for the UDP data interface (The address the device is configured to stream to)
    BTA_PRAGMA_ALIGN uint8_t udpDataIpAddrLen;              //<<< The length of udpDataIpAddr buffer in [byte]
    BTA_PRAGMA_ALIGN uint16_t udpDataPort;                  //<<< The port for the UDP data interface (The port the device is configured to stream to)
    BTA_PRAGMA_ALIGN uint8_t *udpControlOutIpAddr;          //<<< The IP address for the UDP control interface (outbound connection) (The address to send the command to, where the device awaits commands at)
    BTA_PRAGMA_ALIGN uint8_t udpControlOutIpAddrLen;        //<<< The length of udpControlOutIpAddr buffer in [byte]
    BTA_PRAGMA_ALIGN uint16_t udpControlOutPort;            //<<< The port for the UDP control interface (outbound connection) (The address to send the command to, where the device awaits commands at)
    BTA_PRAGMA_ALIGN uint8_t *udpControlInIpAddr;           //<<< The IP address for the UDP control interface (inbound connection) (The address the device should answer to, usually the local IP address)
    BTA_PRAGMA_ALIGN uint8_t udpControlInIpAddrLen;         //<<< The length of udpControlInIpAddr buffer in [byte]
    BTA_PRAGMA_ALIGN uint16_t udpControlInPort;             //<<< The port for the UDP control interface (inbound connection) (The port the device should answer to)
    BTA_PRAGMA_ALIGN uint8_t *tcpDeviceIpAddr;              //<<< The IP address for the TCP data and control interface (The device's IP address)
    BTA_PRAGMA_ALIGN uint8_t tcpDeviceIpAddrLen;            //<<< The length of tcpDeviceIpAddr buffer in [byte]
    BTA_PRAGMA_ALIGN uint16_t tcpDataPort;                  //<<< The port for the TCP data interface (The port the device sends data to)
    BTA_PRAGMA_ALIGN uint16_t tcpControlPort;               //<<< The port for the TCP control interface (The port the device awaits commands at)
    
    BTA_PRAGMA_ALIGN int8_t *uartPortName;                  //<<< The port name of the UART to use
    BTA_PRAGMA_ALIGN uint32_t uartBaudRate;                 //<<< The UART baud rate
    BTA_PRAGMA_ALIGN uint8_t uartDataBits;                  //<<< The number of UART data bits used
    BTA_PRAGMA_ALIGN uint8_t uartStopBits;                  //<<< 0: None, 1: One, 2: Two, 3: 1.5 stop bits
    BTA_PRAGMA_ALIGN uint8_t uartParity;                    //<<< 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space Parity
    BTA_PRAGMA_ALIGN uint8_t uartTransmitterAddress;        //<<< The source address for UART communications
    BTA_PRAGMA_ALIGN uint8_t uartReceiverAddress;           //<<< The target address for UART communications

    BTA_PRAGMA_ALIGN uint32_t serialNumber;                 //<<< Serial number of device to be opened (0 == not specified)

    BTA_PRAGMA_ALIGN uint8_t *calibFileName;                //<<< Name of the calibration file to be loaded into SDK
    BTA_PRAGMA_ALIGN uint8_t *zFactorsFileName;             //<<< Name of the file to be loaded into SDK. It's elements are then multiplied with the Z channel
    BTA_PRAGMA_ALIGN uint8_t *wigglingFileName;

    BTA_PRAGMA_ALIGN BTA_FrameMode frameMode;               //<<< Frame mode to be set in SDK/device
    
    BTA_PRAGMA_ALIGN FN_BTA_InfoEvent infoEvent;            //<<< Callback function pointer to the function to be called upon an informative event (optional but handy for debugging/tracking) (deprecated, use infoEventEx)
    BTA_PRAGMA_ALIGN FN_BTA_InfoEventEx infoEventEx;        //<<< Callback function pointer to the function to be called upon an informative event (optional but handy for debugging/tracking)
    BTA_PRAGMA_ALIGN uint8_t verbosity;                     //<<< A value to tell the library when and when not to generate InfoEvents (0: Only critical events, 10: Maximum amount of events)
    BTA_PRAGMA_ALIGN FN_BTA_FrameArrived frameArrived;      //<<< Callback function pointer to the function to be called when a frame is ready (optional) (deprecated, use frameArrivedEx)
    BTA_PRAGMA_ALIGN FN_BTA_FrameArrivedEx frameArrivedEx;  //<<< Callback function pointer to the function to be called when a frame is ready (optional)
    
    BTA_PRAGMA_ALIGN uint16_t frameQueueLength;             //<<< The library queues this amount of frames internally
    BTA_PRAGMA_ALIGN BTA_QueueMode frameQueueMode;          //<<< The frame queue configuration parameter

} BTA_Config;

/**     @brief This struct is used for the representation of the BTA_Config struct.
               Programming languages that don't use header files are able to query the elements of BTA_Config generically    */
typedef struct {
    char *variableName;                                //<<< The name of the field in the BTA_Config
    uint8_t pointer;                                   //<<< 1 --> field is a pointer, 0 --> field is not a pointer
} BTA_ConfigStructOrg;

/**     @brief This struct contains information about the BTA_Config struct.    */
DLLEXPORT extern BTA_ConfigStructOrg btaConfigStructOrg[];
/**     @brief The size of the struct containing information about the BTA_Config struct.    */
DLLEXPORT extern const uint32_t btaConfigStructOrgLen;

#ifdef __cplusplus
extern "C"
{
#endif

    
/**     @brief With this function a literal description of the config struct can be retrieved.
*              Programming languages which do not support header files it can be used to construct the BTA_Config
*       @param fieldCount The number of elements (config struct fields) in the result on return         */
DLLEXPORT BTA_ConfigStructOrg *BTA_CALLCONV BTAgetConfigStructOrg(uint32_t *fieldCount, uint8_t *bytesPerField);

/* -------------------------------------------------------------------------- */

/**     @brief For querying API version
*       @param buildDateTime A char array allocated by the caller containing the date/time string of build on return.
*       @param buildDateTimeLen Size of the preallocated buffer behind buildDateTime in [byte]
*       @param supportedDeviceTypes Array allocated by the caller containing the codes of all devices supported by a specifiy BTA implementation on return
*       @param supportedDeviceTypesLen Pointer to size of supportedDeviceTypes (the number of supported devices)
*                                      On return it contains number of supported device types
*       @return Please refer to the Bluetechnix support wiki                   */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetVersion(uint32_t *verMaj, uint32_t *verMin, uint32_t *verNonFun, uint8_t *buildDateTime, uint32_t buildDateTimeLen, uint16_t *supportedDeviceTypes, uint32_t *supportedDeviceTypesLen);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Callback function to report on a discovered device
*              Do not call BTAfreeDeviceInfo on deviceInfo, because it is free'd in the lib
*       @param deviceInfo A struct containing information about the discovered device        */
typedef void (BTA_CALLCONV *FN_BTA_DeviceFound)(BTA_DeviceInfo *deviceInfo);

/**     @brief Starts the discovery of devices.
*              If possible, broadcast messages are transmitted otherwise all possible connections are tested
*       @param discoveryConfig Parameters on how to perform the discovery
*                              The library defines which parameters have to be set in BTA_DiscoveryConfig.
*       @param deviceFound This function is called when a device is discovered
*       @return Please refer to the Bluetechnix support wiki                   */
DLLEXPORT BTA_Status BTA_CALLCONV BTAstartDiscovery(BTA_DiscoveryConfig *discoveryConfig, FN_BTA_DeviceFound deviceFound);

/**     @brief Stops the discovery of devices.
*       @return Please refer to the Bluetechnix support wiki            */
DLLEXPORT BTA_Status BTA_CALLCONV BTAstopDiscovery();

//----------------------------------------------------------------------------------------------------------------

/**     @brief Fills the config structure with standard values
*       @param config Pointer to the structure to be initialized to standard values
*       @return Please refer to the Bluetechnix support wiki      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAinitConfig(BTA_Config *config);

/**     @brief  Starts the service and connects to the device
*               If the connection can not be fully established, the function returns an error
*       @param config Pointer to the previously initialized config structure
*       @param handle Pointer containing the handle to the device on return
*       @return Please refer to the Bluetechnix support wiki                                          */
DLLEXPORT BTA_Status BTA_CALLCONV BTAopen(BTA_Config *config, BTA_Handle *handle);

/**     @brief Disconnects from the sensor and stops the service            
*       @param handle Pointer to the handle of the device to be closed
*       @return Please refer to the Bluetechnix support wiki        */
DLLEXPORT BTA_Status BTA_CALLCONV BTAclose(BTA_Handle *handle);

/**     @brief For querying information about the device
*              If successful, BTAfreeDeviceInfo must be called afterwards
*       @param handle Handle of the device to be used
*       @param deviceInfo Pointer to pointer to structure with information about the device on return      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetDeviceInfo(BTA_Handle handle, BTA_DeviceInfo **deviceInfo);

/**     @brief For freeing the device information structure
*       @param deviceInfo Pointer to structure to be freed      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAfreeDeviceInfo(BTA_DeviceInfo *deviceInfo);

/**     @brief Queries whether the service is running or not
*       @param handle Handle of the device to be used
*       @return 1 if the service is running, 0 otherwise                       */
DLLEXPORT uint8_t BTA_CALLCONV BTAisRunning(BTA_Handle handle);

/**     @brief Queries whether the library has a valid connection to the sensor
*       @param handle Handle of the device to be used
*       @return 1 if connected to the sensor, 0 otherwise                       */
DLLEXPORT uint8_t BTA_CALLCONV BTAisConnected(BTA_Handle handle);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Sets the interval of keep-alive-messages.
*              The library takes responsibility of sending a keep-alive-message if there is no
*              communication during <interval> seconds.
*       @param handle Handle of the device to be used
*       @param interval The interval for sending keep-alive-messages
*       @return Please refer to the Bluetechnix support wiki                       */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsetKeepAliveMsgInterval(BTA_Handle handle, float interval);

/**     @brief Sets whether the control interface should use a CRC checksum (if device supports it)
*       @param handle Handle of the device to be used
*       @param enabled 1: enable  0: disable
*       @return Please refer to the Bluetechnix support wiki          */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsetControlCrcEnabled(BTA_Handle handle, uint8_t enabled);

//----------------------------------------------------------------------------------------------------------------

/**     @brief  Allows to set a BTA_FrameMode
*               The device and/or the SDK is configured, so that the desired channels are included
*               in a BTA_Frame when retrieving frames from the SDK
*       @param handle Handle of the device to be used
*       @param frameMode The desired frame-mode
*       @return Please refer to the Bluetechnix support wiki                */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsetFrameMode(BTA_Handle handle, BTA_FrameMode frameMode);

//----------------------------------------------------------------------------------------------------------------


/**     @brief Helper function to clone/duplicate/deep-copy a BTA_Frame structure
*              If successful, BTAfreeFrame must be called on frameDst afterwards
*       @param frameSrc The pointer to the frame to be copied
*       @param frameDst The pointer to the new duplicated frame
*       @return Please refer to the Bluetechnix support wiki        */
DLLEXPORT BTA_Status BTA_CALLCONV BTAcloneFrame(BTA_Frame *frameSrc, BTA_Frame **frameDst);

/**     @brief Actively requests a frame
*              The device-specific library implementation defines if 
*               - it triggers the capturing of a new frame,
*               - it waits for the next frame,
*               - it returns a queued frame or
*               - it's not implemented at all
*              If successful, BTAfreeFrame must be called afterwards
*       @param  handle Handle of the device to be used
*       @param frame Pointer to frame or null on return (needs to be freed with BTAfreeFrame)
*       @param timeout Timeout to wait if no frame is yet available in [ms]
*                      If timeout == 0 the function waits endlessly for a frame from the device.
*       @return Please refer to the Bluetechnix support wiki                                    */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetFrame(BTA_Handle handle, BTA_Frame **frame, uint32_t timeout);

/**     @brief Helper function to free a BTA_Frame structure
*       @param frame The pointer to the frame to be freed
*       @return Please refer to the Bluetechnix support wiki        */
DLLEXPORT BTA_Status BTA_CALLCONV BTAfreeFrame(BTA_Frame **frame);


/**     @brief  Helper function to convert a BTA_Frame structure into a serialized stream
*               Helpful for recording frames to files
*       @param frame The pointer to the frame to be serialized
*       @param frameSerialized Buffer to contain the frame data on return. Must be allocated by the caller.
*       @param frameSerializedLen Pointer to length of the buffer frameSerialized allocated by the caller.
*                                 Pointer to the actual length of the serialized frame on return.
*       @return Please refer to the Bluetechnix support wiki        */
DLLEXPORT BTA_Status BTA_CALLCONV BTAserializeFrame(BTA_Frame *frame, uint8_t *frameSerialized, uint32_t *frameSerializedLen);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Convenience function for extracting distances from a provided frame
*              It simply returns the pointer and copies some information. The same data can be accessed directly going through the BTA_Frame structure
*              If there is no channel with distance data present in the frame, an error is returned
*       @param distBuffer Pointer to the distances on return (null on error)
*       @param dataFormat Pointer to the BTA_DataFormat, thus how to parse 'distBuffer'
*       @param unit Pointer to BTA_Unit, thus how to interpret 'distBuffer'
*       @param xRes Pointer to the number of columns of 'distBuffer'
*       @param yRes Pointer to the number of rows of 'distBuffer'
*       @return Please refer to the Bluetechnix support wiki      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetDistances(BTA_Frame *frame, void **distBuffer, BTA_DataFormat *dataFormat, BTA_Unit *unit, uint16_t *xRes, uint16_t *yRes);

/**     @brief Convenience function for extracting amplitudes from a provided frame
*              It simply returns the pointer and copies some information. The same data can be accessed directly going through the BTA_Frame structure
*              If there is no channel with amplitude data present in the frame, an error is returned
*       @param ampBuffer Pointer to the amplitudes on return (null on error)
*       @param dataFormat Pointer to the BTA_DataFormat, thus how to parse 'ampBuffer'
*       @param unit Pointer to BTA_Unit, thus how to interpret 'ampBuffer'
*       @param xRes Pointer to the number of columns of 'ampBuffer'
*       @param yRes Pointer to the number of rows of 'ampBuffer'
*       @return Please refer to the Bluetechnix support wiki      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetAmplitudes(BTA_Frame *frame, void **ampBuffer, BTA_DataFormat *dataFormat, BTA_Unit *unit, uint16_t *xRes, uint16_t *yRes);

/**     @brief Convenience function for extracting flags from a provided frame
*              It simply returns the pointer and copies some information. The same data can be accessed directly going through the BTA_Frame structure
*              If there is no channel with flag data present in the frame, an error is returned
*       @param flagBuffer Pointer to the flags on return (null on error)
*       @param dataFormat Pointer to the BTA_DataFormat, thus how to parse 'flagBuffer'
*       @param unit Pointer to BTA_Unit, thus how to interpret 'flagBuffer'
*       @param xRes Pointer to the number of columns of 'flagBuffer'
*       @param yRes Pointer to the number of rows of 'flagBuffer'
*       @return Please refer to the Bluetechnix support wiki      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetFlags(BTA_Frame *frame, void **flagBuffer, BTA_DataFormat *dataFormat, BTA_Unit *unit, uint16_t *xRes, uint16_t *yRes);

/**     @brief Convenience function for extracting the 3D-coordinates from a provided frame
*              It simply returns the pointer and copies some information. The same data can be accessed directly going through the BTA_Frame structure
*              If there are not 3 channels with coordinate data present in the sensor data, an error is returned
*       @param frame Pointer to the frame containing the data
*       @param xBuffer A pointer to the cartesian x coordinates on return (null on error)
*       @param yBuffer A pointer to the cartesian y coordinates on return (null on error)
*       @param zBuffer A pointer to the cartesian z coordinates on return (null on error)
*       @param dataFormat Pointer to the BTA_DataFormat, thus how to parse 'xBuffer', 'yBuffer' and 'zBuffer'
*       @param unit Pointer to BTA_Unit, thus how to interpret 'xBuffer', 'yBuffer' and 'zBuffer'
*       @param xRes Pointer to the number of columns of 'xBuffer', 'yBuffer' and 'zBuffer'
*       @param yRes Pointer to the number of rows of 'xBuffer', 'yBuffer' and 'zBuffer'
*       @return Please refer to the Bluetechnix support wiki     */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetXYZcoordinates(BTA_Frame *frame, void **xBuffer, void **yBuffer, void **zBuffer, BTA_DataFormat *dataFormat, BTA_Unit *unit, uint16_t *xRes, uint16_t *yRes);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Facilitates setting the integration time for the default capture sequence
*       @param handle Handle of the device to be used
*       @param integrationTime The desired integration time
*       @return Please refer to the Bluetechnix support wiki                      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsetIntegrationTime(BTA_Handle handle, uint32_t integrationTime);

/**     @brief Facilitates the retrieval of the current integration time of the default capture sequence
*       @param handle Handle of the device to be used
*       @param integrationTime The desired integration time
*       @return Please refer to the Bluetechnix support wiki       */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetIntegrationTime(BTA_Handle handle, uint32_t *integrationTime);

/**     @brief Facilitates setting the frame rate for the default capture sequence
*       @param handle Handle of the device to be used
*       @param frameRate The desired frame rate
*       @return Please refer to the Bluetechnix support wiki                      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsetFrameRate(BTA_Handle handle, float frameRate);

/**     @brief Facilitates the retrieval of the current theoretical frame rate of the default capture sequence
*       @param handle Handle of the device to be used
*       @param frameRate Pointer containing the frame rate on return
*       @return Please refer to the Bluetechnix support wiki                       */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetFrameRate(BTA_Handle handle, float *frameRate);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Function for setting the distance offset being applied to all pixels equally
*       @param handle Handle of the device to be used
*       @param globalOffset offset in [mm]      
*       @return Please refer to the Bluetechnix support wiki                */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsetGlobalOffset(BTA_Handle handle, float globalOffset);


/**     @brief Function for getting the distance offset being applied to all pixels equally
*       @param handle Handle of the device to be used
*       @param globalOffset Pointer to hold offset in mm      
*       @return Please refer to the Bluetechnix support wiki                */
DLLEXPORT BTA_Status BTA_CALLCONV BTAgetGlobalOffset(BTA_Handle handle, float *globalOffset);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Reads registers from the device/SDK
*       @param handle Handle of the device to be used
*       @param address The address of the first register to read from
*       @param data Pointer to buffer allocated by the caller. Contains register data on return
*                   The data in the buffer on return consists of one or more register values, each 4 bytes wide
*       @param registerCount Pointer to the number of registers to be read.
*                            On return, if not null, it contains the number of registers actually read
*                            If null is passed, one register is read
*       @return BTA_StatusOk is returned if all requested registers were read. For error codes Please refer to the Bluetechnix support wiki                       */
DLLEXPORT BTA_Status BTA_CALLCONV BTAreadRegister(BTA_Handle handle, uint32_t address, uint32_t *data, uint32_t *registerCount);

/**     @brief Writes registers to the device/SDK
*       @param handle Handle of the device to be used
*       @param address The address of the first register to write to
*       @param data Pointer to buffer containing register data to be written
*                   The data in the buffer consists of one or more register values, each 4 bytes wide
*       @param registerCount Pointer which contains the number of registers to be written.
*                            On return, if not null, it contains the number of registers actually written
*                            If null is passed, one register is written
*       @return BTA_StatusOk is returned if all requested bytes were written. For error codes Please refer to the Bluetechnix support wiki                      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAwriteRegister(BTA_Handle handle, uint32_t address, uint32_t *data, uint32_t *registerCount);

//----------------------------------------------------------------------------------------------------------------

/**     @brief Initiates a reset of the device
*       @param  handle Handle of the device to be used
*       @return Please refer to the Bluetechnix support wiki                                      */
DLLEXPORT BTA_Status BTA_CALLCONV BTAsendReset(BTA_Handle handle);

//----------------------------------------------------------------------------------------------------------------

/**     @brief  Callback function to report status and progress during transfer and programming
*       @param  status Please refer to the Bluetechnix support wiki
*       @param  percentage Contains the progress in [%]
*                              0: File transfer started (can only be reported once per file transfer)
*                            100: File transfer finished (can only be reported once per file transfer)                    */
typedef int (BTA_CALLCONV *FN_BTA_ProgressReport)(BTA_Status status, uint8_t percentage);

/**     @brief  Allows the sending of large data to the device
*               Mainly this is used for sending calibration data and firmware updates.
*               This function handles the complete transfer of the file and blocks during transmission
*               If possible, this function performs its task in a way that allows the OS to
*               regain control periodically, i.e. it uses blocking functions in a fine granularity.
*               The callback function is called (if not null) in the following cases:
*               - An error occurs -> report <error> with any percentage
*               - An transmission starts -> report BTA_StatusOk with percentage 0%
*               - An transmission ends -> report BTA_StatusOk with percentage 100%
*               (--> so the callback will always be used at least once and at least twice in case of success)
*               During transmission, progress is reported repeatedly when possible
*       @param  handle Handle of the device to be used
*       @param  flashUpdateConfig Contains the data and all the necessary information for handling it
*       @param  progressReport Callback function for reporting the status and progress during transfer and programming
*       @return Please refer to the Bluetechnix support wiki                                     */
DLLEXPORT BTA_Status BTA_CALLCONV BTAflashUpdate(BTA_Handle handle, BTA_FlashUpdateConfig *flashUpdateConfig, FN_BTA_ProgressReport progressReport);

/**     @brief  Convenience function for doing a firmware update. Uses BTAflashUpdate() internally. 
*       @param  handle Handle of the device to be used.
*       @param  filename Name of the firmware binary file.
*       @param  progressReport Callback function for reporting the status and progress during transfer and programming
*       @return Please refer to the Bluetechnix support wiki                                     */
DLLEXPORT BTA_Status BTA_CALLCONV BTAfirmwareUpdate(BTA_Handle handle, const uint8_t *filename, FN_BTA_ProgressReport progressReport);

/**     @brief  Writes the current configuration (i.e. register settings) to non-volatile memory. 
*       @param  handle Handle of the device to be used.
*       @return Please refer to the Bluetechnix support wiki                                     */
DLLEXPORT BTA_Status BTA_CALLCONV BTAwriteCurrentConfigToNvm(BTA_Handle handle);

/**     @brief  Restores the default configuration (but does not save it). 
*       @param  handle Handle of the device to be used.
*       @return Please refer to the Bluetechnix support wiki                                     */
DLLEXPORT BTA_Status BTA_CALLCONV BTArestoreDefaultConfig(BTA_Handle handle);


//----------------------------------------------------------------------------------------------------------------


/**     @brief  A convenience function to convert a BTA_Status into a string
*       @param status The BTA_Status to be converted into a string
*       @param statusString A buffer allocated by the caller to contain the result on return
*       @param statusStringLen The length of the preallocated buffer in statusString
*       @return Please refer to the Bluetechnix support wiki                                     */
DLLEXPORT BTA_Status BTA_CALLCONV BTAstatusToString(BTA_Status status, char* statusString, uint16_t statusStringLen);

/**     @brief  A convenience function to convert a BTA_EventId into a string
*       @param eventId The BTA_EventId to be converted into a string
*       @param eventIdString A buffer allocated by the caller to contain the result on return
*       @param eventIdStringLen The length of the preallocated buffer in statusString
*       @return Please refer to the Bluetechnix support wiki                                     */
DLLEXPORT BTA_Status BTA_CALLCONV BTAeventIdToString(BTA_EventId eventId, char *eventIdString, uint16_t eventIdStringLen);

#ifdef __cplusplus
}
#endif

#endif
