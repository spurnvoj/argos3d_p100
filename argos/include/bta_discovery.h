/**  @file bta_discovery.h
*    @version 1.3.0
*  
*    @brief This header file contains enums and structs regarding discovery functions and device identification
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

#ifndef BTA_DISCOVERY_H_INCLUDED
#define BTA_DISCOVERY_H_INCLUDED


/*  @brief All device types currently known to the SDK    */
typedef enum {
    //products
    BTA_DeviceTypeArgos3dP100 = 0xa3c4,
    BTA_DeviceTypeArgos3dP310 = 0x9ba6,
    BTA_DeviceTypeArgos3dP320 = 0xb320,
    BTA_DeviceTypeSentisTofP510 = 0x5032,
    //BTA_DeviceTypeEPC610TofModule = 0x7a3d,
    BTA_DeviceTypeSentisTofM100 = 0xa9c1,
    BTA_DeviceTypeTimUp19kS3Spartan6 = 0x13ab,
    BTA_DeviceTypeTimUp19kS3Eth = 0x795c,
    BTA_DeviceTypeTimUpOPT8140 = 0x5a2b
} BTA_DeviceType;


/*  @brief This structure is used to configure the process of device discovery   */
typedef struct {
    uint8_t *tcpBroadcastIpAddr;      //<<< The broadcast IP address
    uint8_t tcpBroadcastIpAddrLen;
    uint16_t tcpPortStart;            //<<< The first port to scan
    uint16_t tcpPortEnd;              //<<< The last port to scan

    int32_t uartBaudRate;             //<<< The UART baud rate
    uint8_t uartDataBits;             //<<< The number of UART data bits used
    uint8_t uartStopBits;             //<<< 0: None, 1: One, 2: Two, 3: 1.5 stop bits
    uint8_t uartParity;               //<<< 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space Parity
    uint8_t uartTransmitterAddress;   //<<< The source address for UART communications
    uint8_t uartReceiverAddressStart; //<<< The first target address
    uint8_t uartReceiverAddressEnd;   //<<< The last target address
} BTA_DiscoveryConfig;


/*  @brief This structure holds information about the device    */
typedef struct {
    BTA_DeviceType deviceType;          //<<< Two-byte-id for a device or module (independent of hardware and software versions)
    uint8_t *productOrderNumber;        //<<< String containing the PON (not including the serial number) (unique in combination with serial number)
    uint32_t serialNumber;              //<<< Serial number (unique in combination with PON)
    uint32_t firmwareVersionMajor;      //<<< Firmware version major
    uint32_t firmwareVersionMinor;      //<<< Firmware version minor
    uint32_t firmwareVersionNonFunc;    //<<< Firmware version non functional
    //---------------------------------------
    uint8_t *deviceIpAddr;
    uint32_t deviceIpAddrLen;
    uint8_t *subnetMask;                //<<< Subnet in which the device is
    uint32_t subnetMaskLen;
    uint16_t tcpControlPort;
    uint16_t tcpDataPort;
    uint8_t *udpDataIpAddr;
    uint32_t udpDataIpAddrLen;
    uint16_t udpDataPort;
    uint8_t *udpControlIpAddr;
    uint32_t udpControlIpAddrLen;
    uint16_t udpControlPort;
} BTA_DeviceInfo;

#endif
