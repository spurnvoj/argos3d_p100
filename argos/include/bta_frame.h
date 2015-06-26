/**  @file bta_frame.h
*    @version 1.3.0
*  
*    @brief This header file contains enums and structs dor the representation of a BTA_Frame
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

#ifndef BTA_FRAME_H_INCLUDED
#define BTA_FRAME_H_INCLUDED

#include "bta_status.h"
#include <stdint.h>


/**     @brief Enumerator with valid frame modes to be passed with BTAsetFrameMode        */
typedef enum {
    BTA_FrameModeCurrentConfig    = 0,    //<<< The sensors settings are not changed and data is passed through
    BTA_FrameModeDistAmp          = 1,
    BTA_FrameModeDistAmpFlags     = 2,
    BTA_FrameModeXYZ              = 3,
    BTA_FrameModeXYZAmp           = 4,
    BTA_FrameModeDistAmpColor     = 5,
    BTA_FrameModeXYZAmpFlags      = 6,
    BTA_FrameModeRawPhases        = 7,
    BTA_FrameModeIntensities      = 8 
} BTA_FrameMode;

/*      @brief Enumerator with channel IDs. They allow the identification of the various channels in a BTA_Frame   */
typedef enum {
    BTA_ChannelIdUnknown             = 0x0,
    BTA_ChannelIdDistance            = 0x1,
    BTA_ChannelIdAmplitude           = 0x2,
    BTA_ChannelIdX                   = 0x4,
    BTA_ChannelIdY                   = 0x8,
    BTA_ChannelIdZ                   = 0x10,
    BTA_ChannelIdConfidence          = 0x20,         //<<< in percent
    BTA_ChannelIdFlags               = 0x40,         //<<< additional pixel-wise sensor specific information
    BTA_ChannelIdPhase0              = 0x80,
    BTA_ChannelIdPhase90             = 0x100,
    BTA_ChannelIdPhase180            = 0x200,
    BTA_ChannelIdPhase270            = 0x400,
    BTA_ChannelIdTest                = 0x800,
    BTA_ChannelIdColor               = 0x1000,
    BTA_ChannelIdAmbient             = 0x2000,
    BTA_ChannelIdPhase               = 0x4000,
    BTA_ChannelIdGrayScale           = 0x8000
} BTA_ChannelId;

/*      @brief Enumerator with data formats. They allow the parsing of a channel in BTA_Frame
*              The lowbyte stands for width (number of bytes)
*              The highbyte stands for unsigned/signed/floating-point (continuous numbering)         */
typedef enum {
    BTA_DataFormatUnknown         = 0x0,
    BTA_DataFormatUInt8           = 0x11,
    BTA_DataFormatUInt16          = 0x12,
    BTA_DataFormatUInt24          = 0x13,
    BTA_DataFormatUInt32          = 0x14,
    BTA_DataFormatSInt8           = 0x21,
    BTA_DataFormatSInt16          = 0x22,
    BTA_DataFormatSInt24          = 0x23,
    BTA_DataFormatSInt32          = 0x24,
    BTA_DataFormatFloat8          = 0x31,
    BTA_DataFormatFloat16         = 0x32,
    BTA_DataFormatFloat24         = 0x33,
    BTA_DataFormatFloat32         = 0x34,
    BTA_DataFormatRgb565          = 0x42
} BTA_DataFormat;

/*      @brief Enumerator with units. Allows the interpretation of the data in a channel    */
typedef enum {
    BTA_UnitUnitLess        = 0,
    BTA_UnitMeter           = 1,
    BTA_UnitCentimeter      = 2,
    BTA_UnitMillimeter      = 3,
    BTA_UnitPercent         = 4
} BTA_Unit;

/*      @brief BTA_Channel holds a two-dimensional array of data. A BTA_Frame holds one or more channels.   */
typedef struct {
    BTA_ChannelId id;                 //<<< Type of data in this channel
    uint16_t xRes;                    //<<< Number of columns
    uint16_t yRes;                    //<<< Number of rows
    BTA_DataFormat dataFormat;        //<<< The bytestream in data needs to be casted to this format
    BTA_Unit unit;                    //<<< Informative, for easier interpretation of the data
    uint32_t integrationTime;         //<<< Integration time at which the frame was captured in [us]
    uint32_t modulationFrequency;     //<<< Modulation frequency at which the frame was captured in [Hz]
    uint8_t *data;                    //<<< Pixels starting with upper left pixel. For nofBytesPerPixel > 1 the first byte is the lowbyte
} BTA_Channel;

/*      @brief BTA_Frame holds all the data gathered from one frame.
               Besides information on the capturing it holds one or more channels.      */
typedef struct {
    uint8_t firmwareVersionNonFunc;   //<<< Firmware version non functional
    uint8_t firmwareVersionMinor;     //<<< Firmware version minor
    uint8_t firmwareVersionMajor;     //<<< Firmware version major
    float mainTemp;                   //<<< Main-board/processor temperature sensor in degree Celcius
    float ledTemp;                    //<<< Led-board temperature sensor in degree Celcius
    float genericTemp;                //<<< Additional Generic temperature sensor in degree Celcius
    uint32_t frameCounter;            //<<< Consecutive numbering of frames
    uint32_t timeStamp;               //<<< Time-stamp at which the frame was captured (in microseconds)
    BTA_Channel **channels;           //<<< Data containing ucNofChannels Channel structs
    uint8_t channelsLen;
} BTA_Frame;

#endif
