/********************************************************************
* FileName:     AccessPoint.c
* Dependencies: none
* Processor:    PIC24 (tested)
* Complier:     Microchip C30 v3.23 or higher (tested)
*
* Processor:    PIC18, PIC32, dsPIC30, dsPIC33 (untested)
* Complier:     Microchip C18 v3.04 or higher (untested)
*               Microchip C32 v1.02 or higher
*


*********************************************************************
* File Description:
*
* This program is based off of Microchip's MiWi Development Environment
* and demonstrates the capabilities of Powercast's P2110 Energy Harvester
* product.  This program is for receiving a MiWi packet from the End
* Point node and displaying the sensor information received on a
* computer terminal screen.
*  Updated
*this file is modified by NDSU ECE 209 to collect structure monitoring data
* Change History (Microchip Code):
*  Rev   Date         Author    Description
*  0.1   1/03/2008    yfy       Initial revision
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
*  3.1   5/28/2010    yfy       MiWi DE 3.1
*
* Change History (Powercast Code):
*  Ver   Date         Author    Description
*  1.0   10/14/2010   DWH       Initial revision
*  1.1   04/29/2011   DWH       Updated version for endpoint code revision
* 12    02/15/2017   RG        Updated version for Structure health monitoring
********************************************************************/

/* -- COMPILER DIRECTIVES -- */


/* -- INCLUDES -- */
#include "Console.h"
#include "ConfigApp.h"
#include "HardwareProfile.h"
#include "WirelessProtocols\MCHP_API.h"
#include <math.h>

/* -- DEFINES and ENUMS -- */
#define VBG_VAL             1228800UL   // VBG = 1.2V, VBG_VAL = 1200 mV * 1024 counts
#define T_BIAS              10000.0     // Bias res for temp sensor
#define B_CONST             3380.0      // B Constant of thermistor
#define R_0                 10000.0     // Thermistor resistance at 25C
#define T_0                 298.15      // Temp in kelvin at 25C
#define MYCHANNEL           25
#define CODE_VERSION        12
#define EXT_CHANNEL         2  
#define RX_TIME             1
#define SAMPLE_ADC_VALUE    5
#define MAX_PACKET_SIZE     50          // Maxmium Size in one packet
#define MAX_DATA_SIZE       24          // Maxmium ADC Data Size 
#define UNDEFINED           0
#define HEADERCOOMANDSIZE   2

/* Uncomment for debugging */
//#define DEBUG
enum
{
    SLAVE_0_ID,
    SLAVE_1_ID,
    SLAVE_2_ID,
    SLAVE_3_ID,
    GLOBAL_ID = 0xFF
};
//TODO: EDIT THIS FOR UNIQUE SLAVE DEVICE
#define UNIQUE_SLAVE SLAVE_1_ID


typedef enum
{
    CALC_ADC_CMD,
    REQ_STATUS_CMD
} COMMANDS_E;


enum
{
    SLAVE_ID_INDEX,
    COMMAND_INDEX,
    ADC_VALUEHIGH_INDEX,                        // High bytes
    ADC_VALUELOW_INDEX                          // Low bytes               
};

/* -- TYPEDEFS and STRUCTURES -- */
typedef enum
{
    INACTIVE,
    ADC_CALC,
    SLAVE_RES
} SLAVE_STATES_E;

typedef enum
{
    SLAVE_NO_ACKNOWLEDGE,
    SLAVE_ACKNOWLEDGE
} SLAVE_RES_STATES_E;

/* -- STATIC AND GLOBAL VARIABLES -- */
static int  scu32ADCValue[24]; 
static BYTE scabyResponseBuffer[MAX_PACKET_SIZE];
static BYTE scADCH[24];
static BYTE scADCL[24];
unsigned int DataCount = 0;
/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static int scu32ADCRead(WORD wChannel);
static void scAllocateRespondBuffer(void);



/* This value indicates that the last command
   received from master was processed successfully */
static BOOL scfSlaveStatus;
static SLAVE_STATES_E sceConvertCommandToState(const COMMANDS_E keCommand);


/*----------------------------------------------------------------------------

@Description: The  Application main entry point initialization

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scMainInit(void)
{
    BYTE byI;

    BoardInit();
    
    /*******************************************************************/
    // Function MiApp_ProtocolInit initialize the protocol stack. The
    // only input parameter indicates if previous network configuration
    // should be restored. In this simple example, we assume that the
    // network starts from scratch.
    /*******************************************************************/
        
    SPI1STAT = 0x8000;  // Enable SPI bus to talk radio
    MiApp_ProtocolInit(FALSE);
    // Set default channel
    MiApp_SetChannel(MYCHANNEL);

    /*******************************************************************/
    // Function MiApp_ConnectionMode defines the connection mode. The
    // possible connection modes are:
    //  ENABLE_ALL_CONN:    Enable all kinds of connection
    //  ENABLE_PREV_CONN:   Only allow connection already exists in
    //                      connection table
    //  ENABL_ACTIVE_SCAN_RSP:  Allow response to Active scan
    //  DISABLE_ALL_CONN:   Disable all connections.
    /*******************************************************************/
    MiApp_ConnectionMode(ENABLE_ALL_CONN);

    /* Initialize static variables */
    scu32ADCValue [40] = 0 ;
    scfSlaveStatus = SLAVE_NO_ACKNOWLEDGE;

    for (byI = 0; byI < sizeof(scabyResponseBuffer); byI++)
    {
        scabyResponseBuffer[byI] = UNDEFINED;
    }
}


/*----------------------------------------------------------------------------

@Description: The  Application main entry point

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/12/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
int main(void)
{
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE) {0};
    SLAVE_STATES_E eSlaveStates = INACTIVE;

    scMainInit();

    while(TRUE)
    {
        switch(eSlaveStates)
        {
            case INACTIVE:
                if (scfReceive((RECEIVED_MESSAGE *)&stReceivedMessage))
                {
                    if ((stReceivedMessage.Payload[SLAVE_ID_INDEX] == GLOBAL_ID) ||
                        (stReceivedMessage.Payload[SLAVE_ID_INDEX] == UNIQUE_SLAVE))
                    {
                        eSlaveStates = sceConvertCommandToState((COMMANDS_E)stReceivedMessage.Payload[COMMAND_INDEX]);
                    }
                    else
                    {
                        stReceivedMessage = (RECEIVED_MESSAGE) {0};
                    }
                }

                break;

            case ADC_CALC:                     
                for ( DataCount =0;DataCount<MAX_DATA_SIZE;DataCount++)
                {
                    scu32ADCValue[DataCount] = scu32ADCRead(EXT_CHANNEL); 
                    scADCH[DataCount] = scu32ADCValue[DataCount] >> 8;
                    scADCL[DataCount] = scu32ADCValue[DataCount];
                }
                DataCount = 0;
                scfSlaveStatus = SLAVE_ACKNOWLEDGE;
                eSlaveStates = INACTIVE;
                break;

            case SLAVE_RES: //TODO: more work needed to make generic
                scAllocateRespondBuffer();
                scTransmit((BYTE *)&scabyResponseBuffer, MAX_PACKET_SIZE);
                eSlaveStates = INACTIVE;
                break;

            default:
                // Error case
                eSlaveStates = INACTIVE;
                break;
        }
    }

    return 0;
}


/*----------------------------------------------------------------------------
 
@Prototype: static SLAVE_STATES_E eConvertCommandToState(const BYTE kbyCommand)
 
@Description: Convert a received command from master to a slave state

@Parameters: const COMMANDS_E keCommand - The command from master

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/13/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static SLAVE_STATES_E sceConvertCommandToState(const COMMANDS_E keCommand)
{
    SLAVE_STATES_E eSlaveState = INACTIVE;

    switch (keCommand)
    {
        case CALC_ADC_CMD:
            eSlaveState = ADC_CALC;
            break;

        case REQ_STATUS_CMD:
            eSlaveState = SLAVE_RES;
            break;

        default:
            // Error case
            eSlaveState = INACTIVE;
            break;
    }

    return eSlaveState;
}


/*----------------------------------------------------------------------------
 
@Prototype: static void scTransmit(BYTE * pbyTxBuffer, BYTE byLength)
 
@Description: Request ADC from all slaves

@Parameters: BYTE * pbyTxBuffer - Pointer to the buffer of bytes to transmit 
             BYTE byLength - The length of bytes

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scTransmit(BYTE * pbyTxBuffer, BYTE byLength)
{
    BYTE byI;

    // Clear the transmit buffer
    MiApp_FlushTx();

    // Write new frame to transmit buffer
    for (byI = 0; byI < byLength; byI++)
    {
         MiApp_WriteData(pbyTxBuffer[byI]);
    }

    // Broadcast packet from transmit buffer
    MiApp_BroadcastPacket(FALSE);
}


/*----------------------------------------------------------------------------
 
@Prototype: static BOOL scfReceive(void)
 
@Description: Wait until we receive a packet

@Parameters: RECEIVED_MESSAGE * stReceiveMessageBuffer - Out parameter for the 
             received message. 
 
@Returns: TRUE: If we received a packet, FALSE otherwise
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static BOOL scfReceive(RECEIVED_MESSAGE * stReceiveMessageBuffer)
{
    BOOL fRetVal = FALSE;

    if (MiApp_MessageAvailable())
    {
        * stReceiveMessageBuffer = rxMessage;
        MiApp_DiscardMessage();

        fRetVal = TRUE;
    }

    return fRetVal;
}


/*----------------------------------------------------------------------------
 
@Prototype: static int scu32ADCRead(WORD wADCChannel)
 
@Description: Read an ADC value from a given ADC channel

@Parameters: WORD wADCChannel - ADC Channel 
 
@Returns: int: The ADC value - the microcontroller has 10 bit adc, hence byte is not correct type 
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/12/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static int scu32ADCRead(WORD wADCChannel)
{
    WORD wI = 0;
    int u32ADCVal = SAMPLE_ADC_VALUE;
//    BYTE byADCVal = SAMPLE_ADC_VALUE;

#if 0
    AD1CHS = wADCChannel;           // set channel to measure 
    AD1CON1bits.ADON = 1;        // turn ADC on for taking readings
    for (wI = 0; wI < 50; wI++);
    AD1CON1bits.SAMP = 1;       // start sampling
    while (!AD1CON1bits.DONE);  // wait for ADC to complete
    u32ADCVal = ADC1BUF0;
    AD1CON1bits.ADON = 0;       // turn ADC off for before taking next reading
#endif

    /* TODO: GET RID OF THIS, FOR SIMULATION PURPOSES ONLY */
    (void)wADCChannel;
    DelayMs(50);

    return u32ADCVal;
}
/*----------------------------------------------------------------------------
 
@Prototype: static void scAllocateRespondBuffer(void)
 
@Description: allocate the ADC data to RespondBuffer

@Parameters: None 
 
@Returns: None
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/18/2017       Ruisi Ge           Initial Revision

*----------------------------------------------------------------------------*/
static void scAllocateRespondBuffer(void)
{
    int DataCount;
    scabyResponseBuffer[SLAVE_ID_INDEX] = UNIQUE_SLAVE;
    scabyResponseBuffer[COMMAND_INDEX] = (BYTE)scfSlaveStatus;
    for (DataCount=0;DataCount< MAX_DATA_SIZE;DataCount++)
    {
    scabyResponseBuffer[2*DataCount+HEADERCOOMANDSIZE] = scADCH[DataCount];
    scabyResponseBuffer[2*DataCount+HEADERCOOMANDSIZE+1] = scADCL[DataCount];
    }
}

