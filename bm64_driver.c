/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_command_send.c

  Summary:
   BM64 Bluetooth Static Driver source file for sending commands.

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to sending commands to the BM64 module.
 
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END


// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

#include <atmel_start.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "bm64_driver.h"


/*------------------------------------------------------------*/
uint8_t calculateChecksum(uint8_t* startByte, uint8_t* endByte)
{
    uint8_t checksum = 0;
    while(startByte <= endByte)
    {
        checksum += *startByte;
        startByte++;
    }
    checksum = ~checksum + 1;
    return checksum;
}

uint8_t calculateChecksum2(uint8_t checksum, uint8_t* startByte, uint16_t length)
{
    while(length)
    {
        checksum += *startByte++;
        length--;
    }
    return checksum;
}


/*------------------------------------------------------------*/
void DRV_BM64_SendAckToEvent(uint8_t ack_event)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_MCU_SEND_EVENT_ACK_CMD;        //command ID
    command[4] = ack_event;                 //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
}

/*------------------------------------------------------------*/
void DRV_BM64_SendDiscoverableCommand(uint8_t discoverable)
{
    uint8_t command[6];
    command[0] = 0xAA;
    command[1] = 0x00;
    command[2] = 0x02;
    command[3] = DRV_BM64_DISCOVERABLE_CMD;
    command[4] = discoverable;      //0: disable, 1: enable
    command[5] = calculateChecksum(&command[2], &command[4]);
}

/*------------------------------------------------------------*/
void DRV_BM64_ChangeDeviceNameCommand(const char *name)
{
    char ch;
    uint8_t i;
    uint8_t command[38];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x21;                      //length
    command[3] = DRV_BM64_CHANGE_DEVICE_NAME_CMD;         //command ID
    for (i=0; i < 32; i++)
    {
        ch = name[i];
        if (ch=='\0')
        {
            break;
        }
        command[4+i] = name[i];
    }
    for (; i < 32; i++)
    {
        command[4+i] = '\0';
    }    
    command[36] = calculateChecksum(&command[2], &command[35]);
}          

/*------------------------------------------------------------*/
void DRV_BM64_ReadDeviceAddressCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_READ_LOCAL_BD_ADDR_CMD;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
}

/*------------------------------------------------------------*/
void DRV_BM64_ReadDeviceNameCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_READ_LOCAL_DEVICE_NAME_CMD;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
}

/*------------------------------------------------------------*/
void DRV_BM64_LinkBackToDeviceByBTAddress(uint8_t* address)
{
    uint8_t command[14];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 10;                      //length
    command[3] = DRV_BM64_PROFILE_LINK_BACK_CMD;         //command ID
    command[4] = 0x05;              //0x05: link back to device with specified address
    command[5] = 0x00;
    command[6] = 0x07;
    command[7] = *address++;        //byte 0
    command[8] = *address++;        //byte 1
    command[9] = *address++;        //byte 2
    command[10] = *address++;        //byte 3
    command[11] = *address++;        //byte 4
    command[12] = *address++;        //byte 5
    command[13] = calculateChecksum(&command[2], &command[12]);
}

/*------------------------------------------------------------*/
void DRV_BM64_DisconnectAllProfile(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_DISCONNECT_CMD;                //command ID
    command[4] = 0x0f;                      //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
}


/*------------------------------------------------------------*/
//void DRV_BM64_SendSPPData(uint8_t* addr, uint16_t size, uint8_t link_index)
//{
    //uint8_t command[11];
    //uint8_t checksum = 0;
    //uint16_t cmd_length = size + 7;
    //command[0] = 0xAA;                      //header byte 0
    //command[1] = (uint8_t)(cmd_length>>8);                 //header byte 1
    //command[2] = (uint8_t)(cmd_length&0xff);                      //length
    //command[3] = DRV_BM64_SEND_SPP_DATA_CMD;             //command ID
    //command[4] = link_index;                      //link_index, set to 0
    //command[5] = 0x00;          //single packet format
    ////total_length: 2byte
    //command[6] = (uint8_t)(size>>8);
    //command[7]= (uint8_t)(size&0xff);
    ////payload_length: 2byte
    //command[8] = (uint8_t)(size>>8);
    //command[9] = (uint8_t)(size&0xff);
    ////checksum = calculateChecksum2(checksum, &command[1], 9);
////
    ////checksum = calculateChecksum2(checksum, addr, size);
    ////checksum = ~checksum + 1;
//
//}
//
