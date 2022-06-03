#include <atmel_start.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "bm64_driver.h"


bool checkCHKSUM(void);
void readReset(void);
void setVolume(void);
void batteryRead();
void checkCMD();
void parseCMD(void);
void execute(void);
void sendResponse(void);
void sendSerialNumber(void);
void sendTest();
void timeoutCallback();

volatile struct Parameters {
	volatile uint8_t Mode;																//0-DEVICESTATUS	1-HYPO	2 - NORMAL	3-HYPER
	volatile uint8_t RLedIntensity;
	volatile uint8_t GLedIntensity;
	volatile uint8_t BLedIntensity;
	volatile uint8_t volume;
	volatile uint8_t vibrationIntensity;
	volatile uint8_t batteryStatus;
	volatile uint8_t status;
} parameters;

enum {
	everything_is_fucking_OK,
	lengthError,
	checksumError,
	invalidCMD,
	outOfRange,
	BTModule_misbehaving
};




volatile uint8_t index_read = 0;														//Index for UART RX buffer

uint8_t *read = NULL;																//UART RX buffer


volatile bool received = false;
volatile bool memoryAllocated = false;
volatile bool run = false;
volatile bool tick = false;



int timer0;
int timer1;
int timer2;

int main(void)
{
	
	read = (uint8_t*) calloc(32, sizeof(uint8_t));
	
	memoryAllocated = true;
	received = true;
	//run = true;
	
	read[0] = 0xAA;
	read[1] = 0x00;
	read[2] = 0x0D; //total no of bytes -2
	read[3] = 0x22; //Event OPcode
	read[4] = 0x00; //channel index
	read[5] = 0x00; //packet type
	read[6] = 0x00; //total lenght_H
	read[7] = 0x05; //total lenght_L
	read[8] = 0x00; //payload lenght_H
	read[9] = 0x05; //payload lenght_H
	/************************************************************************/
	/* CMD                                                                  */
	/************************************************************************/
	read[10] = 0x1F; //FLAGS
	read[11] = 0x01; //CMD
	read[12] = 100; //RED LED INTENSTIY
	read[13] = 99; //GREEN LED INTENSTIY
	read[14] = 99; //BUE LED INTENSTIY
	read[15] = 97; //VIBRATION
	
	index_read = 16;
	
	read[index_read] = calculateChecksum(&read[1],&read[index_read-1]);
	
	
	
	/************************************************************************/
	/*                                                                      */
	/************************************************************************/
	
	atmel_start_init();															//MCU initial configuration

	PORTA_OUTCLR = PIN1_bm;
	
	PORTB_OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm;
	
	PORTB_OUTCLR = PIN1_bm;														//GND at pin PC1 for vibration motor testing
	PWM_0_disable_output_ch0();													//initial RED LED turn OFF
	PWM_0_disable_output_ch1();													//initial GREEN LED turn OFF
	PWM_0_disable_output_ch2();													//initial BLUE LED turn OFF
	TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);										//initial turn OFF vibration

	PORTA_PIN7CTRL |= PORT_ISC0_bm | PORT_ISC1_bm;								//PA7 interrupt enable - FALLING edge

	/************************************************************************/
	/*                                                                      */
	/************************************************************************/

	while (1)
	{
		if (received)															//Any received read to check and parse?
		{
			//novyIndex = read[2] + 3;							//position of Checksum Byte
			if (read[read[2] + 3] > 0)									//Check checksum > 0
			{
				
				if (checkCHKSUM())												//Is the checksum correct?
				{
					checkCMD();													//decode received CMD
				}
				else
				{
					readReset();
				}
			}
			

		}
		
		if (run)																//condition for powering effectors
		{
			execute();															//signalization mode execution
			run = false;
		}
	}
	
	
	
	
	
	
	
	
}


bool checkCHKSUM(void){

	
	if (read[read[2] + 3] == calculateChecksum(&read[1],&read[read[2] + 2]))												//compute and compare checksum of received read
	{
		return true;
		}else{
		parameters.status = checksumError;
		return false;
	}
}

void checkCMD(void){
	
	switch (read[3])
	{
		//UART Events
		case 0x00: break;										//Command_ACK
		case 0x01: break;										//BTM_Status
		case 0x02: break;										//Call_Status
		case 0x03: break;										//Caller_ID
		case 0x04: break;										//SMS_Received_Indication
		case 0x05: break;										//Missed_Call_Indication
		case 0x06: break;										//Phone_Max_Battery_Level
		case 0x07: break;										//Phone_Current_Battery_Level
		case 0x08: break;										//Roaming_Status
		case 0x09: break;										//Phone_Max_Signal_Strength_Level
		case 0x0A: break;										//Phone_Current_Signal_Strength_Level
		case 0x0B: break;										//Phone_Service_Status
		case 0x0C: batteryRead(); break;						//BTM_Battery_Status
		case 0x0D: break;										//BTM_Charging_Status
		case 0x0E: break;										//Reset_To_Default
		case 0x0F: break;										//Report_HF_Gain_Level
		case 0x10: break;										//EQ_Mode_Indication
		case 0x17: break;										//Read_Linked_Device_Information_Reply
		case 0x18: break;										//Read_BTM_Version_Reply
		case 0x19: break;										//Call_List_Report
		case 0x1A: break;										//AVC_Specific_Rsp
		case 0x1B: break;										//BTM_Utility_Req
		case 0x1C: break;										//Vendor_AT_Cmd_Reply
		case 0x1D: break;										//Report_Vendor_AT_Event
		case 0x1E: break;										//Read_Link_Status_Reply
		case 0x1F: break;										//Read_Paired_Device_Record_Reply
		case 0x20: break;										//Read_Local_BD_Address_Reply
		case 0x21: break;										//Read_Local_Device_Name_Reply
		case 0x22: parseCMD(); break;							//Report_SPP/iAP_Data
		case 0x23: break;										//Report_Link_Back_Status
		case 0x24: break;										//REPORT_RING_TONE_STATUS
		case 0x25: break;										//User_Confrim_SSP_Req
		case 0x26: break;										//Report_AVRCP_Vol_Ctrl
		case 0x27: break;										//Report_Input_Signal_Level
		case 0x28: break;										//Report_iAP_Info
		case 0x29: setVolume(); break;							//REPORT_AVRCP_ABS_VOL_CTRL
		case 0x2A: break;										//Report_Voice_Prompt_Status
		case 0x2C: break;										//Security_Bonding_Res
		case 0x2D: break;										//Report_Type_Codec
		case 0x2E: break;										//Report_Type_BTM_Setting
		case 0x2F: break;										//Report_MCU_Update_Reply
		case 0x30: break;										//Report_BTM_Initial_Status
		case 0x31: break;										//LE_ANCS_Service_Event
		case 0x32: break;										//LE_Signaling_Event
		case 0x33: break;										//Report_nSPK_Link_Status
		case 0x34: break;										//Report_nSPK_Vendor_Event
		case 0x35: break;										//Report_nSPK_Audio_Setting
		case 0x36: break;										//Report_Sound_Effect_Status
		case 0x37: break;										//Report_Vendor_EEPROM_Data
		case 0x38: break;										//REPORT_IC_VERSION_INFO
		case 0x39: break;										//REPORT_LE_GATT_EVENT
		case 0x3A: break;										//Report_BTM_Link_Mode
		case 0x3B: break;										//DSP_Dedicated_Event
		case 0x3C: break;										//Report_nSPK_MISC_Event
		case 0x3D: break;										//Report_nSPK_Exchange_Link_Info
		case 0x3E: break;										//Report Customized_Information
		case 0x3F: break;										//Report_CSB_CLK
		case 0x40: break;										//Report_Read_Feature_List_Reply
		case 0x41: break;										//REPORT_TEST_RESULT_REPLY
		
		default:
		/* The default keyword specifies the code to run if there is no case match: */
		parameters.status = BTModule_misbehaving;				//BT Module misbehaving
		break;
	}

	readReset();
}

void readReset(void){
	//memset(read, 0, sizeof(read));
	free(read);
	index_read = 0;
	received = false;
	memoryAllocated = false;
}

void setVolume(void){

	parameters.volume = read[5];									//Report in range 0x00~0x7F to indicate the percentage of total(max) volume level
}

void batteryRead(){
	
	parameters.batteryStatus = read[4];
	
	/*
	0x00	dangerous level, and will auto shutdown
	0x01	low level
	0x02	normal level
	0x03	high level
	0x04	full level
	0x05	in charging
	0x06	charging completed
	*/
	
}

void checkBattery(){
	//uint8_t batteryStatus_AA[] = {0x00, 0xAA, 0x00, 0x02, 0x25, 0x00, 0xD8};
	uint8_t command[6];
	command[0] = 0x00;						//start byte
	command[1] = 0xAA;	                    //header byte 0
	command[2] = 0x00;                      //header byte 1
	command[3] = 0x02;                      //length
	command[4] = 0x25;						//command ID
	command[5] = calculateChecksum(&command[3], &command[4]);
	
	for (int i = 0; i < sizeof(command); i++)
	{
		USART_0_write(command[i]);
	}
}

ISR(USART0_RXC_vect){
	if (!memoryAllocated)
	{
		read = (uint8_t*) calloc(32, sizeof(uint8_t));
		memoryAllocated = true;
	}
	if (read[0] == 0xAA)																	//check if there is AA header Byte
	{
		index_read++;
		read[index_read] = USART_0_read();													//reading USART RX into buffer read[index]
		if (!received)																		//one time timeout initiation
		{
			received = true;
			RTC_init();
		}
	}
	else
	{
		read[0] = USART_0_read();															//reading USART RX into header byte
	}
	
	if (index_read > 30)																	//Buffer overflow protection
	{
		readReset();
	}
	
}

ISR(PORTA_PORT_vect){
	
	if ((PORTA_INTFLAGS & PIN7_bm) > 0)														//BUTTON PRESS INTERRUPT PA0
	{
		run	= true;
		PORTA_INTFLAGS |= PIN7_bm;															//clear interrupt flag
	}
}

void parseCMD(void){
	uint8_t position = 10;																	//flag byte offset
	if ((read[10] & 0b00000001) > 0){														//checking flagByte for appropriate flags and setting stored parameters
		position++;
		if(read[position] > 0){
			parameters.Mode = read[position];
			run = true;																			//RUN condition to enable EXECUTE
		}
	}
	if ((read[10] & 0b00000010) > 0){
		position++;
		parameters.RLedIntensity = read[position];
		PWM_0_load_duty_cycle_ch0(parameters.RLedIntensity * 65535 / 100);
	}
	if ((read[10] & 0b00000100) > 0){
		position++;
		parameters.GLedIntensity = read[position];
		PWM_0_load_duty_cycle_ch1(parameters.GLedIntensity * 65535 / 100);
	}
	if ((read[10] & 0b00001000) > 0){
		position++;
		parameters.BLedIntensity = read[position];
		PWM_0_load_duty_cycle_ch2(parameters.BLedIntensity * 65535 / 100);
	}
	if ((read[10] & 0b00010000) > 0){
		position++;
		parameters.vibrationIntensity = read[position];
		TCB0.CCMP = (uint16_t) parameters.vibrationIntensity * 65535 / 100;
	}
	
	//sendResponse();
	sendTest();
}


void execute(void){
	
	uint8_t pulseNo = 0;																	//Number of pulses for different modes
	uint8_t pulseDuration = 0;																//Duration of pulses for different modes
	
	TCA0.SINGLE.CNT = 0x0;																	/* Count: 0x0 */
	TCB0.CNT = 0x0;																			/* Count: 0x0 */
	
	switch (parameters.Mode)
	{
		case 0:
		/* Status report */

		break;
		
		case 1:
		/* Hypoglycemia */
		pulseNo = 3;
		pulseDuration = 50;
		for (int i = 0; i < pulseNo; i++)
		{
			PWM_0_enable_output_ch0();														//turn ON LED PWM
			TCB0.CTRLA |= 1 << TCB_ENABLE_bp;												//turn ON vibration
			_delay_ms(10*pulseDuration);
			PWM_0_disable_output_ch0();														//turn OFF LED PWM
			TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);											//turn OFF vibration
			_delay_ms(1000);
		}
		
		
		break;
		
		case 2:
		/* Normoglycemia */
		pulseNo = 2;
		pulseDuration = 10;
		
		
		
		break;
		
		case 3:
		/* Hyperglycemia */
		pulseNo = 2;
		pulseDuration = 100;
		
		for (int i = 0; i < pulseNo; i++)
		{
			PWM_0_enable_output_ch2();														//turn ON LED PWM
			TCB0.CTRLA |= 1 << TCB_ENABLE_bp;												//turn ON vibration
			_delay_ms(10*pulseDuration);
			PWM_0_disable_output_ch2();														//turn OFF LED PWM
			TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);											//turn OFF vibration
			_delay_ms(1000);
			
			
		}
		
		break;
		
		case 4:
		/* Alarm */
		
		while (true)
		{
			PWM_0_enable_output_ch0();														//turn ON LED PWM
			PORTA_OUTSET = PIN7_bm;															//turn ON vibration
			_delay_ms(500);
			PWM_0_disable_output_ch0();														//turn OFF LED PWM
			PORTA_OUTCLR = PIN7_bm;															//turn OFF vibration
			_delay_ms(500);
		}
		
		
		
		
		break;
		
		default:
		parameters.status = invalidCMD;
		break;
	}
}


/************************************************************************/
/* Serial number retrieval from MCU + send to User via USART (Bluetooth)*/
/************************************************************************/

void sendResponse(){
	
	uint8_t command[17];
	command[0] = 0xAA;																		//header byte 0
	command[1] = 0x00;																		//header byte 1
	command[2] = 0x02;																		//length
	command[3] = DRV_BM64_SEND_SPP_DATA_CMD;													//command ID
	command[4] = 0x00;																		//channel index
	command[5] = 0x00;																		//single packet format
	command[6] = 0x00;																		//TOTAL length HIGH
	command[7] = 0x06;																		//TOTAL LENGTH LOW
	command[8] = 0x00;																		//packet length HIGH
	command[9] = 0x06;																		//PACKET LENGTH LOW
	command[10] = parameters.status;															//
	command[11] = parameters.batteryStatus;													//
	command[12] = parameters.RLedIntensity;													//
	command[13] = parameters.GLedIntensity;													//
	command[14] = parameters.BLedIntensity;													//
	command[15] = parameters.vibrationIntensity;												//
	command[16] = calculateChecksum(&command[2], &command[15]);
	
	
	USART_0_write(0x00);
	for (int i = 0; i < sizeof(command); i++)
	{
		USART_0_write(command[i]);
	}
}

void sendSerialNumber()
{
	static const uint32_t serialNumber[4];
	//const uint32_t *pSN = serialNumber;
	
	//uint32_t* serialNumber[0] = (const uint32_t*)(0x0080A00C);
	//uint32_t* serialNumber[1] = (const uint32_t*)(0x0080A040);
	//uint32_t* serialNumber[2] = (const uint32_t*)(0x0080A044);
	//uint32_t* serialNumber[3] = (const uint32_t*)(0x0080A048);
	
	
	for (int j = 0; j < 4; j++)
	{
		//posilani ?ty? 32b pomoci USART, ktery umi posilat 8b
		for (int i = 0; i < 4; i++)
		{
			USART_0_write(serialNumber[i]);
		}
		
		
		
	}
	

}

void sendTest(){
	
	uint8_t command[11];
	command[0] = 0xAA;																		//header byte 0
	command[1] = 0x00;																		//header byte 1
	command[2] = 0x02;																		//length
	command[3] = DRV_BM64_SEND_SPP_DATA_CMD;													//command ID
	command[4] = 0x00;																		//channel index
	command[5] = 0x00;																		//single packet format
	command[6] = 0x00;																		//TOTAL length HIGH
	command[7] = 0x01;																		//TOTAL LENGTH LOW
	command[8] = 0x00;																		//packet length HIGH
	command[9] = 0x01;																		//PACKET LENGTH LOW
	command[10] = 0x99;											//
	command[11] = calculateChecksum(&command[2], &command[10]);
	
	USART_0_write(0x00);
	for (int i = 0; i < sizeof(command); i++)
	{
		USART_0_write(command[i]);
	}
	
	
	
}