/*
 * audio.c
 *
 *  Created on: Nov 20, 2017
 *      Author: parimalp
 */
#include <stdio.h>
#include <xil_io.h>
#include <sleep.h>
#include "xiicps.h"
#include <xil_printf.h>
#include <xparameters.h>
#include "xgpio.h"
#include "xuartps.h"
#include "stdlib.h"
#include "audio.h"

//----------------------------------------------------
// PROTOTYPE FUNCTIONS
//----------------------------------------------------
unsigned char IicConfig(unsigned int DeviceIdPS);
void AudioPllConfig();
void AudioWriteToReg(unsigned char u8RegAddr, unsigned char u8Data);
void AudioCODECConfig(void);
void SelectMic(void);
void MuteAll(void);
void SelectLineIn(void);
void record(unsigned long u32MemOffset, unsigned long u32NrSamples);
void play(unsigned long u32MemOffset, unsigned long u32NrSamples);
void AudioStraight(unsigned long u32NrSamples);

void AudioConfigureJacks(void);

//Global variables
XIicPs Iic;
#define DDR_MEMORY 0x10000000
#define AUD_NR_SAMPLES 1000000
#define DDR_OFFSET_AUD 0

int main(void)
{

	u32 push, switches;
	u32 flag;
	xil_printf("PYNQ-Z2 Audio Test\r\n");

	//Configure the IIC data structure
	IicConfig(XPAR_XIICPS_1_DEVICE_ID);

	//Configure the Audio Codec's PLL
	AudioPllConfig();

	//Configure the mixers for Line in, MIC in, and Line out ports.
	//Call AudioCODECConfig() for configuring Line In, Mic In, and Line Out
	AudioCODECConfig();
	xil_printf("ADAU1761 configured\n\r");
	while(1) {
		switches = Xil_In32(XPAR_SWITCHES_GPIO_BASEADDR);
		if(switches&01) {
			print("Line In selected\r\n");
			SelectLineIn();
		}
		else if(switches&02) {
			print("Mic In selected\r\n");
			SelectMic();
		}
		else {
			print("Neither Mic nor LineIn selected\r\n");
			MuteAll();		// DeselectLineIn would do the same
			print("Exiting...\r\n");
			exit(0);
		}

		flag=1;
		while(flag)
		{
			push = Xil_In32(XPAR_BTNS_GPIO_BASEADDR);
			switch(push)
			{
				case 0x1:
				{
					xil_printf("Playing straight %d samples\r\n",AUD_NR_SAMPLES);
					AudioStraight(AUD_NR_SAMPLES);
					print("Done...\r\n");
					break;
				}
				case 0x4:
				{
					xil_printf("Playing %d recorded samples\r\n",AUD_NR_SAMPLES);
					play(DDR_OFFSET_AUD,AUD_NR_SAMPLES);
					print("Done...\r\n");
					break;
				}
				case 0x2:
				{
					xil_printf("Recording %d samples\r\n",AUD_NR_SAMPLES);
					record(DDR_OFFSET_AUD,AUD_NR_SAMPLES);
					print("Done...\r\n");
					break;
				}
				case 0x8:
				{
					flag=0;
					break;
				}
				default: break;
			}

		}
	}
}

void SelectMic(void) {
	AudioWriteToReg(R5_RECORD_MIXER_LEFT_CONTROL_1, 0x08);
	AudioWriteToReg(R7_RECORD_MIXER_RIGHT_CONTROL_1, 0x0C);
}

void MuteAll(void) {
	AudioWriteToReg(R5_RECORD_MIXER_LEFT_CONTROL_1, 0x00);  //mute left channel input
	AudioWriteToReg(R7_RECORD_MIXER_RIGHT_CONTROL_1, 0x00); //mute right channel input
}

void SelectLineIn(void) {
	AudioWriteToReg(R5_RECORD_MIXER_LEFT_CONTROL_1, 0x05);
	AudioWriteToReg(R7_RECORD_MIXER_RIGHT_CONTROL_1, 0x05);
}

/******************************************************************************
 * Record and play the audio without storing in the DDR3.
 * @param	u32NrSamples is the number of samples to read and output.
 * @return	none.
 *****************************************************************************/
void AudioStraight(unsigned long u32NrSamples)
{
	u32  temp, u32Temp;
	for(temp=0; temp < u32NrSamples; temp++)
	{
		do //wait for RX data to become available
		{
			u32Temp = Xil_In32(I2S_STATUS_REG);
		} while ( u32Temp == 0);
		Xil_Out32(I2S_STATUS_REG, 0x00000001); //Clear data rdy bit

		/* Read the sample from and output to the codec */
		Xil_Out32(I2S_DATA_TX_L_REG, Xil_In32(I2S_DATA_RX_L_REG));	// left
		Xil_Out32(I2S_DATA_TX_R_REG, Xil_In32(I2S_DATA_RX_R_REG));	// right
	}
}

/******************************************************************************
 * Record the samples and store into the DDR3.
 * @param	u32MemOffset is the offset in the DDR3 from which the data will be
 * 			stored.
 * @param	u32NrSamples is the number of samples to store.
 * @return	none.
 *****************************************************************************/
void record(unsigned long u32MemOffset, unsigned long u32NrSamples)
{
	u32  temp, u32Temp, sample;

	for(temp=0; temp < (2*u32NrSamples); temp++)
	{
		do //wait for RX data to become available
		{
			u32Temp = Xil_In32(I2S_STATUS_REG);
		} while ( u32Temp == 0);
		Xil_Out32(I2S_STATUS_REG, 0x00000001); //Clear data rdy bit


		sample=Xil_In32(I2S_DATA_RX_L_REG);
		Xil_Out32((DDR_MEMORY+u32MemOffset+temp*4),sample);
		temp++;
		sample=Xil_In32(I2S_DATA_RX_R_REG);
		Xil_Out32((DDR_MEMORY+u32MemOffset+temp*4),sample);
	}
}

/******************************************************************************
 * Play the stored data from the DDR3.
 * @param	u32MemOffset is the offset in the DDR3 from which the data will be
 * 			played.
 * @param	u32NrSamples is the number of samples to played.
 * @return	none.
*****************************************************************************/
void play(unsigned long u32MemOffset, unsigned long u32NrSamples)
{
	u32  temp, u32Temp;

	for(temp=0; temp < (2*u32NrSamples); temp++)
	{
		do //wait for RX data to become available
		{
			u32Temp = Xil_In32(I2S_STATUS_REG);
		} while ( u32Temp == 0);
		Xil_Out32(I2S_STATUS_REG, 0x00000001); //Clear data rdy bit

		/* Output stored audio to the codec */
		Xil_Out32(I2S_DATA_TX_L_REG, Xil_In32(DDR_MEMORY+u32MemOffset+temp*4));	// left
		temp++;
		Xil_Out32(I2S_DATA_TX_R_REG, Xil_In32(DDR_MEMORY+u32MemOffset+temp*4));	// right
	}
}


/* ---------------------------------------------------------------------------- *
 * 									IicConfig()									*
 * ---------------------------------------------------------------------------- *
 * Initializes the IIC driver by looking up the configuration in the config
 * table and then initializing it. Also sets the IIC serial clock rate.
 * ---------------------------------------------------------------------------- */
unsigned char IicConfig(unsigned int DeviceIdPS)
{
	XIicPs_Config *Config;
	int Status;

	/* Initialize the IIC driver so that it's ready to use */

	// Look up the configuration in the config table
	Config = XIicPs_LookupConfig(DeviceIdPS);
	if(NULL == Config) {
		return XST_FAILURE;
	}

	// Initialize the IIC driver configuration
	Status = XIicPs_CfgInitialize(&Iic, Config, Config->BaseAddress);
	if(Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//Set the IIC serial clock rate.
	XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);

	return XST_SUCCESS;
}

/* ---------------------------------------------------------------------------- *
 * 								AudioPllConfig()								*
 * ---------------------------------------------------------------------------- *
 * Configures audio codes's internal PLL. With MCLK = 10 MHz it configures the
 * PLL for a VCO frequency = 49.152 MHz, and an audio sample rate of 48 KHz.
 * ---------------------------------------------------------------------------- */
void AudioPllConfig() {

	unsigned char u8TxData[8], u8RxData[6];
	int Status;

	Status = IicConfig(XPAR_XIICPS_1_DEVICE_ID);
	if(Status != XST_SUCCESS) {
		xil_printf("\nError initializing IIC");

	}

	// Disable Core Clock
	AudioWriteToReg(R0_CLOCK_CONTROL, 0x0E);

	/* 	MCLK = 10 MHz
		R = 0100 = 4, N = 0x02 0x3C = 572, M = 0x02 0x71 = 625

		PLL required output = 1024x48 KHz
		(PLLout)			= 49.152 MHz

		PLLout/MCLK			= 49.152 MHz/10 MHz
							= 4.9152 MHz
							= R + (N/M)
							= 4 + (572/625) */

	// Write 6 bytes to R1 @ register address 0x4002
	u8TxData[0] = 0x40; // Register write address [15:8]
	u8TxData[1] = 0x02; // Register write address [7:0]
	u8TxData[2] = 0x02; // byte 6 - M[15:8]
	u8TxData[3] = 0x71; // byte 5 - M[7:0]
	u8TxData[4] = 0x02; // byte 4 - N[15:8]
	u8TxData[5] = 0x3C; // byte 3 - N[7:0]
	u8TxData[6] = 0x21; // byte 2 - 7 = reserved, bits 6:3 = R[3:0], 2:1 = X[1:0], 0 = PLL operation mode
	u8TxData[7] = 0x03; // byte 1 - 7:2 = reserved, 1 = PLL Lock, 0 = Core clock enable // was 0x01

	// Write bytes to PLL Control register R1 @ 0x4002
	XIicPs_MasterSendPolled(&Iic, u8TxData, 8, (IIC_SLAVE_ADDR >> 1));
	while(XIicPs_BusIsBusy(&Iic));

	// Register address set: 0x4002
	u8TxData[0] = 0x40;
	u8TxData[1] = 0x02;

	// Poll PLL Lock bit
	do {
		XIicPs_MasterSendPolled(&Iic, u8TxData, 2, (IIC_SLAVE_ADDR >> 1));
		while(XIicPs_BusIsBusy(&Iic));
		XIicPs_MasterRecvPolled(&Iic, u8RxData, 6, (IIC_SLAVE_ADDR >> 1));
		while(XIicPs_BusIsBusy(&Iic));
	}
	while((u8RxData[5] & 0x02) == 0); // while not locked

	AudioWriteToReg(R0_CLOCK_CONTROL, 0x0F);	// 1111
												// bit 3:		CLKSRC = PLL Clock input
												// bits 2:1:	INFREQ = 1024 x fs
												// bit 0:		COREN = Core Clock enabled
}


/* ---------------------------------------------------------------------------- *
 * 								AudioWriteToReg									*
 * ---------------------------------------------------------------------------- *
 * Function to write one byte (8-bits) to one of the registers from the audio
 * controller.
 * ---------------------------------------------------------------------------- */
void AudioWriteToReg(unsigned char u8RegAddr, unsigned char u8Data) {

	unsigned char u8TxData[3];

	u8TxData[0] = 0x40;
	u8TxData[1] = u8RegAddr;
	u8TxData[2] = u8Data;

	XIicPs_MasterSendPolled(&Iic, u8TxData, 3, (IIC_SLAVE_ADDR >> 1));
	while(XIicPs_BusIsBusy(&Iic));
}

/* ---------------------------------------------------------------------------- *
 * 								AudioCODECConfig()							*
 * ---------------------------------------------------------------------------- *
 * Configures Line-In input, ADC's, DAC's, Line-Out and HP-Out.
 * ---------------------------------------------------------------------------- */
void AudioCODECConfig() {
	// input path control registers are configured in SelectMic and SelectLineIn
	AudioWriteToReg(R4_RECORD_MIXER_LEFT_CONTROL_0, 0x01);
	AudioWriteToReg(R6_RECORD_MIXER_RIGHT_CONTROL_0, 0x01);
	AudioWriteToReg(R8_LEFT_DIFFERENTIAL_INPUT_VOLUME_CONTROL, 0xB2);
	AudioWriteToReg(R9_RIGHT_DIFFERENTIAL_INPUT_VOLUME_CONTROL, 0xB2);
	AudioWriteToReg(R10_RECORD_MICROPHONE_BIAS_CONTROL, 0x01);
	AudioWriteToReg(R14_ALC_CONTROL_3, 0x20);
	AudioWriteToReg(R15_SERIAL_PORT_CONTROL_0, 0x01);
	AudioWriteToReg(R19_ADC_CONTROL, 0x33);
	AudioWriteToReg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x21);
	AudioWriteToReg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x41);
	AudioWriteToReg(R28_PLAYBACK_LR_MIXER_MONO_OUTPUT_CONTROL, 0x01);
	AudioWriteToReg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL, 0xE7);
	AudioWriteToReg(R33_PLAYBACK_MONO_OUTPUT_CONTROL, 0xE5);
	AudioWriteToReg(R35_PLAYBACK_POWER_MANAGEMENT, 0x03);
	AudioWriteToReg(R36_DAC_CONTROL_0, 0x03);
	AudioWriteToReg(R58_SERIAL_INPUT_ROUTE_CONTROL, 0x01);
	AudioWriteToReg(R59_SERIAL_OUTPUT_ROUTE_CONTROL, 0x01);
	AudioWriteToReg(R61_DSP_ENABLE, 0x01);
	AudioWriteToReg(R62_DSP_RUN, 0x01);
	AudioWriteToReg(R65_CLOCK_ENABLE_0, 0x7F);
	AudioWriteToReg(R66_CLOCK_ENABLE_1, 0x03);
}
