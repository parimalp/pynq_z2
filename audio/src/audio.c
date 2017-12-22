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
#define AUD_NR_SAMPLES 200000 // 1000000
#define DDR_OFFSET_AUD 0

int main(void)
{

	u32 push, switches;
	u32 flag; //, LineInSelected, MICInSelected;
	xil_printf("PYNQ-Z2 Audio Test\r\n");

	//Configure the IIC data structure
	IicConfig(XPAR_XIICPS_1_DEVICE_ID);

	//Configure the Audio Codec's PLL
	AudioPllConfig();

	//Configure the mixers for Line in, MIC in, and Line out ports.
	//Call AudioCODECConfig() for configuring Line In, Mic In, and Line Out
	AudioCODECConfig();

	print("ADAU1761 configured\n\r");
	print("Make sure to turn on SW0 for Line In, SW1 for MIC In\r\n");
	print("SW0 is sampled first. If both switches are OFF then program ends\r\n");
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
			print("Neither MIC nor LineIn selected\r\n");
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
				case 0x2:
				{
					xil_printf("Recording %d samples\r\n",AUD_NR_SAMPLES);
					record(DDR_OFFSET_AUD,AUD_NR_SAMPLES);
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
	AudioWriteToReg(R4_RECORD_MIXER_LEFT_CONTROL_0, 0x01); // Mixer 1 (left channel)
	AudioWriteToReg(R5_RECORD_MIXER_LEFT_CONTROL_1, 0x10); // LDBOOST, set to 20 dB
	AudioWriteToReg(R8_LEFT_DIFFERENTIAL_INPUT_VOLUME_CONTROL, 0xB3);  // LDVOL, set to 21 dB

	AudioWriteToReg(R6_RECORD_MIXER_RIGHT_CONTROL_0, 0x01); // Mixer 2 (right channel)
	AudioWriteToReg(R7_RECORD_MIXER_RIGHT_CONTROL_1, 0x10); // RDBOOST, set to 20 dB
	AudioWriteToReg(R9_RIGHT_DIFFERENTIAL_INPUT_VOLUME_CONTROL, 0xB3); // RDVOL, set to 21 dB
}

void MuteAll(void) {
	AudioWriteToReg(R4_RECORD_MIXER_LEFT_CONTROL_0, 0x00);  //mute Mixer 1 (left channel)
	AudioWriteToReg(R6_RECORD_MIXER_RIGHT_CONTROL_0, 0x00); //mute Mixer2 (right channel)
}

void SelectLineIn(void) {
	AudioWriteToReg(R4_RECORD_MIXER_LEFT_CONTROL_0, 0x01); // Mixer 1  (left channel)
	AudioWriteToReg(R5_RECORD_MIXER_LEFT_CONTROL_1, 0x07); // Enable LAUX (MX1AUXG)

	AudioWriteToReg(R6_RECORD_MIXER_RIGHT_CONTROL_0, 0x01); // Mixer 2
	AudioWriteToReg(R7_RECORD_MIXER_RIGHT_CONTROL_1, 0x07); // Enable RAUX (MX2AUXG)
}

/******************************************************************************
 * Record and play the audio without storing in the DDR3.
 * @param	u32NrSamples is the number of samples to read and output.
 * @return	none.
 *****************************************************************************/
void AudioStraight(unsigned long u32NrSamples)
{
	u32  temp, u32Temp;
	// Use Left/Right input and set them to 6 dB
	AudioWriteToReg(R23_PLAYBACK_MIXER_LEFT_CONTROL_1, 0x00); // Mixer1 input muted
	AudioWriteToReg(R25_PLAYBACK_MIXER_RIGHT_CONTROL_1, 0x00); // Mixer2 input muted
	// Enable Mixer3 and Mixer4
	AudioWriteToReg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x21);
	AudioWriteToReg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x41);
	// Enable Left/Right Headphone out
	AudioWriteToReg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL, 0xE7);
	AudioWriteToReg(R30_PLAYBACK_HEADPHONE_RIGHT_VOLUME_CONTROL, 0xE7);
	for(temp=0; temp < u32NrSamples; temp++)
	{
		do //wait for RX data to become available
		{
			u32Temp = Xil_In32(I2S_STATUS_REG);
		} while ( u32Temp == 0);
		Xil_Out32(I2S_STATUS_REG, 0x00000001); //Clear data rdy bit

		Xil_Out32(I2S_DATA_TX_L_REG, Xil_In32(I2S_DATA_RX_L_REG));	// left
		Xil_Out32(I2S_DATA_TX_R_REG, Xil_In32(I2S_DATA_RX_R_REG));	// right
	}
	AudioWriteToReg(R23_PLAYBACK_MIXER_LEFT_CONTROL_1, 0x00);
	AudioWriteToReg(R25_PLAYBACK_MIXER_RIGHT_CONTROL_1, 0x00);
	AudioWriteToReg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x00);
	AudioWriteToReg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x00);
	AudioWriteToReg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL, 0xE5); // Mute LHP
	AudioWriteToReg(R30_PLAYBACK_HEADPHONE_RIGHT_VOLUME_CONTROL, 0xE5); // Mute RHP
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
	// Unmute left and right DAC which gets samples from DAC_SDATA port
	// Enable Mixer3 and Mixer4
	AudioWriteToReg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x21);
	AudioWriteToReg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x41);
	// Enable Left/Right Headphone out
	AudioWriteToReg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL, 0xE7); // Mute LHP
	AudioWriteToReg(R30_PLAYBACK_HEADPHONE_RIGHT_VOLUME_CONTROL, 0xE7); // Mute RHP

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
	// Mute left and right DAC which gets samples from DAC_SDATA port
	AudioWriteToReg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x01);
	AudioWriteToReg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x01);
	// Mute left input to mixer3 (R23) and right input to mixer4 (R25)
	AudioWriteToReg(R23_PLAYBACK_MIXER_LEFT_CONTROL_1, 0x00);
	AudioWriteToReg(R25_PLAYBACK_MIXER_RIGHT_CONTROL_1, 0x00);
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
	// Mute Mixer1 and Mixer2 here, enable when MIC and Line In used
	AudioWriteToReg(R4_RECORD_MIXER_LEFT_CONTROL_0, 0x00);
	AudioWriteToReg(R6_RECORD_MIXER_RIGHT_CONTROL_0, 0x00);
	// Set LDVOL and RDVOL to 21 dB and Enable left and right differential enabled
	AudioWriteToReg(R8_LEFT_DIFFERENTIAL_INPUT_VOLUME_CONTROL, 0xB3);
	AudioWriteToReg(R9_RIGHT_DIFFERENTIAL_INPUT_VOLUME_CONTROL, 0xB3);
	// Enable MIC bias
	AudioWriteToReg(R10_RECORD_MICROPHONE_BIAS_CONTROL, 0x01);
	// Enable ALC control and noise gate
	AudioWriteToReg(R14_ALC_CONTROL_3, 0x20);
	// Put CODEC in Master mode
	AudioWriteToReg(R15_SERIAL_PORT_CONTROL_0, 0x01);
	// Enable ADC on both channels, normal polarity and ADC high-pass filter
	AudioWriteToReg(R19_ADC_CONTROL, 0x33);
	// Mute play back Mixer3 and Mixer4 and enable when output is required (straight and play back modes)
	AudioWriteToReg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x00); // 0x21);
	AudioWriteToReg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x00); // 0x41);
	// Mute left input to mixer3 (R23) and right input to mixer4 (R25)
	// Enable them in straight play mode
	AudioWriteToReg(R23_PLAYBACK_MIXER_LEFT_CONTROL_1, 0x00);
	AudioWriteToReg(R25_PLAYBACK_MIXER_RIGHT_CONTROL_1, 0x00);
	// MONO output control at Mixer7 not required
	// AudioWriteToReg(R28_PLAYBACK_LR_MIXER_MONO_OUTPUT_CONTROL, 0x01);
	// Mute left and right channels output; enable them when output is needed (straight and play back modes)
	AudioWriteToReg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL, 0xE5);
	AudioWriteToReg(R30_PLAYBACK_HEADPHONE_RIGHT_VOLUME_CONTROL, 0xE5);
	// MONO output control not required
	// AudioWriteToReg(R33_PLAYBACK_MONO_OUTPUT_CONTROL, 0xE5);
	// Enable play back right and left channels
	AudioWriteToReg(R35_PLAYBACK_POWER_MANAGEMENT, 0x03);
	// Enable DAC for both channels
	AudioWriteToReg(R36_DAC_CONTROL_0, 0x03);
	// Set SDATA_In to DAC
	AudioWriteToReg(R58_SERIAL_INPUT_ROUTE_CONTROL, 0x01);
	// Set SDATA_Out to ADC
	AudioWriteToReg(R59_SERIAL_OUTPUT_ROUTE_CONTROL, 0x01);
	// Enable DSP and DSP Run
	AudioWriteToReg(R61_DSP_ENABLE, 0x01);
	AudioWriteToReg(R62_DSP_RUN, 0x01);
	// Enables the digital clock engine for different blocks within the ADAU1761
	// All clocks are enabled
	AudioWriteToReg(R65_CLOCK_ENABLE_0, 0x7F);
	// Enable Digital Clock Generator 0 and 1. Generator 0 generates sample rates for the ADCs,
	// DACs, and DSP. Generator 1 generates BCLK and LRCLK for the serial port.
	AudioWriteToReg(R66_CLOCK_ENABLE_1, 0x03);
}
