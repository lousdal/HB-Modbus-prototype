/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#define F_CPU 16000000UL		// Either it doesn't affect the clock speed or it not set to use the extern crystal

#include <SoftwareSerial.h>
#include <avr/io.h>
#include <util/delay.h>

//Beginning of Auto generated function prototypes by Atmel Studio
void txToSensor(unsigned int SensorID, byte SensorSetting, unsigned int SensorSetValue);
bool approveMBCRC(int CRCLowByte);
uint16_t ModRTU_CRC(unsigned char buf[], int len);
//End of Auto generated function prototypes by Atmel Studio

// Defines for txToSensor()
#define STX		0x02    // STX is the start signal
#define HBLC_ID		0x8030  // SensorID - Sensor ID consist of two bytes
#define PayloadLength1	0x01	// PayloadLength1 when getting HBLC_settings not setting any parameters
#define PayloadLength2	0x02	// PayloadLength2 when setting one parameter of one byte
#define PayloadLength3	0x03	// PayloadLength3 when setting one parameter of two bytes

// Message type for HBLC settings for
#define SetVal				0x00	// Set Val
#define PBand				0x01	// P-Band
#define AlarmSet			0x02	// Alarm Set
#define AlarmHysterese			0x03	// Alarm Hysterese
#define AlarmDelay			0x04	// Alarm Delay
#define FilterFunction			0x05	// Filter Function
#define CalFunction			0x06	// Cal Function (Zero & Span cal. function)
#define NONCAlarm			0x07	// NO NC Alarm
#define RunIn				0x0A	// Run In
#define ControlLevel			0x0B	// Control/Level mode
#define HiLoAlarm			0x0C	// Hi Lo Alarm
#define LPHPMode			0x0D	// LP HP Mode
#define LEDShift			0x12	// LED Shift
#define HBLC_settings			0x21	// HBLC_setting with payload 0x01 return all the HBLC_settings. Register number 0x21
#define ZeroCali			0x31	// Zero Calibration
#define SpanCali			0x32	// Span Calibration
//#define RefrigerantDef		0x34	// Refrigerant (define) - Should you be able to write to this one?
#define MeasurementLength		0x35	// Measurement Length (same as Sensor Length)
#define RampeFunction			0x36	// Rampe Function
#define ValveFilter			0x38	// Valve Filter (Valve speed open % in second)
#define ValveCloseFilterDef		0x3B	// Valve Close Filter (define) (Valve speed close % in second)
//#define StandPipe			0x3C	// StandPipe - Should you be able to write to this one?
#define OffsetLengthDef			0x3D	// Offset Length (define)
#define MinValveOpenDef			0x3E	// MinValveOpen (define)
#define AnalogDigitalAlarmSettingDef	0x3F	// Analog Digital Alarm Setting (define)
#define MaxValveOpenDef			0x40	// Max Valve Open (define)
#define OffsetMinimumLengthDef		0x41	// Offset Minimum Length (define)
#define WorkingTemperatureDef		0x42	// Working Temperature (define)
#define SetAnalogOrDigitalOutputDef	0x44	// Set Analog Or Digital Output (define)
#define valveOpeningDegreeDef		0x45	// Valve Opening Degree (define)
#define AnalogDigitalHystSettingDef	0x47	// Analog Digital Hyst Setting (define)
//#define SensorTypeDef			0x4A	// Sensor Type - Should you be able to write to this one?

#define rxPin PD7	// rxPin for SoftwareSerial // 11 or PD7 or just 7
#define txPin PD7	// txPin for SoftwareSerial

#define RE PD2		// RE pin of the MAX485CSA+
#define DE PD3		// DE pin of the MAX485CSA+

#define heaterPin A5	// heaterPin

#define MB_RX_BUFFER_SIZE 256	// Receiver buffer of 256 bytes
#define S_RX_BUFFER_SIZE  256	// Receiver buffer of 256 bytes

// Modbus functions
#define ReadCoilStatus		0x01
#define ReadInputStatus		0x02
#define ReadHoldingRegisters	0x03
#define ReadInputRegisters	0x04
#define WriteSingleCoil		0x05
#define WriteSingleRegister	0x06
#define WriteMultipleCoils	0x15
#define WriteMultipleRegisters	0x16

// Modbus Exception (Error) Codes
#define IllegalFunction		0x01
#define IllegalDataAddress	0x02
#define IllegalDataValue	0x03
#define SlaveDeviceFailure	0x04
#define IsWrong			0x80

// Variables for txToModbusRequestResponse()
int SlaveAddress = 0x01;
int PayloadLength = PayloadLength2;
int quantityOfRegisters = 0;
byte modbusErrorCode = 0x00;
byte modbusExceptionCode = 0x00;

// bool for txToSensor
bool twoByteSensorSetValueFlag = false;

// Array and variables for RX from Modbus
unsigned char MBrxBuffer[MB_RX_BUFFER_SIZE];
uint8_t MBrxReadPos = 0;
uint8_t MBrxWritePos = 0;

// Array and variables for RX from Sensor
int SensorRxBuffer[S_RX_BUFFER_SIZE];
uint8_t SensorRxReadPos = 0;
uint8_t SensorRxWritePos = 0;

// Sensor register array for inserting variables in correct order
int SensorRegisterBuffer[34];

// Constant and variables for doing timings with millis()
unsigned long currentTime;
unsigned long previousTime = 0;
const unsigned long eventInterval = 1000;		// This constant is used to define the interval of how often HBLC_settings will be requested

// Timer1 load and overflow counter
// const int timer1_load = 56875;			// 16MHz / 256 = 62500 tics pr. second. 62500 - (62500 / 10) = 56250 tics the TCNT1 should be loaded with to overflow every 100 ms
// volatile uint8_t alarm_overflow;			// variable to store number of overflows. SHOULD IT BE uint16_t INSTEAD?

// ====================== HBLC_settings constants ArrayPosition(AP) & Register Number(RN) ======================
// Diagnostic
const int ZeroDataAP = 32;
const byte ZeroDataRN = 0x03;			// = decimal 3
const int SpanZeroDataAP = 34;
const byte SpanZeroDataRN = 0x04;		// = decimal 4
const byte ActualLevelPctRN = 0x00;		// = decimal 0
const int pFmdlScaleForToolAP = 36;
const int ValveOpeningDegreeAP = 58;
const byte ValveOpeningDegreeRN = 0x02;		// = decimal 2

// Advanced settings
const int HiLoAlarmAP = 18;
const byte HiLoAlarmRN = 0x14;			// = decimal 20
const int AlarmHystereseAP = 8;
const byte AlarmHystereseRN = 0x15;		// = decimal 21
const int NONCAlarmAP = 13;
const byte NONCAlarmRN = 0x16;			// = decimal 22
const int LPHPModeAP = 19;
const byte LPHPModeRN = 0x17;			// = decimal 23
const int SetAnalogOrDigitalOutputAP = 57;
const byte SetAnalogOrDigitalOutputRN = 0x18;	// = decimal 24
const int AnalogDigitalAlarmSettingAP = 51;
const byte AnalogDigitalAlarmSettingRN = 0x19;	// = decimal 25
const int AnalogDigitalHystSettingAP = 60;
const byte AnalogDigitalHystSettingRN = 0x1A;	// = decimal 26
const int LEDShiftAP = 25;
const byte LEDShiftRN = 0x1B;			// = decimal 27
const int RampeFunctionAP = 41;
const byte RampeFunctionRN = 0x1C;		// = decimal 28
const int ValveFilterAP = 43;
const byte ValveFilterRN = 0x1D;		// = decimal 29
const int ValveCloseFilterAP = 46;
const byte ValveCloseFilterRN = 0x1E;		// = decimal 30
const int MinValveOpenAP = 50;
const byte MinValveOpenRN = 0x1F;		// = decimal 31
const int MaxValveOpenAP = 52;
const byte MaxValveOpenRN = 0x20;		// = decimal 32
const int SensorIDAP = 1;
const byte SensorIDRN = 0x21;			// = decimal 33

// Basic settings
const int ControLevelAP = 17;
const byte ControlLevelRN = 0x05;		// = decimal 5
const int SetValAP = 5;
const byte SetValRN = 0x06;			// = decimal 6
const int PBandAP = 6;
const byte PBandRN = 0x07;			// = decimal 7
const int FilterFunctionAP = 11;
const byte FilterFunctionRN = 0x08;		// = decimal 8
const int RunInAP = 16;
const byte RunInRN = 0x09;			// = decimal 9
const int CalFunctionAP = 12;
const byte CalFunctionRN = 0x0A;		// = decimal 10
const int AlarmSettingAP = 7;
const byte AlarmSettingRN = 0x0B;		// = decimal 11
const byte AboveBelowAlarmSettingRN = 0x01;	// = decimal 1
const int AlarmDelayAP = 9;
const byte AlarmDelayRN = 0x0C;			// = decimal 12
const int OffsetLengthAP = 48;
const byte OffsetLengthRN = 0x0D;		// = decimal 13
const int OffsetMinimumLengthAP = 53;
const byte OffsetMinimumLengthRN = 0x0E;	// = decimal 14
const int SensorTypeAP = 63;
const byte SensorTypeRN = 0x0F;			// = decimal 15
const int RefrigerantAP = 38;
const byte RefrigerantRN = 0x10;		// = decimal 16
const int MeasurementLengthAP = 39;
const byte MeasurementLengthRN = 0x11;		// = decimal 17
const int StandPipeAP = 47;
const byte StandPipeRN = 0x12;			// = decimal 18
const int WorkingTemperatureAP = 55;
const byte WorkingTemperatureRN = 0x13;		// = decimal 19

// ====================== Variables for live sensor readings ======================
// Diagnostic					   [AP][AP]		[RN]	Array Position(AP) in SensorRxBuffer. Register Number(RN) for the position in the register.
unsigned int zero_data;				// [32][33]		[3]	Value range 0-3500 pF. Should maximum be 6553.5 pF (65535)?
unsigned int span_zero_data;			// [34][35]		[4]	Value range 0-3500 pF. Should maximum be 6553.5 pF (65535)?
unsigned int actual_level_pct_int;		//			[0]
unsigned int actual_measurement_pF;		//			[??]	same as pFmdlScaleForTool in HBLC_settings
unsigned int valveOpeningDegree;		// [58]			[2]	Control Level pct of the valve opening degree (0-100%)

// Advanced settings
unsigned int hi_lo_alarm;			// [18]			[20]	High = 1 Low = 0
unsigned int Alarm_hysterese;			// [8]			[21]	in pct. Value range 0-100%
unsigned int NO_NC_alarm;			// [13]			[22]	NC == 0 NO == 1
unsigned int LP_HP_mode;			// [19]			[23]	LP == 0 HP == 1
unsigned int SetAnalogOrDigitalOutput;		// [57]			[24]	Analog == 0 Digital == 1
unsigned int AnalogDigitalAlarmSetting; 	// [51]			[25]	??
unsigned int AnalogDigitalHystSetting;		// [60]			[26]	??
unsigned int led_shift;				// [25]			[27]	Is it the Alarm LED indication. Alarm LED = 0 Control LED = 1
unsigned int Rampe_function;			// [41]			[28]	Ramp startup % in sec
unsigned int Valve_filter;			// [43]			[29]	Valve speed open % in sec
unsigned int ValveCloseFilter;			// [46]			[30]	Valve speed close % in sec
unsigned int MinValveOpen;			// [50]			[31]	Minimum valve opening in %
unsigned int MaxValveOpen;			// [52]			[32]	Maximum valve opening in %
unsigned int SensorID;				// [1][2]		[33]	Sensor ID

// Basic settings
unsigned int Control_level;			// [17]			[02]	Level = 0 Control = 1
unsigned int Set_val;				// [5]			[06]	Setpoint level in % (0-100%)
unsigned int P_band;				// [6]			[07]	P-band in % 0-100

unsigned int Filter_function;			// [11]			[8]	Filter time const. in second. 0-200.
unsigned int Run_in;				// [16]			[9]	Run in signal. OFF == 0x00, ON == 0x01
unsigned int Cal_function;			// [12]			[10]	(Zero & Span cal. function) OFF == 0, ON == 1
unsigned int Alarm_setting;			// [7]			[11]	in pct. Value range 0-100%
unsigned int AboveBelowAlarmSetting;		//			[1]	True == 1, False == 0
unsigned int Alarm_delay;			// [9][10]		[12]	in seconds. Value range 0-600.
unsigned int OffsetLength;			// [48][49]		[13]	??
unsigned int OffsetMinimumLength;		// [53][54]		[14]	??

unsigned int SensorType;			// [63]			[15]	same as mechanical type. Probe sensor = 0, Wire sensor = 1, Flex probe sensor = 2, HBLT-A2 probe = 3, HBLT-A2 Inch probe = 4, Wire sensor ver. W = 5
unsigned int Refrigerant;			// [38]			[16]	NH3 == 0, CO2 == 1, OIL == 2, HFC R134a == 3, HFC R507 == 4, HFC R404A == 5, HFC R407C == 6, HFC R410A == 7, HFC R22 == 8, HFO R1234ZE == 9, HFO R1233ZD == 10
unsigned int Measurement_length;		// [39][40]		[17]	same as Sensor Length.
unsigned int StandPipe;				// [47]			[18]	DN25 1" == 0, DN32 1¼" == 1, DN40 1½" == 2, DN50 2" == 3, DN65 2½" == 4, DN80 3" == 5, DN100 4" == 6
unsigned int WorkingTemperature;		// [55]			[19]	-60 - -40° == 0,  -40 - -30° == 1, -30 - -20° == 2, -20 - -10° == 3, -10 - 0° == 4, 0-10° == 5, 10-20° == 6, 20-30° == 6, 30-40° == 7, 40-50° == 8, 50-60° == 9, 60-70° == 10, 70-80° == 11, 80-90° == 12, 90-100° == 13

// ================================================================================

SoftwareSerial softwareSerial = SoftwareSerial(rxPin, txPin); // Initializing the softwareSerial with RX, TX

// Modbus functions
void resetMBrxBuffer()
{
	memset(MBrxBuffer, 0x00, sizeof(MBrxBuffer));		// Resets the MBrxBuffer with 0x00 on all places in the array
	MBrxWritePos = 0;					// Sets the MBrxWritePos to zero to start at the first position in the array
}

void resetSensorRxBuffer()
{
	memset(SensorRxBuffer, 0x00, sizeof(SensorRxBuffer));	// Resets the SensorRxBuffer with 0x00 on all places in the array
	SensorRxWritePos = 0;					// Sets the SensorRxWritePos to zero to start at the first position in the array
}

// Function for sending HBLC settings to sensor
void txToSensor(unsigned int SensorID, byte SensorSetting, unsigned int SensorSetValue)
{
	Serial.flush();
	pinMode(txPin, OUTPUT);
	//DDRD &=~ (1 << txPin);
	//DDRD |= (1 << txPin);	// sets txPin as output

	byte LowSensorID = SensorID % 256;	// use modulus 256 to get the 8 bit remainder that is the 8 LSB
	byte HighSensorID = SensorID >> 8;	// throw the 8 least significant bits away, so only the 8 most significant bits are left.
	byte LowSensorSetValue;
	byte HighSensorSetValue;
	
	PayloadLength = PayloadLength2;
	
	if((SensorSetting == ZeroCali) || (SensorSetting == SpanCali) || (SensorSetting == SetVal) || (SensorSetting == AlarmDelay) ||
	(SensorSetting == OffsetLengthDef) || (SensorSetting == OffsetMinimumLengthDef) || (SensorSetting == MeasurementLength))
	{
		twoByteSensorSetValueFlag = true;
		PayloadLength = PayloadLength3;
		
		if(SensorSetValue < 256)
		{
			LowSensorSetValue = SensorSetValue % 256;	// use modulus 256 to get the 8 bit remainder that is the 8 LSB
			HighSensorSetValue = (byte)0x00;
		}
		else
		{
			LowSensorSetValue = SensorSetValue % 256;	// use modulus 256 to get the 8 bit remainder that is the 8 LSB
			HighSensorSetValue = SensorSetValue >> 8;	// throw the 8 least significant bits away, so only 8 the most significant bits are left.
		}
	}
	if(SensorSetting == HBLC_settings)
	{
		PayloadLength = PayloadLength1;
	}
	
	int checksum = 0;
	checksum += (LowSensorID ^ 0xFFFF);
	checksum += (HighSensorID ^ 0xFFFF);
	checksum += (PayloadLength ^ 0xFFFF);
	if(SensorSetting != HBLC_settings)
	{
		checksum += (SensorSetting ^ 0xFFFF);
		if(twoByteSensorSetValueFlag == true)
		{
			checksum += (LowSensorSetValue ^ 0xFFFF);
			checksum += (HighSensorSetValue ^ 0xFFFF);
		}
		else
		{
			checksum += (SensorSetValue ^ 0xFFFF);
		}
	}
	else
	{
		checksum += ((SensorSetting) ^ 0xFFFF);
	}
	byte CRC1stByte = checksum % 256;				// throw the 8 least significant bits away, so only the 8 most significant bits are left.
	byte CRC2ndByte = checksum >> 8;				// use modulus 256 to get the 8 bit remainder that is the 8 LSB

	softwareSerial.write(0x02);					// Wake up byte followed by a delay of 50ms
	delay(50);
	softwareSerial.write(STX);					// STX is the start signal instead of Set RS-232 into transmitting mode
	softwareSerial.write(LowSensorID);				// Low byte of SensorID
	softwareSerial.write(HighSensorID);				// High byte of SensorID
	
	if(SensorSetting != HBLC_settings)
	{
		softwareSerial.write(PayloadLength);			// Payload length - the number of bytes containing the payload
		softwareSerial.write(SensorSetting);			// Payload data - 1st payload is the SensorSetting
		if(twoByteSensorSetValueFlag == true)
		{
			softwareSerial.write(LowSensorSetValue);	// Payload data - Low Byte of SensorSetValue
			softwareSerial.write(HighSensorSetValue);	// Payload data - High Byte of SensorSetValue
		}
		else
		{
			softwareSerial.write(SensorSetValue);
		}
	}
	else
	{
		softwareSerial.write(PayloadLength);
		softwareSerial.write(SensorSetting);			// Payload data - 1st payload is the SensorSetting.
	}
	softwareSerial.write(CRC1stByte);				// Checksum (CRC) CRC-First Byte - LSB
	softwareSerial.write(CRC2ndByte);				// Checksum (CRC) CRC-Second Byte - MSB
	softwareSerial.flush();						// To be sure that all that is written to serial is send
	pinMode(rxPin, INPUT);
	//DDRD &=~ (1 << txPin);
	//DDRD |=  (0 << txPin);	// sets txPin as input
	resetSensorRxBuffer();
}

bool approveMBCRC(int CRCLowByte)			// function to check receivedCRC with a calculated CRC of what is received
{
	int tempCalculatedCRC = ModRTU_CRC(MBrxBuffer, CRCLowByte);	// uses the function ModRTU_CRC to calculate the CRC based on first 6 bytes received
	
	int receivedCRC = (MBrxBuffer[CRCLowByte] << 8) + MBrxBuffer[CRCLowByte + 1];	// creates a variable of the CRCLowByte and add with the a CRC high byte
	
	if(tempCalculatedCRC == receivedCRC)		// if tempCalculatedCRC is equal to receivedCRC
	{
		return true;	// return true
	}
	else
	{
		return false;	// return false
	}
}

bool approveHBLCsettingsCRC(int CRCLowByte)
{
	int receivedCRC = (SensorRxBuffer[CRCLowByte] << 8) + SensorRxBuffer[CRCLowByte+1];
	int checksum = 0;
	int calcCRC = 0;
	
	for(int i=1; i <= 80; i++)
	{
		checksum += (SensorRxBuffer[i]^0xFFFF);
	}
	
	byte lowByteChecksum = checksum;
	byte highByteChecksum = checksum >> 8;
	
	calcCRC = lowByteChecksum << 8;
	calcCRC += highByteChecksum;
	
	if(receivedCRC == calcCRC)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(unsigned char buf[], int len)
{
	uint16_t crc = 0xFFFF;
	
	for (int pos = 0; pos < len; pos++)
	{
		crc ^= (uint16_t)buf[pos];          	// XOR byte into least sig. byte of crc
		
		for (int i = 8; i != 0; i--)        	// Loop over each bit
		{
			if ((crc & 0x0001) != 0)	// If the LSB is set
			{
				crc >>= 1;              // Shift right
				crc ^= 0xA001;		// and XOR with 0xA001
			}
			else                            // Else LSB is not set
				crc >>= 1;		// Just shift right
		}
	}
	byte lowByteCRC = crc;				// Makes low byte of crc
	byte highByteCRC = crc >> 8;			// Makes high byte of crc

	// swaps around the low and high byte
	crc = lowByteCRC << 8;
	crc += highByteCRC;
	
	return crc;
}

// Sensor functions
//void txToModbusRequestResponse(int FunctionArrayPosition, byte ModbusFunction)
void txToModbusRequestResponse(byte ModbusFunction, int SensorRegisterNumber)
{
	softwareSerial.flush();
	digitalWrite(DE, HIGH);
	digitalWrite(RE, HIGH);
	
	if((ModbusFunction == ReadInputRegisters) || (ModbusFunction == ReadHoldingRegisters))
	{
		int originalSensorRegisterNumber = SensorRegisterNumber;
		int MODBUS_RESPONSE_BUFFER_SIZE = 5;
		int ModbusFunctionReceived = ModbusFunction;
		byte byteCount = 0x02;
		
		if(quantityOfRegisters > 0x01)	//quantityOfRegisters is what's on MBrxBuffer[5]
		{
			byteCount *= quantityOfRegisters;
			MODBUS_RESPONSE_BUFFER_SIZE = 5 + byteCount;
		}
				
		unsigned char *ModbusResponseBuffer = new unsigned char(MODBUS_RESPONSE_BUFFER_SIZE);	// creates an dynamic buffer with the given MODBUS_RESPONSE_BUFFER_SIZE
		
		ModbusResponseBuffer[0] = SlaveAddress;
		ModbusResponseBuffer[1] = ModbusFunctionReceived;
		ModbusResponseBuffer[2] = byteCount;
		ModbusResponseBuffer[3] = SensorRegisterBuffer[SensorRegisterNumber] >> 8;
		ModbusResponseBuffer[4] = SensorRegisterBuffer[SensorRegisterNumber];
		SensorRegisterNumber++;
		
		if(quantityOfRegisters > 0x01)	//quantityOfRegisters is what's on MBrxBuffer[5]
		{
			for(int i = 5; i < (5 + byteCount - (byteCount / 2 + 1)); i++)
			{
				ModbusResponseBuffer[i] = SensorRegisterBuffer[SensorRegisterNumber] >> 8;
				ModbusResponseBuffer[i+1] = SensorRegisterBuffer[SensorRegisterNumber++];
			}
		}

		int calcCRC = ModRTU_CRC(ModbusResponseBuffer, MODBUS_RESPONSE_BUFFER_SIZE);
		
		Serial.write(SlaveAddress);
		Serial.write(ModbusFunctionReceived);
		Serial.write(byteCount);

		Serial.write(SensorRegisterBuffer[originalSensorRegisterNumber] >> 8);	// Data High
		Serial.write(SensorRegisterBuffer[originalSensorRegisterNumber]);		// Data Low
		originalSensorRegisterNumber += 1;
		
		if(quantityOfRegisters > 0x01)	//quantityOfRegisters is what's on MBrxBuffer[5]
		{
			for(int i = 5; i < (5 + byteCount - (byteCount / 2 + 1)); i++)			// for loop that runs for the quantityOfRegisters - 1
			{
				Serial.write(SensorRegisterBuffer[originalSensorRegisterNumber] >> 8);	// Data High
				Serial.write(SensorRegisterBuffer[originalSensorRegisterNumber++]);	// Data Low
				i+1;
			}
		}
		
		Serial.write(calcCRC >> 8);
		Serial.write(calcCRC);
		
		delete [] ModbusResponseBuffer;
	}
	else if(ModbusFunction == WriteSingleRegister)
	{
		for(int i = SensorRegisterNumber; i < 8; i++)	// In this for loop SensorRegisterNumber is referring the to array position in the MBrxBuffer // Couldn't I just say that is should be 0??
		{
			Serial.write(MBrxBuffer[i]);		// Function (lenght is 8 bits). Indicates the function code; e.g. read coils/holding registers
		}
	}
	
	Serial.flush();
	
	digitalWrite(DE, LOW);
	digitalWrite(RE, LOW);
	
	resetMBrxBuffer();
	resetSensorRxBuffer();
}

void txToModbusRequestErrorResponse(byte modbusExceptionResponse, byte modbusExceptionErrorCode)
{
	digitalWrite(DE, HIGH);
	digitalWrite(RE, HIGH);
	
	int MODBUS_ERROR_RESPONSE_BUFFER_SIZE = 3;
	unsigned char *ModbusErrorResponseBuffer = new unsigned char(MODBUS_ERROR_RESPONSE_BUFFER_SIZE);	// creates an dynamic buffer with the given size of
	
	ModbusErrorResponseBuffer[0] = SlaveAddress;
	ModbusErrorResponseBuffer[1] = modbusExceptionResponse;
	ModbusErrorResponseBuffer[2] = modbusExceptionErrorCode;

	int calcCRC = ModRTU_CRC(ModbusErrorResponseBuffer, MODBUS_ERROR_RESPONSE_BUFFER_SIZE);

	Serial.write(SlaveAddress);
	Serial.write(modbusExceptionResponse);
	Serial.write(modbusExceptionErrorCode);
	Serial.write(calcCRC >> 8);		// CRC for error checking
	Serial.write(calcCRC);			// CRC for error checking
	
	Serial.flush();
	
	digitalWrite(DE, LOW);
	digitalWrite(RE, LOW);
	
	resetMBrxBuffer();
}

void requestHBLCsettings()
{
	txToSensor(HBLC_ID, HBLC_settings, 0x01);
}

// void timer1_init()		// initialize timer, interrupt and variable
// {
// 	TCCR1A = 0x00;				// Resets Timer1 Control Reg A
// 	
// 	// Set to prescaler of 256
// 	TCCR1B |=  (1 << CS12);		// Sets it to 1
// 	TCCR1B &= ~(1 << CS11);		// Sets it to 0
// 	TCCR1B &= ~(1 << CS10);		// Sets it to 0
// 	
// 	TCNT1 = 59375;				// loads the timer with		56250 == 100 ms to overflow		//		56875 == 90 ms to overflow		//		57500 == 80 ms to overflow		//		59375 == 50 ms to overflow
// 
// 	TIMSK1 = (1 << TOIE1);		// enable overflow interrupt
// 	
// 	sei();						// enable global interrupts
// 	
// 	alarm_overflow = 0;			// sets overflow counter variable to 0
// }
// 
// ISR(TIMER1_OVF_vect)		// TIMER1 overflow Interrupt Service Routine called whenever TCNT1 overflows. It overflows every 100 ms.
// {
// 	alarm_overflow++;		// keep track of number of overflows. Increments every 100 ms.
// 	TCNT1 = 59375;
// }

void setup()
{
	pinMode(rxPin, INPUT);		// define pin modes for tx, rx
	//DDRD |= (0 << rxPin);		// sets PD7 as input
	
	//DDRC |= (1 << 5);		// sets PC5 as output
	//PORTC |= (0 << 5);		// sets PORT C5 LOW. Then the Heater isn't supplied and the resistors isn't getting hot.
	pinMode(heaterPin, OUTPUT);	// sets the heaterPin as an output 
	digitalWrite(heaterPin, LOW);	// set LOW to avoid the heater being on, while developing the code. Should be enabled when code is done.
	
	pinMode(RE, OUTPUT);
	pinMode(DE, OUTPUT);
	digitalWrite(RE, LOW);		// sets MAX486csa in receiving mode
	digitalWrite(DE, LOW);		// sets MAX486csa in receiving mode
	
	Serial.begin(9600);		// Open serial communications and wait for port to open
	while (!Serial)
	{
		;			// wait for serial port to connect. Needed for native USB port only
	}
	softwareSerial.begin(9600);	// Set data rate for the SoftwareSerial port
	
	//timer1_init();		// initialize timer1 settings
}

void loop()
{
	currentTime = millis();					// Updates the variable currentTime frequently
	
	if (currentTime - previousTime >= eventInterval)	// The constant eventInterval shows after how many milliseconds this if statement is true
	{
		resetSensorRxBuffer();
		requestHBLCsettings();
		previousTime = currentTime;			// Update the timing for the next time around
	}
	
	if (Serial.available())
	{
		if(MBrxWritePos < MB_RX_BUFFER_SIZE)
		{
			MBrxBuffer[MBrxWritePos++] = Serial.read();	// Read the serial buffer and put into MBrxBuffer and increment the MBrxWritePos
		}
		else
		{
			MBrxWritePos = 0;
		}
		
		if(MBrxBuffer[0] == 0x00 || MBrxBuffer[2] != 0x00)	// Sometimes there is 0x00 on position 0 and 2 i the MBrxBuffer. Then the resetMBrxBuffer needs to be reset in order to not destroy the communication.
		{
			resetMBrxBuffer();
		}
		
		if(MBrxBuffer[0] == SlaveAddress && approveMBCRC(6) == true)
		{
			if(MBrxBuffer[1] == ReadCoilStatus);
			else if(MBrxBuffer[1] == ReadInputStatus);
			else if(MBrxBuffer[1] == ReadHoldingRegisters)
			{
// 				if((MBrxBuffer[3] == HBLC_settings) && (MBrxBuffer[5] == 0x01))	// HBLC_setting (0x21 is 33)
// 				{
// 					txToSensor(HBLC_ID, HBLC_settings, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));
// 					resetMBrxBuffer();
// 				}
//				if((MBrxBuffer[5] > 0x01) && (MBrxBuffer[5] <= 0x21))	// if-statement to read multiple ReadHoldingRegisters
//				{
//					quantityOfRegisters = MBrxBuffer[5];
//					int startingAddress = MBrxBuffer[3];
//					txToModbusRequestResponse(ReadHoldingRegisters, startingAddress);
//				}
				if((MBrxBuffer[2] == 0x00) && (MBrxBuffer[3] >= 0x00 || MBrxBuffer[3] <= 0x21) && (MBrxBuffer[4] == 0x00) && (MBrxBuffer[5] == 0x01))
				{
					int sensorRegisterNumber = MBrxBuffer[3];
					txToModbusRequestResponse(ReadHoldingRegisters, sensorRegisterNumber);
					resetMBrxBuffer();
				}
				else
				{
					txToModbusRequestErrorResponse((ReadHoldingRegisters + IsWrong), IllegalDataAddress);	// IsWrong = 0x80, IllegalDataAddress = 0x02
				}
			}
			else if(MBrxBuffer[1] == ReadInputRegisters)
			{
//				if((MBrxBuffer[5] > 0x01) && (MBrxBuffer[5] <= 0x21))	// if-statement to read multiple ReadInputRegisters
//				{
//					quantityOfRegisters = MBrxBuffer[5];
//					int startingAddress = MBrxBuffer[3];
//					txToModbusRequestResponse(ReadInputRegisters, startingAddress);
//				}
				
				if((MBrxBuffer[2] == 0x00) && (MBrxBuffer[3] >= 0x00 || MBrxBuffer[3] <= 0x21) && (MBrxBuffer[4] == 0x00) && (MBrxBuffer[5] == 0x01))
				{
					int sensorRegisterNumber = MBrxBuffer[3];
					txToModbusRequestResponse(ReadInputRegisters, sensorRegisterNumber);
					resetMBrxBuffer();
				}
				else
				{
					txToModbusRequestErrorResponse((ReadInputRegisters + IsWrong), IllegalDataAddress);	// IsWrong = 0x80, IllegalDataAddress = 0x02
				}
			}
			
			else if(MBrxBuffer[1] == WriteSingleCoil);
			else if(MBrxBuffer[1] == WriteSingleRegister)
			{
				if(MBrxBuffer[3] == ZeroDataRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[4] << 8) + MBrxBuffer[5] <= 65535)			// ZeroDataRN can be 0 - 65535 (0-6553.5 pF).
					{
						txToSensor(HBLC_ID, ZeroCali, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, ZeroDataRN and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == SpanZeroDataRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[4] << 8) + MBrxBuffer[5] <= 65535)			// SpanZeroDataRN can be 0 - 65535 (0-6553.5 pF).
					{
						txToSensor(HBLC_ID, SpanCali, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, SpanZeroDataRN and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == ControlLevelRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// ControlLevel can be 0 == Level or 1 == Control
					{
						txToSensor(HBLC_ID, ControlLevel, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, ControlLevel and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == SetValRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// SetVal (set point level in %) can be 0 - 100 %
					{
						txToSensor(HBLC_ID, SetVal, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, SetVal and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == PBandRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// PBand can be 0 - 100 %
					{
						txToSensor(HBLC_ID, PBand, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, PBand and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == FilterFunctionRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 200))						// FilterFunction can be 0 - 200 sec
					{
						txToSensor(HBLC_ID, FilterFunction, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, FilterFunction and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == RunInRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// ControlLevel can be 0 == Level or 1 == Control
					{
						txToSensor(HBLC_ID, RunIn, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, RunIn and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == CalFunctionRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// ControlLevel can be 0 == Level or 1 == Control
					{
						txToSensor(HBLC_ID, CalFunction, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, CalFunction and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == AlarmSettingRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// AlarmSettingRN can be 0 - 100 %
					{
						txToSensor(HBLC_ID, AlarmSet, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, AlarmSet and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == AlarmDelayRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (((MBrxBuffer[4] << 8) + MBrxBuffer[5]) <= 600))			// Alarm Delay can be 0 - 600 seconds.
					{
						txToSensor(HBLC_ID, AlarmDelay, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, AlarmDelay and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == OffsetLengthRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (((MBrxBuffer[4] << 8) + MBrxBuffer[5]) <= 65535))		// Offset Length (max level) can be 0 - 65535 mm?
					{
						txToSensor(HBLC_ID, OffsetLengthDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, OffsetLengthDef and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == OffsetMinimumLengthRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (((MBrxBuffer[4] << 8) + MBrxBuffer[5]) <= 65535))			// Offset Minimum Length (level) can be 0 - 65535 mm?
					{
						txToSensor(HBLC_ID, OffsetMinimumLengthDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, OffsetMinimumLengthDef and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);		// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == MeasurementLengthRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (((MBrxBuffer[4] << 8) + MBrxBuffer[5]) <= 65535))		// Measurement Length (same as Sensor Length) (mm or inch? how to differentiate?) can be 0 - 65535 mm?
					{
						txToSensor(HBLC_ID, MeasurementLength, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, MeasurementLength and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == WorkingTemperatureRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 14))											// WorkingTemperature can be 0 - 14
					{
						txToSensor(HBLC_ID, WorkingTemperatureDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, WorkingTemperatureDef and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);		// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == HiLoAlarmRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// Hi Lo Alarm can be 0 == Low or 1 == High
					{
						txToSensor(HBLC_ID, HiLoAlarm, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, HiLoAlarm and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == AlarmHystereseRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// Alarm Hysterese can be 0 - 100 %
					{
						txToSensor(HBLC_ID, AlarmHysterese, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, AlarmHysterese and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == NONCAlarmRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// NO NC Alarm can be 0 == NC or 1 == NO
					{
						txToSensor(HBLC_ID, NONCAlarm, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, NONCAlarm and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == LPHPModeRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// LP HP Mode can be 0 == LP or 1 == HP
					{
						txToSensor(HBLC_ID, LPHPMode, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, LPHPMode and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == SetAnalogOrDigitalOutputRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))													// Set Analog Or Digital Output can be 0 == Analog or 1 == Digital
					{
						txToSensor(HBLC_ID, SetAnalogOrDigitalOutputDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, SetAnalogOrDigitalOutputDef and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);		// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == AnalogDigitalAlarmSettingRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))												// Analog Digital Alarm Setting can be 0 - 100 %
					{
						txToSensor(HBLC_ID, AnalogDigitalAlarmSettingDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, AnalogDigitalAlarmSettingDef and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);		// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == AnalogDigitalHystSettingRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))												// Analog Digital Hyst Setting can be 0 - 100 %
					{
						txToSensor(HBLC_ID, AnalogDigitalHystSettingDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, AnalogDigitalHystSettingDef and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);		// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == LEDShiftRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 1))						// Set LED Shift can be 0 == Alarm LED or 1 == Control LED
					{
						txToSensor(HBLC_ID, LEDShift, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, LEDShift and the SensorSetValue found on position 4 and 5 in the array
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == RampeFunctionRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// Rampe Function can be 0 - 100 %
					{
						txToSensor(HBLC_ID, RampeFunction, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, RampeFunction and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == ValveFilterRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// Valve Filter (valve speed open % in sec) can be 0 - 100 %
					{
						txToSensor(HBLC_ID, ValveFilter, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));		// Calls the txToSensor function with parameter HBLC_ID, ValveFilter and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == ValveCloseFilterRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// Valve Close Filter (valve speed close % in sec) can be 0 - 100 %
					{
						txToSensor(HBLC_ID, ValveCloseFilterDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, ValveCloseFilterDef and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == MinValveOpenRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// Min Valve Open can be 0 - 100 %
					{
						txToSensor(HBLC_ID, MinValveOpenDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, MinValveOpenDef and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else if(MBrxBuffer[3] == MaxValveOpenRN)
				{
					if((MBrxBuffer[5] >= 0x00) && (MBrxBuffer[5] <= 100))						// Max Valve Open can be 0 - 100 %
					{
						txToSensor(HBLC_ID, MaxValveOpenDef, (int)((MBrxBuffer[4] << 8) + MBrxBuffer[5]));	// Calls the txToSensor function with parameter HBLC_ID, MaxValveOpenDef and the SensorSetValue found on position 4
					}
					else
					{
						txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataValue);	// IsWrong = 0x80, IllegalDataValue = 0x03
					}
				}
				else
				{
					txToModbusRequestErrorResponse((WriteSingleRegister + IsWrong), IllegalDataAddress);	// IsWrong = 0x80, IllegalDataAddress = 0x02
				}
			}
			else if(MBrxBuffer[1] == WriteMultipleCoils);
			else if(MBrxBuffer[1] == WriteMultipleRegisters);
			else
			{
				txToModbusRequestErrorResponse((MBrxBuffer[1] + IsWrong), IllegalFunction);			// IsWrong = 0x80, IllegalFunction = 0x01
			}
		}
	}

	if (softwareSerial.available())
	{
		if(SensorRxWritePos < S_RX_BUFFER_SIZE)
		{
			SensorRxBuffer[SensorRxWritePos++] = softwareSerial.read();		// Read the softwareSerial and put into the SensorRxBuffer and increment the SensorRxWritePos
			if(SensorRxBuffer[0] == 0x00)						// Sometimes the SensorRxBuffer starts with 0x00 instead of 0x02 (STX from sensor). This if statement is to avoid that.
			{
				resetSensorRxBuffer();
			}
		}
		else
		{
			SensorRxWritePos = 0;
		}
		
		if((SensorRxBuffer[4] == 0x23) && (SensorRxBuffer[5] == 0x01) && (SensorRxBuffer[7] == 0xFF)) // 0x23 means that the sensor received the information. 0x01 means that the value is set correct. Array position 7 is the last digit of the CRC and will always be 0xFF if the new setting is set.
		{
			txToModbusRequestResponse(WriteSingleRegister, 0);		// Respond with Slave Address, Function code, Register Address High & Low and Register Value High & Low. Parameter 1 represents the position of the function code in the array.
			SensorRxBuffer[4] = 0x00;					// Set SensorRxBuffer[4] = 0x00 so that it will not go into this if statement again. Then the HBCL_settings will be in the SensorRxBuffer.
		}
		
		if(approveHBLCsettingsCRC(81) == true)	// calls the approveHBLCsettingsCRC that checks if the received CRC is equal to the calculatedCRC
		{
			// Diagnostic
			zero_data = (SensorRxBuffer[ZeroDataAP] + (SensorRxBuffer[ZeroDataAP+1] << 8));
			span_zero_data = (SensorRxBuffer[SpanZeroDataAP] + (SensorRxBuffer[SpanZeroDataAP+1] << 8));
			
			actual_measurement_pF = (SensorRxBuffer[pFmdlScaleForToolAP] + (SensorRxBuffer[pFmdlScaleForToolAP+1] << 8));
			float zero_data_float = zero_data;
			float span_zero_data_float = span_zero_data;
			float actual_mesurement_pF_float = actual_measurement_pF;
			float ActualLevelPctFloat = ((actual_mesurement_pF_float - zero_data_float) / (span_zero_data_float + zero_data_float - zero_data_float)) * 100; // multiply with 100 if you want an integer and with 1000 if you want one number after the digit
			actual_level_pct_int = round(ActualLevelPctFloat);
			valveOpeningDegree = SensorRxBuffer[ValveOpeningDegreeAP];
			
			// Advanced settings
			hi_lo_alarm = SensorRxBuffer[HiLoAlarmAP];
			Alarm_hysterese = SensorRxBuffer[AlarmHystereseAP];
			NO_NC_alarm = SensorRxBuffer[NONCAlarmAP];
			LP_HP_mode = SensorRxBuffer[LPHPModeAP];
			SetAnalogOrDigitalOutput = SensorRxBuffer[SetAnalogOrDigitalOutputAP];
			AnalogDigitalAlarmSetting = SensorRxBuffer[AnalogDigitalAlarmSettingAP];
			AnalogDigitalHystSetting = SensorRxBuffer[AnalogDigitalHystSettingAP];
			led_shift = SensorRxBuffer[LEDShiftAP];
			Rampe_function = SensorRxBuffer[RampeFunctionAP];
			Valve_filter = SensorRxBuffer[ValveFilterAP];
			ValveCloseFilter = SensorRxBuffer[ValveCloseFilterAP];
			MinValveOpen = SensorRxBuffer[MinValveOpenAP];
			MaxValveOpen = SensorRxBuffer[MaxValveOpenAP];
			SensorID = (SensorRxBuffer[SensorIDAP] + (SensorRxBuffer[SensorIDAP+1] << 8));
			
			// Basic settings
			Control_level = SensorRxBuffer[ControLevelAP];
			Set_val = SensorRxBuffer[SetValAP];
			P_band = SensorRxBuffer[PBandAP];
			Filter_function = SensorRxBuffer[FilterFunctionAP];
			Run_in = SensorRxBuffer[RunInAP];
			Cal_function = SensorRxBuffer[CalFunctionAP];
			Alarm_setting = SensorRxBuffer[AlarmSettingAP];
			Alarm_delay = (SensorRxBuffer[AlarmDelayAP] + (SensorRxBuffer[AlarmDelayAP+1] << 8));
			OffsetLength = (SensorRxBuffer[OffsetLengthAP] + (SensorRxBuffer[OffsetLengthAP+1] << 8));
			OffsetMinimumLength = (SensorRxBuffer[OffsetMinimumLengthAP] + (SensorRxBuffer[OffsetMinimumLengthAP+1] << 8));
			SensorType = SensorRxBuffer[SensorTypeAP];
			Refrigerant = SensorRxBuffer[RefrigerantAP];
			Measurement_length = (SensorRxBuffer[MeasurementLengthAP] + (SensorRxBuffer[MeasurementLengthAP+1] << 8));
			StandPipe = SensorRxBuffer[StandPipeAP];
			WorkingTemperature = SensorRxBuffer[WorkingTemperatureAP];
			
			
// 			if((actual_level_pct_int > Alarm_setting) && (hi_lo_alarm == 0x0001))
// 			{
// 				if((alarm_overflow / 10) >= Alarm_delay)		// alarm_overflow overflows every 100 ms. alarm_overflow / 10 turns the ms into seconds which can then be compared to Alarm_delay
// 				{
// 					AboveBelowAlarmSetting = 0x01;				// 0x01 == true
// 				}
// 			}
// 			else if((actual_level_pct_int < Alarm_setting) && (hi_lo_alarm == 0x0000))
// 			{
// 				if((alarm_overflow / 10) >= Alarm_delay)		// alarm_overflow overflows every 100 ms. alarm_overflow / 10 turns the ms into seconds which can then be compared to Alarm_delay
// 				{
// 					AboveBelowAlarmSetting = 0x01;				// 0x01 == true
// 				}
// 			}
// 			else
// 			{
// 				alarm_overflow = 0;
// 				AboveBelowAlarmSetting = 0x00;					// 0x00 == false
// 			}
			
			// Inserting variables into SensorRegisterBuffer array
			SensorRegisterBuffer[0] = actual_level_pct_int;
			SensorRegisterBuffer[1] = AboveBelowAlarmSetting;
			SensorRegisterBuffer[2] = valveOpeningDegree;
			SensorRegisterBuffer[3] = zero_data;
			SensorRegisterBuffer[4] = span_zero_data;
			SensorRegisterBuffer[5] = Control_level;
			SensorRegisterBuffer[6] = Set_val;
			SensorRegisterBuffer[7] = P_band;
			SensorRegisterBuffer[8] = Filter_function;
			SensorRegisterBuffer[9] = Run_in;
			SensorRegisterBuffer[10] = Cal_function;
			SensorRegisterBuffer[11] = Alarm_setting;
			SensorRegisterBuffer[12] = Alarm_delay;
			SensorRegisterBuffer[13] = OffsetLength;
			SensorRegisterBuffer[14] = OffsetMinimumLength;
			SensorRegisterBuffer[15] = SensorType;
			SensorRegisterBuffer[16] = Refrigerant;
			SensorRegisterBuffer[17] = Measurement_length;
			SensorRegisterBuffer[18] = StandPipe;
			SensorRegisterBuffer[19] = WorkingTemperature;
			SensorRegisterBuffer[20] = hi_lo_alarm;
			SensorRegisterBuffer[21] = Alarm_hysterese;
			SensorRegisterBuffer[22] = NO_NC_alarm;
			SensorRegisterBuffer[23] = LP_HP_mode;
			SensorRegisterBuffer[24] = SetAnalogOrDigitalOutput;
			SensorRegisterBuffer[25] = AnalogDigitalAlarmSetting;
			SensorRegisterBuffer[26] = AnalogDigitalHystSetting;
			SensorRegisterBuffer[27] = led_shift;
			SensorRegisterBuffer[28] = Rampe_function;
			SensorRegisterBuffer[29] = Valve_filter;
			SensorRegisterBuffer[30] = ValveCloseFilter;
			SensorRegisterBuffer[31] = MinValveOpen;
			SensorRegisterBuffer[32] = MaxValveOpen;
			SensorRegisterBuffer[33] = SensorID;
			
			resetSensorRxBuffer();
		}
	}
}
