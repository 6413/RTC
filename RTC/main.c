#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define F_CPU 8000000UL
#define F_SCL 100000L
#define LEDS 8
#define OPEN_BOTTOM_LEDS PORTD&=~(1 << PD4)
#define CLOSE_BOTTOM_LEDS PORTD|=(1 << PD4)
#define PIEZO_ON PORTD&=~(1<<PD5)
#define PIEZO_OFF PORTD|=(1<<PD5)
#define WAKEUP_LIGHT_ON PORTD&=~(1 << PD6)
#define WAKEUP_LIGHT_OFF PORTD|=(1 << PD6)
#define TW_START 0xA4
#define TW_READY (TWCR & 0x80)
#define TW_STATUS (TWSR & 0xF8)
#define TW_SEND 0x84
#define TW_STOP 0x94
#define TW_NACK 0x84
#define I2C_Stop() TWCR = TW_STOP
#define READ 1
#define MCP7940N_WRITE			0xDE
#define MCP7940N_ST_BIT_ENABLE		0x80
#define MCP7940N_SQWEN_BIT_ENABLE	0x40

#define Dtotalports 2
volatile unsigned char* Ports[2] = { &PORTC, &PORTD };
unsigned char		portsegmentnums[2] = { 4, 1 };
unsigned char		PORTCsegments[4] = { PC0, PC1, PC2, PC3 };
unsigned char		PORTDsegments[1] = { PD4 };
unsigned char* PORTSEGMENTS[Dtotalports] = { PORTCsegments, PORTDsegments };

unsigned char		SegmentNums[LEDS] = { PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7 };

#define HowManyRTC 6
uint8_t RTCDIGITMAX[HowManyRTC] = { 60, 60, 24, 13, 33, 60 };
unsigned char RTCADDRESS[HowManyRTC + 1] = { 0,  1,  2,  5,  4,  6, 7 };

struct ST_RTCTIME {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t months;
	uint8_t days;
	uint8_t years;
};

uint8_t BINARY2BCD(uint8_t P_BINARY) {
	return ((P_BINARY / 10) << 4) | (P_BINARY % 10);
}
uint8_t BCD2BINARY(uint8_t P_BCD) {
	return (P_BCD & 0x0f) + (((P_BCD >> 4) & 7) * 10);
}

struct ST_RTCTIME RTC2BINARY(struct ST_RTCTIME LRTCTIME) {
	for (int i = 0; i < 6; i++)
	((uint8_t*)(&LRTCTIME))[i] = BCD2BINARY(((uint8_t*)(&LRTCTIME))[i]);
	return LRTCTIME;
}

struct ST_RTCTIME BINARY2RTC(struct ST_RTCTIME LRTCTIME) {
	for (int i = 0; i < 6; i++)
	((uint8_t*)(&LRTCTIME))[i] = BINARY2BCD(((uint8_t*)(&LRTCTIME))[i]);
	return LRTCTIME;
}

uint8_t TBCD2O(uint8_t P_BCD, bool P_LR) {
	if (P_LR)
	return P_BCD & 0x0f;
	else
	return (P_BCD >> 4) & 0x7;
}

void OpenOrCloseAction(unsigned int PPORT, unsigned int Psegment, unsigned int Popenorclose) {
	if (Popenorclose)
	*Ports[PPORT] &= ~(1 << PORTSEGMENTS[PPORT][Psegment]);
	else
	*Ports[PPORT] |= (1 << PORTSEGMENTS[PPORT][Psegment]);
}

unsigned int HandleProcess_PORTNUM = 0;
unsigned int HandleProcess_SEGMENTNUM = 0;
unsigned int LastPortNumber = 0;
unsigned int LastSegmentNumber = 0;

void Open_Digit(uint8_t P_LeftSide, uint8_t P_RightSide)
{
	unsigned char Decimal2Digit[10] = { 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10 };
	unsigned char LRDigit[2] = { P_LeftSide, P_RightSide };
	PORTB = Decimal2Digit[TBCD2O(LRDigit[HandleProcess_SEGMENTNUM >> 1], HandleProcess_SEGMENTNUM & 1)];
}

void Open_Led(uint8_t P_LED) {
	PORTB = ~BCD2BINARY(P_LED);
}

void HandleProcess(struct ST_RTCTIME LRTCTIME, unsigned int Ptimediv, unsigned int Pleddiv) {
	OpenOrCloseAction(LastPortNumber, LastSegmentNumber, 0);
	if (HandleProcess_PORTNUM == 0) {
		unsigned char LLeftDigits;
		if (Ptimediv + 1 == HowManyRTC)	LLeftDigits = 0x20;
		else LLeftDigits = ((uint8_t*)(&LRTCTIME))[Ptimediv + 1];
		Open_Digit(LLeftDigits, ((uint8_t*)(&LRTCTIME))[Ptimediv]);
	}
	else if (HandleProcess_PORTNUM == 1)
	Open_Led(((uint8_t*)(&LRTCTIME))[Pleddiv]);
	OpenOrCloseAction(HandleProcess_PORTNUM, HandleProcess_SEGMENTNUM, 1);

	LastPortNumber = HandleProcess_PORTNUM;
	LastSegmentNumber = HandleProcess_SEGMENTNUM;
	HandleProcess_SEGMENTNUM++;

	if (HandleProcess_SEGMENTNUM == portsegmentnums[HandleProcess_PORTNUM]) {
		HandleProcess_PORTNUM++;
		HandleProcess_SEGMENTNUM = 0;
	}
	if (HandleProcess_PORTNUM == Dtotalports)
	HandleProcess_PORTNUM = 0;
}

unsigned int HoldTime[3] = { 0,0,0 };
unsigned char ButtonD[3] = { PD3, PD0, PD1 };
uint32_t ButtonTimers[6] = { 0, 0, 0, 0, 0, 0 };

bool IBP(uint8_t P_B) { return (!(PIND & (1 << ButtonD[P_B]))); }

bool HandleButtonTimers(uint32_t P_Limit, uint32_t* P_Current, unsigned int P_TimeAddress, unsigned int P_Bx, unsigned int P_By) {
	if (P_Current[P_TimeAddress] < P_Limit) {
		P_Current[P_TimeAddress]++;
		return 0;
	}
	else if (IBP(P_Bx) & IBP(P_By)) {
		P_Current[P_TimeAddress] = 0;
		return 1;
	}
	return 0;
}

void I2C_Init() {
	TWSR = 0;
	TWBR = ((F_CPU / F_SCL) - 16) / 2;
}

uint8_t I2C_Start() {
	TWCR = TW_START;
	while (!TW_READY);
	return (TW_STATUS == 0x08);
}


uint8_t I2C_SendAddr(uint8_t addr) {
	TWDR = addr;
	TWCR = TW_SEND;
	while (!TW_READY);
	return (TW_STATUS == 0x18);
}

uint8_t I2C_Write(uint8_t data) {
	TWDR = data;
	TWCR = TW_SEND;
	while (!TW_READY);
	return (TW_STATUS != 0x28);
}

uint8_t I2C_ReadNACK() {
	TWCR = TW_NACK;
	while (!TW_READY);
	return TWDR;
}

void I2C_WriteRegister(uint8_t deviceRegister, uint8_t data) {
	I2C_Start();
	I2C_SendAddr(MCP7940N_WRITE);
	I2C_Write(deviceRegister);
	I2C_Write(data);
	I2C_Stop();
}

uint8_t  I2C_ReadRegister(uint8_t  busAddr, uint8_t deviceRegister) {
	uint8_t data = 0;
	I2C_Start();
	I2C_SendAddr(MCP7940N_WRITE);
	I2C_Write(deviceRegister);
	I2C_Start();
	I2C_SendAddr(MCP7940N_WRITE + READ);
	data = I2C_ReadNACK();
	I2C_Stop();
	return data;
}

struct ST_RTCTIME MCP7940N_GetRTC(void) {
	struct ST_RTCTIME LRTCTIME;

	for (int i = 0; i < 6; i++)
	((uint8_t*)(&LRTCTIME))[i] = I2C_ReadRegister(MCP7940N_WRITE, RTCADDRESS[i]);

	return LRTCTIME;
}

void MCP7940N_WriteRTC(struct ST_RTCTIME LRTCTIME) {
	for (int i = 0; i < 6; i++)
	I2C_WriteRegister(RTCADDRESS[i], ((uint8_t*)(&LRTCTIME))[i]);
}

void MCP7940N_EnableSTOscillator() {
	uint8_t MCP7940Nregister = 0;
	MCP7940Nregister = I2C_ReadRegister(MCP7940N_WRITE, RTCADDRESS[0]);
	MCP7940Nregister |= MCP7940N_ST_BIT_ENABLE;
	I2C_WriteRegister(RTCADDRESS[0], MCP7940Nregister);
}

void MCP7940N_EnableSQWEN() {
	uint8_t MCP7940Nregister = 0;
	MCP7940Nregister = I2C_ReadRegister(MCP7940N_WRITE, RTCADDRESS[6]);
	MCP7940Nregister |= MCP7940N_SQWEN_BIT_ENABLE;
	I2C_WriteRegister(RTCADDRESS[6], MCP7940Nregister);
}

unsigned int G_CurrentMenu = 0;

struct ST_RTCTIME SetClockRTCTIME;
uint8_t SetClockPDiv = 0;
void MENU_SetClock(void) {
	if (HandleButtonTimers(0x1000, ButtonTimers, 3, 0, 2)) {
		G_CurrentMenu = 0;
		MCP7940N_WriteRTC(BINARY2RTC(SetClockRTCTIME));
		MCP7940N_EnableSTOscillator();
		ButtonTimers[2] = 0;
		return;
	}
	if (IBP(0) || IBP(2)) ButtonTimers[1] = 0;
	if (HandleButtonTimers(0x1000, ButtonTimers, 1, 1, 1)) {
		SetClockPDiv = (SetClockPDiv + 1) % HowManyRTC;
	}
	if (HandleButtonTimers(0x800, ButtonTimers, 0, 0, 1)) {
		((uint8_t*)(&SetClockRTCTIME))[SetClockPDiv] -= 1;
		if (((uint8_t*)(&SetClockRTCTIME))[SetClockPDiv] > RTCDIGITMAX[SetClockPDiv]) ((uint8_t*)(&SetClockRTCTIME))[SetClockPDiv] = RTCDIGITMAX[SetClockPDiv] - 1;
	}
	if (HandleButtonTimers(0x800, ButtonTimers, 2, 2, 1)) {
		((uint8_t*)(&SetClockRTCTIME))[SetClockPDiv] += 1;
		if (((uint8_t*)(&SetClockRTCTIME))[SetClockPDiv] >= RTCDIGITMAX[SetClockPDiv]) ((uint8_t*)(&SetClockRTCTIME))[SetClockPDiv] = 0;
	}
	HandleProcess(BINARY2RTC(SetClockRTCTIME), SetClockPDiv, 0);
}

void MENU_Select(void) {
	struct ST_RTCTIME LRTCTIME = MCP7940N_GetRTC();
	HandleProcess(LRTCTIME, SetClockPDiv, 0);
	if (HandleButtonTimers(0xb0, ButtonTimers, 1, 1, 1)) {
		SetClockPDiv = (SetClockPDiv + 1) % HowManyRTC;
	}
	if (HandleButtonTimers(0x100, ButtonTimers, 2, 2, 2)) {
		SetClockRTCTIME = RTC2BINARY(MCP7940N_GetRTC());
		G_CurrentMenu = 1;
	}
}

typedef void(*fc)(void);
fc FC_Menu[] = { &MENU_Select, &MENU_SetClock };

int main(void) {
	DDRB = 0xFF; PORTB = 0x0;
	DDRD = 0x70; PORTD = 0x00;
	DDRC = 0x0F; PORTC = 0xFF;
	PIEZO_OFF;
	WAKEUP_LIGHT_OFF;
	CLOSE_BOTTOM_LEDS;

	I2C_Init();
	MCP7940N_EnableSTOscillator();
	MCP7940N_EnableSQWEN();
	I2C_WriteRegister(RTCADDRESS[1], 0);

	while (1) {
		FC_Menu[G_CurrentMenu]();
	}
	return 0;
}
