#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

uint8_t dynamicIndPlace = 0;
uint8_t previousIndPlace = 3;
uint8_t numbersPlaceBuff[4];
uint8_t charsPlaceBuff[4];
uint8_t dispMode = 0; // 0 - get numbers from numbersPlaceBuff, 1 - get chars from charsPlaceBuff

uint8_t DS3231_addr = 0xD0;

uint8_t error_msg[] = {0x9E, 0x0A, 0x0A, 0x00}; //Err
uint8_t succes_msg[] = {0xB6, 0x7C, 0x9C, 0x9C}; //SUCC
uint8_t numbers[] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6}; //0,1,2,3,4,5,6,7,8,9
uint8_t hexNums[] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6, 0xEE, 0x3E, 0x9C, 0x7A, 0x9E, 0x8E}; //0,1,2,3,4,5,6,7,8,9,A,b,C,d,E,F



void shiftByte(uint8_t data)
{
	PORTD &= ~(1<<4); //OE low
	PORTD |= (1<<3); //MR high
	for (uint8_t i=0; i<8; i++)
	{
		PORTD &= ~(1<<2); //clock pulse falling
		if (data & (1<<i))
		{
			PORTD |= (1<<0); //set DS to high if bit is high
		}
		else
		{
			PORTD &= ~(1<<0); //set DS to low if bit is low
		}
		PORTD |= (1<<2); //clock pulse rising
	}
	PORTD |= (1<<1); //STCP high
	PORTD &= ~(1<<1); //STCP low
 	for (uint8_t i = 0; i<2; i++)
	{
		PORTD ^= (1<<2); //1 pulse to SHCP to flush shift register
	}
}

void resetShiftReg(void)
{
	PORTD &= ~((1<<3) | (1<<4)); //set to low OE, MR
	/*for (uint8_t i=0; i<16; i++)
	{
		PORTD ^= (1<<1); //send 8 clock pulses to STCP to reset
	}*/
	PORTD |= (1<<1);  //STCP high
	PORTD &= ~(1<<1); //STCP low
	PORTD |= (1<<4);  //OE high
}

void setRegOutput(uint8_t state)
{
	if (state)
	{
		PORTD &= ~(1<<4); //enable outputs
	}
	else
	{
		PORTD |= (1<<4); //disable outputs
	}
}

void SEND_RAW(uint8_t data[])
{	
	cli();
	dispMode = 1;
	for(uint8_t i=0; i<4; i++)
	{
		charsPlaceBuff[i] = data[i];
	}
	sei();
}

void showHexCode(uint16_t code)
{
	cli();
	dispMode = 1;
	charsPlaceBuff[0] = hexNums[((code & 0xF000)>>12)];
	charsPlaceBuff[1] = hexNums[((code & 0x0F00)>>8)];
	charsPlaceBuff[2] = hexNums[((code & 0x00F0)>>4)];
	charsPlaceBuff[3] = hexNums[(code & 0x000F)];
	sei();
}

void ERROR(uint8_t code)
{
	SEND_RAW(error_msg);
	_delay_ms(2000);
	showHexCode(code);
	_delay_ms(2000);
	cli();
	dispMode = 0;
	sei();
}

void SUCCESS(uint8_t code)
{
	SEND_RAW(succes_msg);
	_delay_ms(2000);
	showHexCode(code);
	_delay_ms(2000);
	cli();
	dispMode = 0;
	sei();
}

void showNumDec(uint16_t number)
{
	cli();
	dispMode = 0;
	numbersPlaceBuff[0] = (number / 1000) % 10;
	numbersPlaceBuff[1] = (number / 100) % 10;
	numbersPlaceBuff[2] = (number / 10) % 10;
	numbersPlaceBuff[3] = number % 10;
	sei();
}

void showTime(uint8_t hours, uint8_t minutes)
{
	cli();
	dispMode = 0;
	numbersPlaceBuff[0] = (hours / 10) % 10;
	numbersPlaceBuff[1] = hours % 10;
	numbersPlaceBuff[2] = (minutes / 10) % 10;
	numbersPlaceBuff[3] = minutes % 10;
	sei();
}

void TWI_Init(void)
{
	TWBR = 2; // set TWI freq to 31250 Hz
}

void TWI_Start(void)
{
	TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));   // send start condition
	while (!(TWCR & (1<<TWINT)));                   // wait until TWINT become 1
	if (!(((TWSR & 0xF8) == 0x08) | ((TWSR & 0xF8) == 0x10)))	ERROR(TWSR & 0xF8); // check status
}

void TWI_SendByte(uint8_t data)
{
	TWDR = data;                     // put data into TWDR
	TWCR = ((1<<TWINT) | (1<<TWEN)); // send data
	while (!(TWCR & (1<<TWINT)));    // wait until TWINT become 1
	if (!(((TWSR & 0xF8) == 0x18) | ((TWSR & 0xF8) == 0x28) | ((TWSR & 0xF8) == 0x40)))	ERROR(TWSR & 0xF8); // check status
}

uint8_t TWI_ReciveByte(uint8_t sendAck)
{
	if (sendAck != 0)       // send ack?
	{
		TWCR |= (1<<TWEA);  // send ack
	}
	else
	{
		TWCR &= ~(1<<TWEA); // send nack
	}
	TWCR |= ((1<<TWINT) | (1<<TWEN)); // start reciving
	while (!(TWCR & (1<<TWINT)));     // wait until TWINT become 1
	if (!(((TWSR & 0xF8) == 0x50) | ((TWSR & 0xF8) == 0x58)))	ERROR(TWSR & 0xF8); // check status
	return TWDR; // return received data
}

void TWI_Stop(void)
{
	TWCR |= ((1<<TWINT) | (1<<TWSTO) | (1<<TWEN)); // send stop condition
}

void DS3231_write_regs(uint8_t data[], uint8_t start_addr, uint8_t regs_num)
{
	TWI_Start();
	TWI_SendByte(DS3231_addr);
	TWI_SendByte(start_addr);
	for (uint8_t i=0; i<regs_num; i++)
	{
		TWI_SendByte(data[i]);
	}
	TWI_Stop();
}

void DS3231_write_reg(uint8_t data, uint8_t addr)
{
	TWI_Start();               // start twi
	TWI_SendByte(DS3231_addr); // send slave address (r/w default low)
	TWI_SendByte(addr);        // send register address
	TWI_SendByte(data);        // send data
	TWI_Stop();                // stop twi
}

uint8_t DS3231_read_reg(uint8_t addr)
{
	TWI_Start();                         // start twi
	TWI_SendByte(DS3231_addr);           // send slave address (r/w low)
	TWI_SendByte(addr);                  // send register address
	TWI_Start();                         // restart twi
	TWI_SendByte(DS3231_addr | (1<<0));  // send slave address (r/w high)
	uint8_t temp = TWI_ReciveByte(0);    // recive byte
	TWI_Stop();                          // stop twi
	return temp;
}

void DS3231_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	seconds = ((seconds / 10)<<4) | (seconds % 10);  // calculate seconds and minutes to decimal format
	minutes = ((minutes / 10)<<4) | (minutes % 10);
	hours = ((hours / 10)<<4) | (hours % 10);        // calculate hours. Only 24 - hours format
	seconds &= 0x7F;
	minutes &= 0x7F;
	hours &= 0x3F;
	uint8_t pack[3] = {seconds, minutes, hours};
	DS3231_write_regs(pack, 0x00, 3);                // write to the module
}

void DS3231_set_date(uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	day &= 0x07;                                    // calculate decimal
	date = ((date / 10)<<4) | (date % 10);
	month = ((month / 10)<<4) | (month % 10);
	year = ((year / 10)<<4) | (year % 10);
	date &= 0x3F;
	month &= 0x1F;
	uint8_t pack[4] = {day, date, month, year};
	DS3231_write_regs(pack, 0x03, 4);              // write to the module
}

uint8_t DS3231_read_second(void)
{
	uint8_t temp = DS3231_read_reg(0x00);
	temp = (10 * ((temp & 0x70)>>4)) + (temp & 0x0F);
	return temp;
}

uint8_t DS3231_read_minute(void)
{
	uint8_t temp = DS3231_read_reg(0x01);
	temp = (10 * ((temp & 0x70)>>4)) + (temp & 0x0F);
	return temp;
}

uint8_t DS3231_read_hour(void)
{
	uint8_t temp = DS3231_read_reg(0x02);
	temp = (10 * ((temp & 0x30)>>4)) + (temp & 0x0F);
	return temp;
}

uint8_t DS3231_read_day(void)
{
	return DS3231_read_reg(0x03) & 0x07;
}

uint8_t DS3231_read_date(void)
{
	uint8_t temp = DS3231_read_reg(0x04);
	temp = (10 * ((temp & 0x30)>>4)) + (temp & 0x0F);
	return temp;
}

uint8_t DS3231_read_month(void)
{
	uint8_t temp = DS3231_read_reg(0x05);
	temp = (10 * ((temp & 0x10)>>4)) + (temp & 0x0F);
	return temp;
}

uint8_t DS3231_read_year(void)
{
	uint8_t temp = DS3231_read_reg(0x06);
	temp = (10 * ((temp & 0xF0)>>4)) + (temp & 0x0F);
	return temp;
}

ISR(TIMER0_OVF_vect)
{
	setRegOutput(0); // disable outputs
	PORTC &= ~(1<<previousIndPlace); // disable previous symbol place
	if (dispMode == 1)
	{
		shiftByte(charsPlaceBuff[dynamicIndPlace]); // load byte into shift register
	}
	else
	{
		shiftByte(numbers[numbersPlaceBuff[dynamicIndPlace]]); // load byte into shift register
	}
	PORTC |= (1<<dynamicIndPlace); // enable next symbol place
	previousIndPlace = dynamicIndPlace;
	dynamicIndPlace++;
	if (dynamicIndPlace > 3)
	{
		dynamicIndPlace = 0;
	}
}

int main(void)
{
	DDRD |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4));  //setup outputs for shift register
    /*
	* Shift regiser 74hc595
	*
	* Connections:
	*
	* PD0 - DS    (MOSI)
	* PD1 - STCP  (storage register clock)
	* PD2 - SHCP  (shift register clock)
	* PD3 - MR    (master reset)
	* PD4 - OE    (output enable)
	*
	*/
	resetShiftReg();
	
	DDRC |= ((1<<0) | (1<<1) | (1<<2) | (1<<3)); // transistors for dynamic indication (npn)
	PORTC &= ~((1<<0) | (1<<1) | (1<<2) | (1<<3));
	
	TCNT0 = 0;
	TCCR0 |= (1<<CS01); // setup prescaler 8 for timer 0
	TIMSK |= (1<<TOIE0);
	sei();
	
	// TWI setup beginning
	//DDRC &= ~((1<<5) | (1<<4));
	//PORTC |= ((1<<5) | (1<<4));
	TWI_Init();
	// TWI setup ending
	
	uint8_t dt, mn;
	dt = DS3231_read_date();
	mn = DS3231_read_month();
	showTime(dt, mn);
	_delay_ms(2000);
    while (1) 
    {
		uint8_t min, hour;
		min = DS3231_read_minute();
		hour = DS3231_read_hour();
		showTime(hour, min);
		_delay_ms(100);
    }
}
