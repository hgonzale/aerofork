#ifndef _AEROQUAD_SPI_HARDWARESPIEXT_H_
#define _AEROQUAD_SPI_HARDWARESPIEXT_H_

//#if defined(AeroQuadSTM32)

// helper class to extend the maple HardwareSPI class
// used by the MPU6000 library

#include <SPI.h>

#define SPI_READ_FLAG  0x80
#define SPI_MULTI_FLAG 0x40

#define SetPin digitalWrite
#define DOUT     51 //MOSI
#define DIN      50 //MISO
#define SCK      52 //SCLK
#define OSD_CS   22 //SS_OSD on AeroQuad v2.x shield

class HardwareSPIExt {

public:

	void setBitOrder(uint8_t bitOrder) {
		if(bitOrder == LSBFIRST) {
		SPCR |= _BV(DORD);
		} 
		else {
			SPCR &= ~(_BV(DORD));
		}
	}

	void setDataMode(uint8_t mode) {
		SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
	}

	void setClockDivider(uint8_t rate) {
		SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
		SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
	}

	void begin(uint8_t clockdivider, uint8_t bitOrder, uint8_t mode) {   

		pinMode( 53, OUTPUT ); //Default CS pin for MPU6000 motion sensor - needs to be output or SPI peripheral will behave as a slave instead of master
		pinMode( 40, OUTPUT ); //Default CS pin for MS5611 barometer - needs to be output or SPI peripheral will behave as a slave instead of master
		pinMode( DOUT, OUTPUT );
		pinMode( DIN, INPUT );
		pinMode( SCK, OUTPUT );

		pinMode( OSD_CS, OUTPUT );
		digitalWrite( OSD_CS, HIGH );

		// Warning: if the SS pin ever becomes a LOW INPUT then SPI
		// automatically switches to Slave, so the data direction of
		// the SS pin MUST be kept as OUTPUT.
		SPCR |= _BV(MSTR);
		SPCR |= _BV(SPE);

		setClockDivider(clockdivider);
		setBitOrder(bitOrder);
		setDataMode(mode);

		digitalWrite(53, HIGH);
		digitalWrite(40, HIGH);

	}

	byte spi_transfer(byte data) {

	 SPDR = data; //transfer data with hardware SPI
	 while (!(SPSR & _BV(SPIF))) ;
	 return SPDR;

	}

	void Read(int addr, unsigned char *data, int dataLen) {

		while(digitalRead(40) == 0) {

		}

		SetPin(53, 0);
		spi_transfer(addr | SPI_READ_FLAG);

		while(dataLen-- > 0) {

			*data++ = spi_transfer(0);

		}

		SetPin(53, 1);

	}

	unsigned char Read(int addr) {

		unsigned char data;
		Read(addr, &data, 1);

		return data;

	}

	void Write(int addr, unsigned char *data, int dataLen) {

		while(digitalRead(40) == 0) {

		}

		SetPin(53, 0);
		spi_transfer(addr);
		
		while(dataLen-- > 0) {

			spi_transfer(*data++);

		}

		SetPin(53, 1);

	}

	void Write(int addr, unsigned char data) {

		Write(addr, &data, 1);

	}

	//for MS5611 Barometer
	uint8_t spi_read(uint8_t reg) {
		
		uint8_t return_value;
		uint8_t addr = reg; // | 0x80; // Set most significant bit
		
		while(digitalRead(53) == 0) {

		}

		digitalWrite(40, LOW);
		spi_transfer(addr); // discarded
		return_value = spi_transfer(0);
		digitalWrite(40, HIGH);

		return return_value;

	}
	
	uint16_t spi_read_16bits(uint8_t reg) {

		uint8_t byteH, byteL;
		uint16_t return_value;
		uint8_t addr = reg; // | 0x80; // Set most significant bit
		
		while(digitalRead(53) == 0) {

		}

		digitalWrite(40, LOW);
		spi_transfer(addr); // discarded
		byteH = spi_transfer(0);
		byteL = spi_transfer(0);
		digitalWrite(40, HIGH);

		return_value = ((uint16_t)byteH<<8) | (byteL);
		return return_value;

	}
	
	uint32_t spi_read_adc() {

		uint8_t byteH,byteM,byteL;
		uint32_t return_value;
		uint8_t addr = 0x00;

		while(digitalRead(53) == 0) {

		}

		digitalWrite(40, LOW);
		spi_transfer(addr); // discarded
		byteH = spi_transfer(0);
		byteM = spi_transfer(0);
		byteL = spi_transfer(0);
		digitalWrite(40, HIGH);

		return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
		return return_value;

	}
	

	
	void spi_write(uint8_t reg) {

		while(digitalRead(53) == 0) {

		}

		digitalWrite(40, LOW);
		spi_transfer(reg); // discarded
		digitalWrite(40, HIGH);

	}
	//

	void end() {
		SPCR &= ~_BV(SPE);
	}



};


#endif