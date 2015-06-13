#ifndef _AEROQUAD_SPI_ARDUPILOTSPIEXT_H_
#define _AEROQUAD_SPI_ARDUPILOTSPIEXT_H_

// helper class for executing SPI protocols on the APM2.5 flight processor
// extends SPI.h from Arduino SPI library
// protocols taken from the Arduino SPI library and AP_Baro_MS5611.cpp
// used by the MPU6000 and MS5611 library

#include <AQ_SPI\SPI.h>

#define SPI_READ_FLAG  0x80
#define SPI_MULTI_FLAG 0x40

#define DOUT     51 //MOSI on APM2.5 flight processor
#define DIN      50 //MISO on APM2.5 flight processor
#define SCK      52 //SCK on APM2.5 flight processor
#define MPUCS    53 //Default CS pin for MPU6000 motion sensor - needs to be output or SPI peripheral will behave as a slave instead of master
#define BAROCS   40 //Default CS pin for MS5611 barometer - needs to be output or SPI peripheral will behave as a slave instead of master

class ArdupilotSPIExt {

public:

	//SPI initiation sequence taken from Arduino SPI library
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

		pinMode( MPUCS, OUTPUT ); 
		pinMode( BAROCS, OUTPUT ); 
		pinMode( DOUT, OUTPUT );
		pinMode( DIN, INPUT );
		pinMode( SCK, OUTPUT );

		// Warning: if the CS pin ever becomes a LOW INPUT then SPI
		// automatically switches to Slave, so the data direction of
		// the SS pin MUST be kept as OUTPUT.
		SPCR |= _BV(MSTR);
		SPCR |= _BV(SPE);

		setClockDivider(clockdivider);
		setBitOrder(bitOrder);
		setDataMode(mode);

		digitalWrite(MPUCS, HIGH);
		digitalWrite(BAROCS, HIGH);

	}

	byte spi_transfer(byte data) {

	 SPDR = data; //transfer data with hardware via SPI
	 while (!(SPSR & _BV(SPIF))) ;
	 return SPDR;

	}

	//SPI Read protocol for MPU6000 device
	void Read(int addr, unsigned char *data, int dataLen) {

		while(digitalRead(BAROCS) == 0) {
			//to prevent simultaneous SPI communications with both MPU6000 and MS5611
		} 

		digitalWrite(MPUCS, LOW);
		spi_transfer(addr | SPI_READ_FLAG);

		while(dataLen-- > 0) {

			*data++ = spi_transfer(0);

		}

		digitalWrite(MPUCS, HIGH);

	}
	
	unsigned char Read(int addr) {

		unsigned char data;
		Read(addr, &data, 1);

		return data;

	}
	//SPI Read protocol for MPU6000 device

	//SPI Write protocol for MPU6000 device
	void Write(int addr, unsigned char *data, int dataLen) {

		while(digitalRead(BAROCS) == 0) {
			//to prevent simultaneous SPI communications with both MPU6000 and MS5611
		}

		digitalWrite(MPUCS, LOW);
		spi_transfer(addr);
		
		while(dataLen-- > 0) {

			spi_transfer(*data++);

		}

		digitalWrite(MPUCS, HIGH);

	}

	void Write(int addr, unsigned char data) {

		Write(addr, &data, 1);

	}
	//SPI Write protocol for MPU6000 device



	//8-bit SPI Read protocol for MS5611 Barometer
	uint8_t spi_read(uint8_t reg) {
		
		uint8_t return_value;
		uint8_t addr = reg; 
		
		while(digitalRead(MPUCS) == 0) {

		}

		digitalWrite(BAROCS, LOW);

		spi_transfer(addr); 

		return_value = spi_transfer(0);

		digitalWrite(BAROCS, HIGH);

		return return_value;

	}
	//8-bit SPI Read protocol for MS5611 Barometer
	
	//16-bit SPI Read protocol for MS5611 Barometer
	uint16_t spi_read_16bits(uint8_t reg) {

		uint8_t byteH, byteL;
		uint16_t return_value;
		uint8_t addr = reg; // | 0x80; // Set most significant bit
		
		while(digitalRead(MPUCS) == 0) {

		}

		digitalWrite(BAROCS, LOW);

		spi_transfer(addr); 

		byteH = spi_transfer(0);
		byteL = spi_transfer(0);

		digitalWrite(40, HIGH);

		return_value = ((uint16_t)byteH<<8) | (byteL);

		return return_value;

	}
	//16-bit SPI Read protocol for MS5611 Barometer
	
	//Reads 24-bit sensor value from MS5611 Barometer
	uint32_t spi_read_adc() {

		uint8_t byteH,byteM,byteL;
		uint32_t return_value;
		uint8_t addr = 0x00; //register address on MS5611 for reading from its ADC

		while(digitalRead(MPUCS) == 0) {

		}

		digitalWrite(BAROCS, LOW);

		spi_transfer(addr); 

		byteH = spi_transfer(0);
		byteM = spi_transfer(0);
		byteL = spi_transfer(0);

		digitalWrite(BAROCS, HIGH);

		return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
		return return_value;

	}
	//Reads 24-bit sensor value from MS5611 Barometer	

	//8-bit SPI Write protocol for MS5611 Barometer	
	void spi_write(uint8_t reg) {

		while(digitalRead(MPUCS) == 0) {

		}

		digitalWrite(BAROCS, LOW);

		spi_transfer(reg); 

		digitalWrite(BAROCS, HIGH);

	}
	//8-bit SPI Write protocol for MS5611 Barometer	

	void end() {
		SPCR &= ~_BV(SPE);
	}



};


#endif