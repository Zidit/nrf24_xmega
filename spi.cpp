

#include "spi.h"
#include <util/delay.h>

spiDriver::spiDriver(SPI_t* spi, PORT_t* spiPort, const uint8_t ssPin, const uint8_t mosiPin, const uint8_t misoPin, const uint8_t clkPin) : _spi(spi)
{
    _spi->CTRL = SPI_MASTER_bm | SPI_MODE0_bm | SPI_PRESCALER_DIV16_gc;
    spiPort->DIRSET = (1 << mosiPin) | (1 << clkPin);
    spiPort->DIRCLR = (1 << misoPin);

	_ssPinBm = 1 << ssPin;
    _spiPort = spiPort;
		
    _spiPort->DIRSET = _ssPinBm;
    _spiPort->OUTSET = _ssPinBm;
}



void spiDriver::setSpeed(SPI_PRESCALER_enum prescaler, bool doubleSpeed)
{
    _spi->CTRL &= ~(SPI_PRESCALER_gm | SPI_CLK2X_bm);
    _spi->CTRL |= prescaler;
    if (doubleSpeed) _spi->CTRL |= SPI_CLK2X_bm;
}

void spiDriver::setMasterMode(bool master)
{
    if(master) _spi->CTRL |= SPI_MASTER_bm;
    else _spi->CTRL &= ~ SPI_MASTER_bm;
}

void spiDriver::setMode(SPI_MODE_enum mode)
{
    _spi->CTRL &= ~(SPI_MODE_gm);
    _spi->CTRL |= mode;
}

void spiDriver::transmit(uint8_t* data, uint8_t len)
{
    _isTransmitting = true;
	_spiPort->OUTCLR = _ssPinBm;
    _dataPtr = data;
    _dataLen = len;
    _bytesSent = 0;
    _spi->INTCTRL = SPI_INTLVL_MED_gc;
    _spi->DATA = _dataPtr[0];

}

uint8_t spiDriver::transmit(uint8_t data)
{
    _spiPort->OUTCLR = _ssPinBm;
    _spi->DATA = data;
    while (!(_spi->STATUS & SPI_IF_bm));

	_spiPort->OUTSET = _ssPinBm;
    return _spi->DATA;
}

bool spiDriver::transmitReady()
{
    bool ret = _transmitReady;
    _transmitReady = false;
    return ret;
}

void spiDriver::flush()
{
    while(_isTransmitting);
}


