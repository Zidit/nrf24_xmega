

#include "spi.h"

spiDriver::spiDriver(SPI_t* spi, PORT_t* spiPort, const uint8_t mosiPin, const uint8_t clkPin) : _spi(spi)
{
   _spi->CTRL = SPI_MASTER_bm | SPI_MODE0_bm | SPI_PRESCALER_DIV16_gc ;
    SPIC.CTRL = SPI_MASTER_bm | SPI_MODE0_bm | SPI_PRESCALER_DIV16_gc ;
    spiPort->DIRSET = (1 << mosiPin) | (1 << clkPin);
}

void spiDriver::setCsPin(PORT_t* csPort, const uint8_t csPin)
{
    _csPort = csPort;
    _csPinBm = 1 << csPin;

    _csPort->DIRSET = _csPinBm;
    _csPort->OUTSET = _csPinBm;
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
    _csPort->OUTCLR = _csPinBm;
    _isTransmitting = true;
    _spi->INTCTRL = SPI_INTLVL_MED_gc;
    _dataPtr = data;
    _dataLen = len;
    _bytesSent = 0;
    _spi->DATA = _dataPtr[0];
}

uint8_t spiDriver::transmit(uint8_t data)
{
    _csPort->OUTCLR = _csPinBm;
    _spi->DATA = data;
    while (!(_spi->STATUS & SPI_IF_bm));

    _csPort->OUTSET = _csPinBm;
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
    while(!_transmitReady && _isTransmitting);
}


