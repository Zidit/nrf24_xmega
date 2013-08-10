
#ifndef INCLUDE_SPI
#define INCLUDE_SPI

#include <avr/io.h>

void __attribute__((weak)) spiHookTransmitReady();

class spi
{
public:
    spi(SPI_t* spi, PORT_t* spiPort, const uint8_t mosiPin, const uint8_t clkPin);
    void setCsPin(PORT_t* port, const uint8_t pin_nbr);
    void setSpeed(SPI_PRESCALER_enum prescaler, bool doubleSpeed);
    void setMode(SPI_MODE_enum mode);
    void setMasterMode(bool master);
    void enable() {_spi->CTRL |= SPI_ENABLE_bm;}
    void disable() {_spi->CTRL &= ~SPI_ENABLE_bm;}

    void transmit(uint8_t* data, const uint8_t len);
    uint8_t transmit(const uint8_t data);

    bool isTransmitting() {return _isTransmitting;}
    bool transmitReady();
    void flush();

    void interrupt()
    {
        _dataPtr[_bytesSent] = _spi->DATA;
        _bytesSent++;
        if(_dataLen == _bytesSent)
        {
            _transmitReady = true;
            _isTransmitting = false;
            _csPort->OUTSET = _csPinBm;
            _spi->INTCTRL = SPI_INTLVL_OFF_gc;
            spiHookTransmitReady();
            return;
        }
        else
        {
            _spi->DATA = _dataPtr[_bytesSent];
        }
    }


private:
    SPI_t* _spi;
    PORT_t* _csPort;
    uint8_t _csPinBm;

    uint8_t* _dataPtr;
    volatile bool _transmitReady;
    volatile bool _isTransmitting;
    volatile uint8_t _bytesSent;
    volatile uint8_t _dataLen;
};






#endif
