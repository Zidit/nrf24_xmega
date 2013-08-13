
#ifndef INCLUDE_SPI
#define INCLUDE_SPI

#include <avr/io.h>



class spiDriver
{
public:
    spiDriver(SPI_t* const spi, PORT_t* const spiPort, const uint8_t mosiPin, const uint8_t misoPin, const uint8_t clkPin);
    void setSpeed(const SPI_PRESCALER_enum prescaler, const bool doubleSpeed);
    void setMode(const SPI_MODE_enum mode);
    void setMasterMode(const bool master);
    void enable() {_spi->CTRL |= SPI_ENABLE_bm;}
    void disable() {_spi->CTRL &= ~SPI_ENABLE_bm;}

    void transmit(uint8_t* const data, const uint8_t len);
    uint8_t transmit(const uint8_t data);

    bool isTransmitting() {return _isTransmitting;}
    bool transmitReady();
    void flush();

    bool interrupt()
    {
        _dataPtr[_bytesSent] = _spi->DATA;
        _bytesSent++;
        if(_dataLen == _bytesSent)
        {
            _transmitReady = true;
            _isTransmitting = false;
            _spi->INTCTRL = SPI_INTLVL_OFF_gc;
            return true;
        }
        else
        {
            _spi->DATA = _dataPtr[_bytesSent];
            return false;
        }
    }



private:
    SPI_t* const _spi;

    uint8_t* _dataPtr;
    volatile bool _transmitReady;
    volatile bool _isTransmitting;
    volatile uint8_t _bytesSent;
    volatile uint8_t _dataLen;
};






#endif
