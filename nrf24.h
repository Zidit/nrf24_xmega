
#ifndef INCLUDE_NRF24
#define INCLUDE_NRF24

#include <avr/io.h>
#include "spi.h"



class nrf24
{
public:
    nrf24(spiDriver* spi, PORT_t* csnPort, const uint8_t csnPin);
    void setIqrPin(PORT_t* iqrPort, const uint8_t iqrPin);
    void setCePin(PORT_t* cePort, const uint8_t cePin);

    void setRegister(uint8_t* data, uint8_t len);
    void getRegister(uint8_t* data, uint8_t len);
    void sendData(uint8_t* data, uint8_t len);
    void reciveData(uint8_t* data, uint8_t len);

    uint8_t getStatus();

    void spiInterrupt();
    void pinInterrupt();

private:
    spiDriver* _spi;

    PORT_t* _iqrPort;
    uint8_t _iqrPin;
    PORT_t* _cePort;
    uint8_t _cePin;
};


#define R_REGISTER      0b00000000
#define W_REGISTER      0b00100000
#define R_RX_PAYLOAD    0b01100001
#define W_TX_PAYLOAD    0b10100000
#define FLUSH_TX        0b11100001
#define FLUSH_RX        0b11100010
#define REUSE_TX_PL     0b11100011
#define NOP             0b11111111













#endif
