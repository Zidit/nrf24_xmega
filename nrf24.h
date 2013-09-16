
#ifndef INCLUDE_NRF24
#define INCLUDE_NRF24

#include <avr/io.h>
#include "spi.h"

enum nrf_state {off, rx_idle, rx_set_reg, rx_listen, rx_read, rx_recived, tx_idle, tx_set_reg, tx_send, tx_wait_ack, tx_ok, tx_fail};

typedef struct {
	uint8_t status;
	uint8_t data[32];
} nrf_packet;


class nrf24
{
public:
    nrf24(spiDriver* const spi, PORT_t* const ssPort, const uint8_t ssPin, PORT_t* const iqrPort, const uint8_t iqrPin, PORT_t* const cePort, const uint8_t cePin);
	
	void primaryRx();
	void primaryTx();
	void powerOff();

    void setRegister(uint8_t reg, uint8_t* data, uint8_t len);
    void getRegister(uint8_t reg, uint8_t* data, uint8_t len);
    void sendData(nrf_packet* data, uint8_t payload_len);
	void reciveData(nrf_packet* data, uint8_t payload_len);
	
	void flushTx();
	void flushRx();
    uint8_t getStatus();
	nrf_state getState() {return state;}

    void spiInterrupt();
    void pinInterrupt();
	
private:
    spiDriver* _spi;
	nrf_state state;

    PORT_t* _iqrPort;
    uint8_t _iqrPinBm;

    PORT_t* _cePort;
    uint8_t _cePinBm;
	
	PORT_t* _ssPort;
    uint8_t _ssPinBm;
	
	uint8_t _buffer[32];
	
	nrf_packet* rx_buffer;
	uint8_t		rx_buffer_len;

	void setIqrPin(PORT_t* const iqrPort, const uint8_t iqrPin);
    void setCePin(PORT_t* const cePort, const uint8_t cePin);
	void setSsPin(PORT_t* const ssPort, const uint8_t ssPin);

};


#define NRF_R_REGISTER      0b00000000
#define NRF_W_REGISTER      0b00100000
#define NRF_R_RX_PAYLOAD    0b01100001
#define NRF_W_TX_PAYLOAD    0b10100000
#define NRF_FLUSH_TX        0b11100001
#define NRF_FLUSH_RX        0b11100010
#define NRF_REUSE_TX_PL     0b11100011
#define NRF_NOP             0b11111111


#define MODE_TRANSMIT	0
#define MODE_RECIVE		1


#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_OBSERVE_TX  0x08
#define NRF_RPD         0x09
#define NRF_RX_ADDR_P0  0x0A
#define NRF_RX_ADDR_P1  0x0B
#define NRF_RX_ADDR_P2  0x0C
#define NRF_RX_ADDR_P3  0x0D
#define NRF_RX_ADDR_P4  0x0E
#define NRF_RX_ADDR_P5  0x0F
#define NRF_TX_ADDR     0x10
#define NRF_RX_PW_P0    0x11
#define NRF_RX_PW_P1    0x12
#define NRF_RX_PW_P2    0x13
#define NRF_RX_PW_P3    0x14
#define NRF_RX_PW_P4    0x15
#define NRF_RX_PW_P5    0x16
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD		0x1C
#define NRF_FEATURE		0x1D


#define NRF_MASK_RX_DR_bm  (1 << 6)
#define NRF_MASK_TX_DS_bm  (1 << 5)
#define NRF_MASK_MAX_RT_bm (1 << 4)
#define NRF_EN_CRC_bm      (1 << 3)
#define NRF_CRCO_bm        (1 << 2)
#define NRF_PWR_UP_bm      (1 << 1)
#define NRF_PRIM_RX_bm     (1 << 0)
#define NRF_ENAA_P5_bm     (1 << 5)
#define NRF_ENAA_P4_bm     (1 << 4)
#define NRF_ENAA_P3_bm     (1 << 3)
#define NRF_ENAA_P2_bm     (1 << 2)
#define NRF_ENAA_P1_bm     (1 << 1)
#define NRF_ENAA_P0_bm     (1 << 0)
#define NRF_ERX_P5_bm      (1 << 5)
#define NRF_ERX_P4_bm      (1 << 4)
#define NRF_ERX_P3_bm      (1 << 3)
#define NRF_ERX_P2_bm      (1 << 2)
#define NRF_ERX_P1_bm      (1 << 1)
#define NRF_ERX_P0_bm      (1 << 0)
#define NRF_AW_bm          (3 << 0)
#define NRF_ARD_bm         (0xF << 4)
#define NRF_ARC_bm         (0xF << 0)
#define NRF_RF_CH_bm       (0x7F << 0) 
#define NRF_CONT_WAVE_bm   (1 << 7)
#define NRF_RF_LOW_bm      (1 << 5)
#define NRF_PLL_LOCK_bm    (1 << 4)
#define NRF_RF_DR_HIGH_bm  (1 << 3)
#define NRF_RF_PWR_bm      (3 << 1)
#define NRF_RX_DR_bm       (1 << 6)
#define NRF_TX_DS_bm       (1 << 5)
#define NRF_MAX_RT_bm      (1 << 4)
#define NRF_RX_P_NO_bm     (7 << 1)
#define NRF_TX_FULL_bm     (1 << 0)
#define NRF_PLOS_CNT_bm    (0xF << 4)
#define NRF_ARC_CNT_bm     (0xF << 0)
#define NRF_RPD_bm     	   (1 << 0)
#define NRF_RX_PW_P0_bm    (0x1F << 0)
#define NRF_RX_PW_P1_bm    (0x1F << 0)
#define NRF_RX_PW_P2_bm    (0x1F << 0)
#define NRF_RX_PW_P3_bm    (0x1F << 0)
#define NRF_RX_PW_P4_bm    (0x1F << 0)
#define NRF_RX_PW_P5_bm    (0x1F << 0)
#define NRF_TX_REUSE_bm    (1 << 6)
#define NRF_TXFIFO_FULL_bm (1 << 5)
#define NRF_TX_EMPTY_bm    (1 << 4)
#define NRF_RX_FULL_bm     (1 << 1)
#define NRF_RX_EMPTY _bm   (1 << 0)
#define NRF_PDL_P5_bm      (1 << 5)
#define NRF_PDL_P4_bm      (1 << 4)
#define NRF_PDL_P3_bm      (1 << 3)
#define NRF_PDL_P2_bm      (1 << 2)
#define NRF_PDL_P1_bm      (1 << 1)
#define NRF_PDL_P0_bm      (1 << 0)
#define NRF_EN_DPL_bm      (1 << 2)
#define NRF_EN_ACK_PAY_bm  (1 << 1)
#define NRF_DYN_ACK_bm     (1 << 0)




#endif
