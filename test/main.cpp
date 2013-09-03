/*
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "serial.h"
#include "utils.h"
#include "../nrf24.h"
#include "../spi.h"

void printAllRegisters();
void printRegister(const char *str, uint8_t reg, uint8_t len);

// TX = PD3, RX = PD2

uart debug(&USARTD0, 57600); 
ISR (USARTD0_RXC_vect){ debug.rxInterrupt(); }
ISR (USARTD0_DRE_vect){ debug.txInterrupt(); }


/*
Miso = PC6
Mosi = PC5
clk  = PC7
csn  = PC4
irq	 = PC3



*/
spiDriver nrfSpi(&SPIC, &PORTC, 4, 5, 6, 7);
nrf24 nrf(&nrfSpi);


//ISR (SPIC_INT_vect) { if(nrfSpi.interrupt(&SPIC)) nrf.spiInterrupt(); }
ISR (PORTC_INT0_vect) {nrf.pinInterrupt();}
ISR (SPIC_INT_vect) { nrfSpi.interrupt();}





int main(void)
{
	srand(12);

	//Config external 16 MHz clock
	
	OSC.XOSCCTRL = 0x17; 
	OSC.CTRL = OSC_XOSCEN_bm;
	while (!(OSC.STATUS & OSC_XOSCRDY_bm));
	
	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = 0;
	
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_XOSC_gc;

	
    //Enable all interrupts
    PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();



    debug.sendStringPgm(PSTR("\n\n\nTest \n"));
	
	_delay_ms(200);
	
	printAllRegisters();

	
    while(1)
    {
        uint8_t data;
        if(debug.dataAvailable())
        {

			data = debug.getChar();

        }

    }

    return 0;
}

void printAllRegisters()
{
	printRegister(PSTR("Config"), NRF_CONFIG, 1);
	printRegister(PSTR("En AA"), NRF_EN_AA, 1);
	printRegister(PSTR("En rx pipe"), NRF_EN_RXADDR, 1);
	printRegister(PSTR("Aw"), NRF_SETUP_AW, 1);
	printRegister(PSTR("Re trans"), NRF_SETUP_RETR, 1);
	printRegister(PSTR("RH ch"), NRF_RF_CH, 1);
	printRegister(PSTR("RF setup"), NRF_RF_SETUP, 1);
	printRegister(PSTR("Status"), NRF_STATUS, 1);
	printRegister(PSTR("Obs tx"), NRF_OBSERVE_TX, 1);
	printRegister(PSTR("RPD"), NRF_RPD, 1);
	printRegister(PSTR("RX0 addr"), NRF_RX_ADDR_P0, 5);
	printRegister(PSTR("RX1 addr"), NRF_RX_ADDR_P1, 5);
	printRegister(PSTR("RX2 addr"), NRF_RX_ADDR_P2, 1);
	printRegister(PSTR("RX3 addr"), NRF_RX_ADDR_P3, 1);
	printRegister(PSTR("RX4 addr"), NRF_RX_ADDR_P4, 1);
	printRegister(PSTR("RX5 addr"), NRF_RX_ADDR_P5, 1);
	printRegister(PSTR("TX addr"), NRF_TX_ADDR, 5);
	
	printRegister(PSTR("RX0 pl"), NRF_RX_PW_P0, 1);
	printRegister(PSTR("RX1 pl"), NRF_RX_PW_P1, 1);
	printRegister(PSTR("RX2 pl"), NRF_RX_PW_P2, 1);
	printRegister(PSTR("RX3 pl"), NRF_RX_PW_P3, 1);
	printRegister(PSTR("RX4 pl"), NRF_RX_PW_P4, 1);
	printRegister(PSTR("RX5 pl"), NRF_RX_PW_P5, 1);
	
	printRegister(PSTR("FIFO"), NRF_FIFO_STATUS, 1);
	printRegister(PSTR("Dyn pl"), NRF_DYNPD, 1);
	printRegister(PSTR("Feature"), NRF_FEATURE, 1);
	
}

void printRegister(const char *str, uint8_t reg, uint8_t len)
{
	uint8_t data[5];
	nrf.getRegister(reg, data, len);
	
	debug.sendStringPgm(str);
	debug.sendString(": ");
	
	for (uint8_t i = 0; i < len; i++)
		debug.sendHex(data[i]);
		
	debug.sendChar('\n');
}
