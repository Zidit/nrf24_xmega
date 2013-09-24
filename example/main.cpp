/*
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "serial.h"
#include "../nrf24.h"
#include "../spi.h"


// TX = PC3, RX = PC2

uart debug(&USARTC0, 57600); 
ISR (USARTC0_RXC_vect){ debug.rxInterrupt(); }
ISR (USARTC0_DRE_vect){ debug.txInterrupt(); }



/*
nRF 0
Miso = PC6
Mosi = PC5
clk  = PC7
csn  = PC4
irq	 = PC1
ce	 = PC0
*/

spiDriver nrfSpi(&SPIC, &PORTC, 5, 6, 7);
nrf24 nrf0(&nrfSpi, &PORTC, 4, &PORTC, 1, &PORTC, 0);

ISR (PORTC_INT0_vect) {nrf0.pinInterrupt();}
ISR (SPIC_INT_vect) { if(nrfSpi.interrupt()) nrf0.spiInterrupt();}



int main(void)
{

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


    debug.sendStringPgm(PSTR("\n\n\nBasic example of half-duplex communication\n"));
	
	#define PAYLOAD_SIZE 8

	nrf0.primaryRx();

	nrf0.setRegister(NRF_RX_PW_P0, PAYLOAD_SIZE);
	nrf0.setRegister(NRF_RX_PW_P1, PAYLOAD_SIZE);
	nrf0.setRegister(NRF_SETUP_RETR, 0x33);
	
	
	uint8_t rx_data[PAYLOAD_SIZE + 1];
	uint8_t tx_data[PAYLOAD_SIZE + 1];

	uint8_t tx_bytes = 0;



    while(1)
    {
		while(nrf0.getState() != rx_idle);
		nrf0.reciveData(rx_data,PAYLOAD_SIZE);
		
		while(nrf0.getState() != rx_idle) 
		{	
			
			if(debug.dataAvailable())
			{
			    		
				tx_data[tx_bytes + 1] = debug.getChar();
				tx_bytes++;
				
				if(tx_bytes == PAYLOAD_SIZE)
				{
					debug.sendStringPgm(PSTR("Sending: "));
					for(uint8_t i = 0; i < PAYLOAD_SIZE; i++)
						debug.sendChar(tx_data[i+1]);			
					
					debug.sendChar(' ');
					
					tx_bytes = 0;
					nrf0.primaryTx();
					
					nrf0.sendData(tx_data, PAYLOAD_SIZE);	
					while(nrf0.getState() != tx_idle);
					
					debug.sendHex(tx_data[0]);
					
					if(tx_data[0] & NRF_TX_DS_bm) debug.sendStringPgm(PSTR(" ok\n"));
					else {
						debug.sendStringPgm(PSTR(" failed\n"));
						nrf0.flushTx();
					}
					
					nrf0.primaryRx();
					nrf0.reciveData(rx_data,PAYLOAD_SIZE);
					
				}
			}		
		}
		
		debug.sendStringPgm(PSTR("Recived: "));
		
		for(uint8_t i = 0; i < PAYLOAD_SIZE; i++)
			debug.sendChar(rx_data[i+1]);

		debug.sendChar('\n');
		
	}

    return 0;
}
