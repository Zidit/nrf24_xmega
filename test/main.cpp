/*
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "serial.h"
#include "utils.h"
#include "../nrf24.h"
#include "../spi.h"

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
	
	debug.sendHex(nrf.getStatus());
	
    while(1)
    {
        uint8_t data;
	    uint8_t asd[10];
        if(debug.dataAvailable())
        {

			data = debug.getChar();
			asd[0] = 0x0A;
			nrfSpi.transmit(asd,6);
			nrfSpi.flush();
			
			for (int i = 0; i < 6; i++)
				debug.sendHex(asd[i]);
				
			debug.sendChar('\n');
        }

    }

    return 0;
}


