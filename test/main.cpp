/*
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "serial.h"
#include "utils.h"
#include "../nrf24.h"
#include "../spi.h"

uart debug(&USARTC0, 57600);
ISR (USARTC0_RXC_vect){ debug.rxInterrupt(); }
ISR (USARTC0_DRE_vect){ debug.txInterrupt(); }

spiDriver nrfSpi(&SPIC, &PORTC, 5, 6, 7);
nrf24 nrf(&nrfSpi);


ISR (SPIC_INT_vect) { if(nrfSpi.interrupt()) nrf.spiInterrupt(); }
ISR (PORTC_INT0_vect) {};




int main(void)
{
	srand(12);
    // Set sys clock to 16 Mhz
    // -if you change this, remember to change F_CUP define too
    set32MHzClock(CLK_PSADIV_2_gc);

    //Enable all interrupts
    PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();



    debug.sendStringPgm(PSTR("\n\n\nTest \n"));



    char asd[] = "asdlol";

    while(1)
    {
        uint8_t data;
        if(debug.dataAvailable())
        {
            data = debug.getChar();
            debug.sendString(asd);
            debug.sendChar('\n');
            nrfSpi.transmit((uint8_t*)asd, 6);
            nrfSpi.flush();
            debug.sendString(asd);
            debug.sendChar('\n');
            debug.sendChar('\n');
        }

    }

    return 0;
}


