/*
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "serial.h"
#include "utils.h"
#include "../spi.h"

uart debug(&USARTC0, 57600);
ISR (USARTC0_RXC_vect){ debug.rxInterrupt(); }
ISR (USARTC0_DRE_vect){ debug.txInterrupt(); }

spi nrf(&SPIC, &PORTC, 5, 7);
ISR (SPIC_INT_vect) { nrf.interrupt();}


int main(void)
{
	srand(12);
    // Set sys clock to 16 Mhz
    // -if you change this, remember to change F_CUP define too
    set32MHzClock(CLK_PSADIV_2_gc);

    //Enable all interrupts
    PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

    nrf.setMasterMode(true);
    nrf.setMode(SPI_MODE_0_gc);
    nrf.setSpeed(SPI_PRESCALER_DIV16_gc, false);
    nrf.setCsPin(&PORTC, 4);
    nrf.enable();

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
            nrf.transmit((uint8_t*)asd, 6);
            nrf.flush();
            debug.sendString(asd);
            debug.sendChar('\n');
            debug.sendChar('\n');
        }

    }

    return 0;
}
/*
void spiHookTransmitReady()
{
    debug.sendChar('T');


}*/

