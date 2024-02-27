#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"

void PORT_init (void)
{
	/*!
	 * Pins definitions
	 * ===================================================
	 *
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTD0              | GPIO [BLUE LED]
	 * PTD15             | GPIO [RED LED]
	 * PTD16			 | GPIO [GREEN LED]
	 */
  PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTD */
  PORTD->PCR[0]  = PORT_PCR_MUX(1);   /* Port D0: MUX = GPIO  */
  PORTD->PCR[15] = PORT_PCR_MUX(1);  /* Port D15: MUX = GPIO */
  PORTD->PCR[16] = PORT_PCR_MUX(1);  /* Port D16: MUX = GPIO */

  PTD->PDDR |= 1<<0		/* Port D0:  Data Direction = output */
  	  	  	  |1<<15	    /* Port D15: Data Direction = output */
  	  	  	  |1<<16;    /* Port D16: Data Direction = output */
}

void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

int main(void)
{
  uint32_t adcResultInMv=0;	/*< ADC0 Result in miliVolts */

	/*!
	 * Initialization:
	 * =======================
	 */
  WDOG_disable();        /* Disable WDOG												*/
  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal 				*/
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC 				*/
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash*/
  PORT_init();		     /* Init  port clocks and gpio outputs 						*/
  ADC_init();            /* Init ADC resolution 12 bit									*/

	/*!
	 * Infinite for:
	 * ========================
	 */
	  for(;;)
	  {
		convertAdcChan(13);                   /* Convert Channel AD13 to pot on EVB 	*/
		while(adc_complete()==0){}            /* Wait for conversion complete flag 	*/
		adcResultInMv = read_adc_chx();       /* Get channel's conversion results in mv */

		if (adcResultInMv > 3750) {           /* If result > 3.75V 		*/
		  PTD->PSOR |= 1<<0 | 1<<16;    /* turn off blue, green LEDs */
		  PTD->PCOR |= 1<<15;              /* turn on red LED 			*/
		}
		else if (adcResultInMv > 2500) {      /* If result > 2.50V 		*/
		  PTD->PSOR |= 1<<0 | 1<<15;    /* turn off blue, red LEDs 	*/
		  PTD->PCOR |= 1<<16;     	      /* turn on green LED 		*/
		}
		else if (adcResultInMv >1250) {       /* If result > 1.25V 		*/
		  PTD->PSOR |= 1<<15 | 1<<16;   /* turn off red, green LEDs  */
		  PTD->PCOR |= 1<<0;     	      /* turn on blue LED 			*/
		}
		else {
		  PTD->PSOR |= 1<<0 | 1<<15 | 1<<16; /* Turn off all LEDs */
		}

	  }
}
