#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"
#include "LPUART.h"

#define LettuceHM 75
#define LettuceTemp 20
#define TomatoHM 70
#define TomatoTemp 22
#define BeanHM 90
#define BeanTemp 24

int lpit0_ch0_flag_counter, lpit0_ch1_flag_counter, lpit0_ch2_flag_counter = 0;
unsigned int FND_DATA[10]={0x7E, 0x0C,0xB6,0x9E,0xCC,0xDA,0xFA,0x4E,0xFE,0xCE};
unsigned int FND_SEL[4]={0x0100,0x0200,0x0400,0x0800};
unsigned int j=0; /*FND select pin index */
unsigned int num,num0,num1,num2,num3 =0;
unsigned int Dtime = 0;
uint8_t temperature;
uint8_t humidity;
unsigned int BestLight=1;
unsigned int BestHM=LettuceHM;
unsigned int	BestTemp=LettuceTemp;	/*Each Mode, Default=Lettuce*/
unsigned int External_PIN=0;
uint32_t adcResultInMv3,adcResultInMv2=0;
unsigned int flag = 0; /*flag for Water Motor*/


void PORT_init (void){		
		PCC-> PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
		PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
		PCC-> PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK;
	  PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT D */
	
	/* UART PORT MUX ON*/
	
		PORTC->PCR[6]|=PORT_PCR_MUX(2); /* Port C6: MUX = ALT2,UART1 TX */
		PORTC->PCR[7]|=PORT_PCR_MUX(2); /* Port C7: MUX = ALT2,UART1 RX */
	
	/* Interrupt input PORT C10~14*/
	
		PTC->PDDR &= ~(1<<10);		/* Port C10 Port Input set, value '0'*/
		PTC->PDDR &= ~(1<<11);		/* Port C11 Pory Input set, value '0'*/
		PTC->PDDR &= ~(1<<12);		/* Port C12 Port Input set, value '0'*/
		PTC->PDDR &= ~(1<<13);		/* Port C13 Port Input set, value '0'*/
		PTC->PDDR &= ~(1<<9);		/* Port C14 Pory Input set, value '0'*/
	
		PORTC->PCR[10] |= PORT_PCR_MUX(1)|PORT_PCR_PE(1)|PORT_PCR_PS(1); // Port C10 mux = GPIO
		PORTC->PCR[11] |= PORT_PCR_MUX(1)|PORT_PCR_PE(1)|PORT_PCR_PS(1); // Port C11 mux = GPIO
		PORTC->PCR[12] |= PORT_PCR_MUX(1)|PORT_PCR_PE(1)|PORT_PCR_PS(1); // Port C12 mux = GPIO
		PORTC->PCR[13] |= PORT_PCR_MUX(1)|PORT_PCR_PE(1)|PORT_PCR_PS(1); // Port C13 mux = GPIO
		PORTC->PCR[9] |= PORT_PCR_MUX(1)|PORT_PCR_PE(1)|PORT_PCR_PS(1); // Port C14 mux = GPIO
	
		PORTC->PCR[10] |=(10<<16); // Port C10 IRQC : interrupt on Falling-edge
		PORTC->PCR[11] |=(10<<16); // Port C11 IRQC : interrupt on Falling-edge
		PORTC->PCR[12] |=(10<<16); // Port C12 IRQC : interrupt on Falling-edge
		PORTC->PCR[13] |=(10<<16); // Port C13 IRQC : interrupt on Falling-edge
		PORTC->PCR[9] |=(10<<16); // Port C14 IRQC : interrupt on Falling-edge
		
	/* Segment output PORT E1~11*/
	
	  PTE->PDDR |= (1<<1);   /* Port D1: Data Direction= input (default) */
	  PTE->PDDR |= (1<<2);   /* Port D2: Data Direction= input (default) */
	  PTE->PDDR |= (1<<3);   /* Port D3: Data Direction= input (default) */
	  PTE->PDDR |= (1<<4);   /* Port D4: Data Direction= input (default) */
	  PTE->PDDR |= (1<<5);   /* Port D5: Data Direction= input (default) */
	  PTE->PDDR |= (1<<6);   /* Port D6: Data Direction= input (default) */
	  PTE->PDDR |= (1<<7);   /* Port D7: Data Direction= input (default) */

	  PORTE->PCR[1] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
	  PORTE->PCR[2] = PORT_PCR_MUX(1); /* Port D2: MUX = GPIO */
	  PORTE->PCR[3] = PORT_PCR_MUX(1); /* Port D3: MUX = GPIO */
	  PORTE->PCR[4] = PORT_PCR_MUX(1); /* Port D4: MUX = GPIO */
	  PORTE->PCR[5] = PORT_PCR_MUX(1); /* Port D5: MUX = GPIO */
	  PORTE->PCR[6] = PORT_PCR_MUX(1); /* Port D6: MUX = GPIO */
	  PORTE->PCR[7] = PORT_PCR_MUX(1); /* Port D7: MUX = GPIO */

	  PTE->PDDR |= 1<<8|1<<9|1<<10|1<<11;
	  PORTE->PCR[8] = PORT_PCR_MUX(1); /* Port D8: MUX = GPIO */
	  PORTE->PCR[9] = PORT_PCR_MUX(1); /* Port D9: MUX = GPIO */
	  PORTE->PCR[10] = PORT_PCR_MUX(1); /* Port D10: MUX = GPIO */
	  PORTE->PCR[11] = PORT_PCR_MUX(1); /* Port D11: MUX = GPIO */
		
		/* PORT D10 = Humid&Temp sensor, D11 = BUZZER, PORT D1,15,16 = FTM0CH0,1,3*/

		PTD->PDDR |= (1<<10);
		PORTD->PCR[12] = PORT_PCR_MUX(1);
		PORTD->PCR[10] = PORT_PCR_MUX(1);
		PORTD->PCR[16] |=PORT_PCR_MUX(2);   
		PORTB->PCR[5] |=PORT_PCR_MUX(2);   
		PORTD->PCR[1] |=PORT_PCR_MUX(2);	/* Port D16: MUX = ALT2, FTM0CH1 */
}

void WDOG_disable (void) //WatchDog Disable
	{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}


void LPIT0_init (uint32_t delay){
   uint32_t timeout;
   /*!
    * LPIT Clocking:
    * ==============================
    */
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs       */

  /*!
   * LPIT Initialization:
   */
  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  /* DBG_EN-0: Timer chans stop in Debug mode */
                                        /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                                        /* SW_RST=0: SW reset does not reset timer chans, regs */
                                        /* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */
	LPIT0->MIER = 0x07;

	LPIT0->TMR[2].TVAL = 3*40000000;      /* Chan 2 Timeout period: 40M = 1s */
  LPIT0->TMR[2].TCTRL = 0x00000001;
	
	LPIT0->TMR[1].TVAL = 6*40000000;      /* Chan 1 Timeout period: 60*40M = 60s */
  LPIT0->TMR[1].TCTRL = 0x00000001;
	
  timeout=delay* 40;            //ms setting*40000 us setting *40
  LPIT0->TMR[0].TVAL = timeout;      /* Chan 0 Timeout period: 40M clocks */
  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
                                     /* T_EN=1: Timer channel is enabled */
                              /* CHAIN=0: channel chaining is disabled */
                              /* MODE=0: 32 periodic counter mode */
                              /* TSOT=0: Timer decrements immediately based on restart */
                              /* TSOI=0: Timer does not stop after timeout */
                              /* TROT=0 Timer will not reload on trigger */
                              /* TRG_SRC=0: External trigger soruce */
                              /* TRG_SEL=0: Timer chan 0 trigger source is selected*/

}

void delay_us (volatile int us){
   LPIT0_init(us);           /* Initialize PIT0 for 1 second timeout  */
   while (0 == (LPIT0->MSR &  0x01/*LPIT_MSR_TIF0_MASK*/)) {} /* Wait for LPIT0 CH0 Flag */
               lpit0_ch0_flag_counter++;         /* Increment LPIT0 timeout counter */
               LPIT0->MSR |= 0x00;//............LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
}


void NVIC_init_IRQs(void){
	S32_NVIC->ICPR[1] |= 1<<(61%32); // Clear any pending IRQ61(PORT C)
	S32_NVIC->ISER[1] |= 1<<(61%32); // Enable IRQ61
	S32_NVIC->IP[61] =0x0B; //Priority 11 of 15
	/*LPIT ch1 overflow set*/
	S32_NVIC->ICPR[1] = 1 << (49 % 32);
	S32_NVIC->ISER[1] = 1 << (49 % 32);
	S32_NVIC->IP[49] = 0x09;
	/*LPIT ch2 overflow set*/
	S32_NVIC->ICPR[1] |= 1<< (50 % 32);
	S32_NVIC->ISER[1] |= 1<< (50 % 32);
	S32_NVIC->IP[50] = 0x0A;
	
}
	
void getHumidity(void) { //Humid&Temp Sensor
	
	long highDuty[40];								/*40bit Data	*/
	uint8_t recData[5];								/*8bit Data each (0,2 is each humid and temp)	*/
	int counter=0;
	
	for(int i=0; i<5; i++){
		recData[i]=0;
	}
	
	PTD->PCOR |= (1<<10);							/*PORT D10 = LOW	*/
	delay_us(18000);									/*Start Signal over 18ms	*/
	PTD->PSOR |= (1<<10);							/*PORT D10 = HIGH	*/
	delay_us(40);											/*Wait 40us	(pull up time)	*/
	
	PTD->PDDR &= ~(1<<10);						/*PORT D10 = input	*/
	while(((PTD->PDIR)&(1<<10))==0){}		/*Wait for LOW	*/
		for(int i=0;i<40;i++){					/*40bit write start	*/
			while(PTD->PDIR == (1<<10)) {	/*PORT D10 = HIGH */
				counter++; /*counter++ = measure HIGH time	*/
				delay_us(1);
			}
				highDuty[i] = counter;			/*highDuty[i] = HIGH time	*/
				counter = 0;								/* clear flag	*/
		}
		
		for(int i=0;i<5;i++){						/*divide 5parts 8bit each */
			for(int j=0;j<8;j++){
				int temp = (8*i) + j;
				if(highDuty[temp]>50){
					recData[i] |= 1<<7-j;		/*Binary Humid, Demical Humid, Binary Temp, Demical Temp, CheckSum value	*/
				}
			}
		}
		humidity = recData[0];							/*Need Binary Humid & Temp	*/
		temperature = recData[2];
		
	}


void PORTC_IRQHandler(void){
	
	PORTC->PCR[10] &= ~(0x01000000); // Port Control Register ISF bit '0' set
	PORTC->PCR[11] &= ~(0x01000000); // Port Control Register ISF bit '0' set
	PORTC->PCR[12] &= ~(0x01000000); // Port Control Register ISF bit '0' set
	PORTC->PCR[13] &= ~(0x01000000); // Port Control Register ISF bit '0' set
	PORTC->PCR[9] &= ~(0x01000000); // Port Control Register ISF bit '0' set
	
	if((PORTC->ISFR & (1<<10)) != 0){ //SW1 ON = Humid&Temp display by Bluetooth
		
		getHumidity();
		
		/* Translate int value to ASCII code*/
		
		char humi1 = (char)(humidity%10+48);
		char humi2 = (char)((humidity/10)%10+48);
		char t1 = (char)(temperature%10+48);
		char t2 = (char)((temperature/10)%10+48);
		
		LPUART1_transmit_string("Humid is");
		LPUART1_transmit_char(humi2);
		LPUART1_transmit_char(humi1);
		LPUART1_transmit_char('%');
		LPUART1_transmit_string("Temp is");
		LPUART1_transmit_char(t2);
		LPUART1_transmit_char(t1);
		LPUART1_transmit_string("C\n\r");
	}
	
	else if((PORTC->ISFR & (1<<11)) != 0){ //SW2 ON = display current Water by Bluetooth
		convertAdcChan(15);
		while(adc_complete()==0){}            /* Wait for conversion complete flag 	*/
		adcResultInMv3 = read_adc_chx();       /* Get channel's conversion results in mv */
		if(adcResultInMv3>1800) {
			LPUART1_transmit_string("Enough Water\n");
		}
		else if(adcResultInMv3>1600)	{
			LPUART1_transmit_string("Not Much Water\n");
		}
		else {
			LPUART1_transmit_string("Almost No Water\n");
		}	
	}
		
	else if((PORTC->ISFR & (1<<12)) != 0){	//SW3~5 ON = change MODE
		External_PIN = 1;
	}
	else if((PORTC->ISFR & (1<<13)) != 0){
		External_PIN = 2;
	}
	else if((PORTC->ISFR & (1<<9)) != 0){
		External_PIN = 3;
	}
	
	switch(External_PIN){
		case 1:
			BestHM = LettuceHM;
			BestTemp = LettuceTemp;
			BestLight = 1;
			LPUART1_transmit_string("Current Mode: Lettuce\n");
			External_PIN=0;
			break;
		case 2:
			BestHM = TomatoHM;
			BestTemp = TomatoTemp;
			BestLight = 1;
			LPUART1_transmit_string("Current Mode: Tomato\n");
			External_PIN=0;
			break;
		case 3:
			BestHM = BeanHM;
			BestTemp = BeanTemp;
			BestLight = 0;
			LPUART1_transmit_string("Current Mode: Bean\n");
			External_PIN=0;
			break;
		default:
			break;
	}
	
	PORTC->PCR[10] |= 0x01000000; // Port Control Register ISF bit '1' set
	PORTC->PCR[11] |= 0x01000000; // Port Control Register ISF bit '1' set
	PORTC->PCR[12] |= 0x01000000; // Port Control Register ISF bit '1' set
	PORTC->PCR[13] |= 0x01000000; // Port Control Register ISF bit '1' set
	PORTC->PCR[9] |= 0x01000000; // Port Control Register ISF bit '1' set
	}

void Seg_out(int number){

	 Dtime = 1000;

	num3=(number/1000)%10;
	num2=(number/100)%10;
	num1=(number/10)%10;
	num0= number%10;


	// 1000??? ??
	PTE->PSOR = FND_SEL[j];
	PTE->PCOR =0x7f;
	PTE->PSOR = FND_DATA[num3];
   delay_us(Dtime);
	PTE->PCOR = 0xfff;
	j++;

	// 100??? ??
	PTE->PSOR = FND_SEL[j];
	PTE->PCOR =0x7f;
	PTE->PSOR = FND_DATA[num2];
   delay_us(Dtime);
	    PTE->PCOR = 0xfff;
	j++;

	// 10??? ??
	PTE->PSOR = FND_SEL[j];
	PTE->PCOR =0x7f;
	PTE->PSOR = FND_DATA[num1];
   delay_us(Dtime);
   PTE->PCOR = 0xfff;
	j++;

	// 1??? ??
	PTE->PSOR = FND_SEL[j];
	PTE->PCOR =0x7f;
	PTE->PSOR = FND_DATA[num0];
   delay_us(Dtime);
	    PTE->PCOR = 0xfff;
	j=0;

}



void FTM_init (void){

	//FTM0 clocking
	PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;		//Ensure clk diabled for config
	PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(0b010)		//Clocksrc=1, 8MHz SIRCDIV1_CLK
								| PCC_PCCn_CGC_MASK;		//Enable clock for FTM regs

//FTM0 Initialization
	FTM0->SC = FTM_SC_PWMEN5_MASK| FTM_SC_PWMEN1_MASK|FTM_SC_PWMEN3_MASK					//Enable PWM channel 1output
				|FTM_SC_PS(0.5);								//TOIE(timer overflow Interrupt Ena) = 0 (deafault)
															//CPWMS(Center aligned PWM Select) =0 (default, up count)
															/* CLKS (Clock source) = 0 (default, no clock; FTM disabled) 	*/
															/* PS (Prescaler factor) = 1. Prescaler = 2 					*/

	FTM0->MOD = 8000-1;									//FTM0 counter final value (used for PWM mode)
															// FTM0 Period = MOD-CNTIN+0x0001~=16000 ctr clks=8ms
															//8Mhz /2 =4MHz
	FTM0->CNTIN = FTM_CNTIN_INIT(0);


	FTM0->CONTROLS[5].CnSC |=FTM_CnSC_MSB_MASK;
	FTM0->CONTROLS[5].CnSC |=FTM_CnSC_ELSA_MASK;		
	FTM0->CONTROLS[1].CnSC |=FTM_CnSC_MSB_MASK;
	FTM0->CONTROLS[1].CnSC |=FTM_CnSC_ELSA_MASK;			/* FTM0 ch1: edge-aligned PWM, low true pulses 		*/
	FTM0->CONTROLS[3].CnSC |=FTM_CnSC_MSB_MASK;
	FTM0->CONTROLS[3].CnSC |=FTM_CnSC_ELSA_MASK;	/* FTM0 ch1: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/
															/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/							
}

void FTM0_CH5_PWM (int i){//uint32_t i){ //FTM for Water Pump
	FTM0->CONTROLS[5].CnV=FTM_CnV_VAL(i);
	FTM0->SC|=FTM_SC_CLKS(3);
}

void FTM0_CH1_PWM (int i){//uint32_t i){ //FTM for LED

	FTM0->CONTROLS[1].CnV = i;//8000~0 duty; ex(7200=> Duty 0.1 / 800=>Duty 0.9)
	//start FTM0 counter with clk source = external clock (SOSCDIV1_CLK)
	FTM0->SC|=FTM_SC_CLKS(3);
}

void FTM0_CH3_PWM (int i){//uint32_t i){ //FTM for Servo

	FTM0->CONTROLS[3].CnV=FTM_CnV_VAL(i);
	FTM0->SC|=FTM_SC_CLKS(3);
	
}

void LPIT0_Ch2_IRQHandler (void){	  /* delay counter */
	lpit0_ch2_flag_counter++;         /* Increment LPIT1 timeout counter */
	
	if(flag == 1){
		FTM0_CH5_PWM(0);
		flag=0;
	}
				
	lpit0_ch2_flag_counter=0;
	LPIT0->MSR |= LPIT_MSR_TIF2_MASK;  /* Clear LPIT0 timer flag 0 */
}

void LPIT0_Ch1_IRQHandler (void){		/* delay = 60s counter */
	lpit0_ch1_flag_counter++;         /* Increment LPIT0 timeout counter */
	convertAdcChan(14); /*Soil Humid sensor*/
	while(adc_complete()==0){} //Wait for conversion
	adcResultInMv2 = read_adc_chx();
	if(adcResultInMv2>1000) {
		FTM0_CH5_PWM(5000);
		flag = 1;
		
	}
	lpit0_ch1_flag_counter=0;
	LPIT0->MSR |= LPIT_MSR_TIF1_MASK;  /* Clear LPIT0 timer flag 0 */
}

int main(void){
  uint32_t adcResultInMv=0;	/*< ADC0 Result in miliVolts */

	int D =0;
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
	FTM_init();
	NVIC_init_IRQs();
	LPUART1_init(); /* Initialize LPUART @ 9600*/
	
	/*!
	 * Infinite for:
	 * ========================
	 */
	  for(;;)
	  {
		Seg_out(temperature);
			
		convertAdcChan(13);                   /* Convert Channel AD13 to pot on EVB 	*/
		while(adc_complete()==0){}            /* Wait for conversion complete flag 	*/
		adcResultInMv = read_adc_chx();       /* Get channel's conversion results in mv */
		D=adcResultInMv*1.6;  /* 5000*1.6=8000*/
	
		if(BestLight==1) {
			FTM0_CH1_PWM(8000-D);
			FTM0_CH3_PWM(0);
		}
		else if(BestLight==0) {
			FTM0_CH1_PWM(0);
			FTM0_CH3_PWM(adcResultInMv);
		}		 
	}
}
