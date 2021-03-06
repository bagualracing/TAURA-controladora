/*
* ControladorBagualProgramV1.2.2.c
*
* Equipe Bagual Racing, Projeto controladora motor Brushless Trifásico
*
* Updated: 14/08/2021 
* Author : Vinícius Monaretto e Walter Távora
*/
#define F_CPU 16000000UL				// freq do ATmega328p
#include <avr/io.h>         //biblioteca para acesso aos registradores do uC
#include <avr/interrupt.h>	//biblioteca para interrupções
#include <util/delay.h>

// variaveis globais sempre começam com s_*
uint8_t s_lastPortDstate = 0;
uint8_t s_phaseTurnedOn = 0;
uint8_t s_phaseTurnedOff = 0;
volatile char s_ReadFlag = 0;
volatile float s_AnalogValue = 0;	//valor do sinal analogico
char k = 0;



#define TIMER_PERIOD 0x3E8				// periodo de 16kHz em hexadecimal
#define VIN 5

/*
Programa feito esperando a seguinte montagem do sistema:

- Sensores de efeito Hall nas portas PC0 (fase 0), PC1 (fase 1), PC2 (fase 2);

- Ponte de Fets ativada como:
* High fase 0: PD2
* Low fase 0:  PD3
* High fase 1: PD4
* Low fase 1:  PD5
* High fase 2: PD6
* Low fase 2:  PD7
*/

void debug(char on){
	if (on){
		PORTB |= (1 << PINB5);
	}
	else {
		PORTB &= ~(1 << PINB5);
	}
} 


ISR(TIMER1_COMPB_vect){
	
	
	PORTD &= ~((1 << PIND6)) & ~((1 << PIND4)) & ~((1 << PIND2));
}

ISR(TIMER1_COMPA_vect){
	
	if(s_AnalogValue != 0)
	{
		//PORTD |= s_phaseTurnedOff;                                  //Seta .os valores LOW da ponte H
		switch(s_phaseTurnedOff)
		{
			case 1:
				PORTD &= ~((1 << PIND5)) & ~((1 << PIND7));
				PORTD |= (1 << PIND3);
				break;
			case 2:
				PORTD &= ~((1 << PIND3)) & ~((1 << PIND7));
				PORTD |= (1 << PIND5);
				break;
			case 4:
				PORTD &= ~((1 << PIND3)) & ~((1 << PIND5));
				PORTD |= (1 << PIND7);
				break;
			default:
			//Seta led vermelho
			break;
		}
		

		switch (s_phaseTurnedOn)                                     //Seta os valores HIGH na ponte H
		{
			case 1:
				PORTD &= ~((1 << PIND4)) & ~((1 << PIND6));
				PORTD |= (1 << PIND2);
				break;
			case 2:
				PORTD &= ~((1 << PIND2)) & ~((1 << PIND6));
				PORTD |= (1 << PIND4);
				break;
			case 4:
				PORTD &= ~((1 << PIND4)) & ~((1 << PIND2));
				PORTD |= (1 << PIND6);
				break;
			default:
				PORTD &= ~((1 << PIND6)) & ~((1 << PIND4))& ~((1 << PIND2));
				//Seta led vermelho
				break;
			
		}
		
	}
	else
	{
		PORTD &= ~((1 << PIND6)) & ~((1 << PIND4))& ~((1 << PIND2));
		PORTD &= ~((1 << PIND7)) & ~((1 << PIND5))& ~((1 << PIND3));
			
	}
}

ISR (PCINT1_vect)
{//interrupcao de porta C
	char i;
	//se o sensor de freio estiver ligado, desliga todas as fases do sistema
	if(PINC & (1<<3) || PINC & (1<<4))
	{
		s_phaseTurnedOff = 0;
		s_phaseTurnedOn = 0;
	}
	else
	{
		for(i = 0; i < 3; i++)
		{
			if((PINC & (1 << i)) != (s_lastPortDstate & (1 << i)))	//compara estado atual PINCn com estado passado
			{
				if((PINC & (1 << i)) > 0)
				{
					int PhaseToSetFloat = i - 1;
					if(PhaseToSetFloat<0)
					{
						PhaseToSetFloat = 2; // A B C
					}                        // 0 0 1
					s_phaseTurnedOff &= ~(1 << (PhaseToSetFloat));		//indica que a fase referente ao sensor hall deve ir para Float
					s_phaseTurnedOff |= (1 << (i));				//indica que a fase referente ao sensor hall deve ir para High
				}
				else
				{
					int PhaseToSetFloat = i - 1;
					if(PhaseToSetFloat<0)
					{
						PhaseToSetFloat = 2;
					}
					s_phaseTurnedOn  &= ~(1 << (PhaseToSetFloat));		//indica que a fase referente ao sensor hall deve ir para Float
					s_phaseTurnedOn  |= (1 << (i));			//indica que a fase referente ao sensor hall deve ir para High
					
				}
			}
		}
		
	}
	s_lastPortDstate = PINC;
}

void initADC()
{
	
	ADMUX |= (1<<REFS0);                                               //5 volts de referência, AVcc com capacitor externo no pino AREF
	
	ADMUX |= (1 << MUX0) | (1 << MUX2);                                // Seta ADC5 como conversor ADC utilizado
	
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);              //preescaler de 128,liga o ADC
	
	//ADCSRB = 0x00;			                           //modo de conversão contínua
	
	DIDR0 |= (1 << ADC5D);                                             //desabilita porta digital ADC5, ou PINC5, para possibilitar o ADC a trabalhar
	
	ADCSRA |= (1<<ADSC);                                               // Inicia Conversão
}

void initPWMTimer()
{
	//DDRB  = 0x00;								//pinos OC1B e OC1A (PB2 e PB1) como saída
	//DDRB  |=(1<<PORTB1)|(1<<PORTB2);			//pinos OC1B e OC1A (PB2 e PB1) como saída
	//PORTB = 0b11111001;						//zera saídas e habilita pull-ups nos pinos não utilizados
	// fclk = 16MHz
	// (1/16MHz) * (2^16 -1) = 4,096 ms
	//timer 1 periodo total, sem preescaler: 4,096 ms
	//total de valores do contador:65535
	//valor de tensão mínimo desejado: 3,8 V
	//valor colocado no registrador de interrupção mínimo: 65535*3,8/5 = 49807 = 0xC28F
	//valor de tensão máximo desejado: 4,6 V
	//valor máximo colocado: 65535*4,6/5 = 60292 = 0xEB84
	//uma interrupção ocorre em 65535, com o intuito de colocar o valor positivo na saída, para depois este ser zerado pela outra
	
	TCCR1B = (1<<CS10)|(1<<WGM12);		//Sem prescaler , modo CTC
	TIMSK1 = (1<<OCIE1B)|(1<<OCIE1A);	//habilita interrupcao nos comparadores A e B, dando clear no A
	TCNT1 = 0x00;
	OCR1A = TIMER_PERIOD;
	OCR1B = 0;
	
}

void InitAtmega()
{
	DDRC = 0;					// seta a porta PCx para input (HALLS)
	PORTC = 0xFF;				// liga todos resistores de pull-up da porta PC

	DDRD |= 0xFF;				// seta porta D como output (PWMs)
	PORTD = 0;					// inicia a Porta D toda zerada
	
	DDRB |= 1 << 5 ;				// seta porta B como output (PWMs)
	PORTB = 0;					// inicia a Porta B toda zerada

	PCICR |= 1 << PCIE1;		// Habilita interrupção da porta C
	PCMSK1 |=( (1 << PCINT8) | (1 << PCINT9) |(1 << PCINT10) |(1 << PCINT11) |(1 << PCINT12));              // Mask indicando que apenas PC0, PC1, PC2, PC3 e PC4  causarão interrupção

	initPWMTimer();				//inicia PWM
	initADC();					//inicia conversor analogico digital
	sei();						//habilita interrupcoes
}

int main(void)
{
	InitAtmega();				//Setup do ATmega
	
	debug(0);
	
	while (1)
	{	
		while ( (ADCSRA & (1 << ADSC)) );        // Espera ADC terminar a conversão
		float valorAdc = (float)(ADC/165) - 4.7;
		if(valorAdc > 1)
		{
			valorAdc = 1;
		}
		else if(valorAdc < 0)
		{
			valorAdc = 0;
		}
		s_AnalogValue = valorAdc;
		OCR1B = (int) (TIMER_PERIOD* valorAdc);   // calcula novo valor de PWM
		if(OCR1B>0){
			debug(1);
		}
		_delay_ms(200);                        // Espera 1/5 de segundo, não muito preciso, mas não faz real diferença
		ADCSRA |= (1<<ADSC);
	}
}

//TODO
/*
- Testar e conferir fases e código como um todo

- Implementar sensores de corrente e freio

- Implementar condição default para desligar  (procedimento genérico para desligar o motor)

- Implementar leds de aviso. (Erro é fase ligou errado ou Halls todos desligados.)
(Warning é coisa desconectada antes de iniciar o acionamento(volante e halls))

*/

