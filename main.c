#define F_CPU 1600000UL
#include <avr/io.h>
#include "SSD1306/SSD1306.h"
#include "SSD1306/Font5x8.h"
#include <stdio.h>
#include <avr/interrupt.h>         //comentando oque foi alterado ou criado para o Sprint 10 
#include <avr/eeprom.h>						// FAROL INTELIGENTE E SENSOR DE PORTA ABERTA
#include <util/delay.h>  
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
//#define BOT (1<<3)
//#define GATE (1<<6)

#define ENDERECO_HODOMETRO 0
#define ENDERECO_DIAMETRO_PENEU 4
#define ENDERECO_TEMP_MAX 6

//para essa questão, essas variavéis
//uint16_t acelerador = 0;
//uint16_t bateria = 0, temperatura = 0, Vt = 0, Rt = 0;
#define ENDERECO_HODOMETRO 0
#define ENDERECO_DIAMETRO_PNEU 4

typedef struct stc_veiculo{
	
	uint16_t Velocidade_carro_kmH;
	uint16_t RPM_motor;
	uint16_t Diametro_peneu_cm;
	uint16_t Curso_pedal;
	uint16_t Luminosidade;     //valor do nivel de luz
	uint16_t Distancia_sonar_cm;
	
	uint16_t Distancia_porta;   //valor da porta aberta ou fechada
	
	uint8_t Bateria_percent_v;
	uint8_t Temp_bateria_celsius;
	uint8_t Temp_maxima_celsius;
		
	float Distancia_hodometro_km;
	char Marcha;
	
	
	
} stc_veiculo;


stc_veiculo Veiculo = {.Luminosidade = 0 , .Distancia_porta = 0 ,.Temp_maxima_celsius = 0 ,.Bateria_percent_v=0 , .Temp_bateria_celsius = 0 ,.Distancia_sonar_cm = 0, .Curso_pedal = 0, .Velocidade_carro_kmH = 0, .RPM_motor = 0, .Diametro_peneu_cm = 65, .Distancia_hodometro_km = 10, .Marcha = 'D'};
uint8_t Flag_5ms = 0, Flag_50ms=0, Flag_100ms=0, Flag_500ms=0;
uint32_t Tempo_ms = 0 ; //variavel global para o tempo de subida

void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(stc_veiculo veiculo, uint8_t *flag_disparo);
void load_EEPROM(stc_veiculo *veiculo);
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);

void leitura_sensores_ADC(stc_veiculo *veiculo, uint8_t *flag_disparo);

ISR(USART_RX_vect)
{
	char recebido;
	recebido = UDR0;
	if(recebido=='d')
	USART_Transmit(Veiculo.Temp_maxima_celsius);
	if(recebido=='l')
	Veiculo.Temp_maxima_celsius = 0;
	
}


ISR(TIMER0_COMPA_vect){
	
	Tempo_ms++;
	if((Tempo_ms % 5)==0)
	Flag_5ms = 1;
	if((Tempo_ms % 50)==0)
	Flag_50ms = 1;
	if((Tempo_ms % 100)==0)
	Flag_100ms = 1;
	if((Tempo_ms % 500)==0)
	Flag_500ms = 1;
	
	
}

ISR(INT0_vect){
	
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0 ,distancia_hodometro_km_anterior = 0 ;
	uint16_t delta_t_ms = 0;
	
	if (cont_5voltas == 5)
	{
		
		delta_t_ms = Tempo_ms - tempo_ms_anterior;
		Veiculo.RPM_motor = 300000/(delta_t_ms);
		
		Veiculo.Velocidade_carro_kmH = ((uint32_t)Veiculo.Diametro_peneu_cm*565)/delta_t_ms;
		tempo_ms_anterior = Tempo_ms;
		cont_5voltas = 0;
		Veiculo.Distancia_hodometro_km += ((float)Veiculo.Diametro_peneu_cm*5*3.1415)/100000;
		if ((uint32_t)Veiculo.Distancia_hodometro_km > distancia_hodometro_km_anterior)
		{
			eeprom_write_dword(ENDERECO_HODOMETRO, (uint32_t)Veiculo.Distancia_hodometro_km);
			distancia_hodometro_km_anterior = Veiculo.Distancia_hodometro_km;
		
		}
		
	}
	cont_5voltas++;
	
	//Veiculo.Distancia_hodometro_km += ((float)Veiculo.Diametro_peneu_cm*3.1415)/100000;
	
}

ISR(PCINT2_vect)
{
	static uint32_t tempo_ms_anterior1 = 0 ;
	uint16_t delta_t_ms1 = 0;
		
		if ((PIND&0b00010000)==0)
		{
			if (Veiculo.Diametro_peneu_cm < 200)
			{
				Veiculo.Diametro_peneu_cm++;
				eeprom_write_byte(ENDERECO_DIAMETRO_PENEU, Veiculo.Diametro_peneu_cm);
			}
		}
		
		if ((PIND&0b00100000)==0)
		{
			if (Veiculo.Diametro_peneu_cm > 1)
			{
				Veiculo.Diametro_peneu_cm--;
				eeprom_write_byte(ENDERECO_DIAMETRO_PENEU, Veiculo.Diametro_peneu_cm);
			}
		}
		
	//novo método do LCD
	if((PIND & 0b01000000))
	{
		Veiculo.Marcha = 'D';
	}else{
		Veiculo.Marcha = 'R';
	}
	if((PIND & 0b10000000))
	{
		Veiculo.Marcha = 'P';
	}
	
	
	
	
	
}

	ISR(ADC_vect){
		
		Veiculo.Curso_pedal = ADC;
		if(Veiculo.Distancia_sonar_cm > 300){
			OCR2B = Veiculo.Curso_pedal/4;
		}
		else if (Veiculo.Velocidade_carro_kmH > 20)
		{
			OCR2B = 25;
		}
		
	}
	
	ISR(TIMER1_CAPT_vect){
		
		static uint16_t tempo_borda_subida = 0;
		if (TCCR1B & (1<<ICES1))
			tempo_borda_subida = ICR1;
		else
		Veiculo.Distancia_sonar_cm = ((uint32_t)(ICR1 - tempo_borda_subida)*4)/58;
		TCCR1B ^=(1<<ICES1);
		
		
	}
	
	
	



	int main(void)
	{
		
		DDRB = 0b11111110;         
		//DDRD &= 0b00001011;
		DDRD &= 0b00000011;
		//PORTD = 0b00110000;
		PORTD = 0b00110000;
		PORTB = 0b00000001;         
		DDRC |= 0b01000000;        //determinando entradas e saidas na porta C
		DDRC &= 0b11110111;
		
		//DDRC &= ~BOT;
		//DDRC |= GATE;
		
		
		
		
		
		EICRA = 0b00000010;
		EIMSK = 0b00000001;
		PCICR = 0b00000100;
		//PCMSK2 = 0b00110000;
		PCMSK2= 0b11110000;
		
		TCCR0A = 0b00000010;
		TCCR0B = 0b00000011;
		OCR0A = 249;
		TIMSK0 = 0b00000010;
		
		//TCCR1A = 0;
		TCCR1B = (1<<ICES1)|(1<<CS11)|(1<<CS10);
		TIMSK1 = 1 << ICIE1;
		//hj
		TCCR2A = 0b00100011;
		//--
		TCCR2B = 0b00000100;
		OCR2B = 128;
		
		ADMUX = 0b01000000;
		ADCSRA = 0b11100111;
		//ADCSRA = 0b11100111;
		ADCSRB = 0b00000000;
		DIDR0 = 0b00100000;
		
		//TCCR1B = (1<<ICES1)|(1<<CS12);
		//TIMSK1 = 1<<ICIE1;
		
		USART_Init(MYUBRR);
		load_EEPROM(&Veiculo);
		
		
		
		sei();
		//settando o novo LCD
		GLCD_Setup();
		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
		GLCD_InvertScreen();
		
		
		
		
		
		while (1)
		{
		
			int16_t LEITBOT =0;
			int16_t ULTLEITBOT = 0; 
			
			anima_velocidade(Veiculo.Velocidade_carro_kmH, &Flag_5ms);
			anima_LCD(Veiculo, &Flag_500ms);
			//Sensor_ADC(&Flag_500ms, &temperatura, &bateria);
			//void load_EEPROM(Veiculo);
			//colocando os valores no LCD
			leitura_sensores_ADC(&Veiculo, &Flag_50ms);
			
			if ((PIND&0b00001000)==0){                    //checando se a porta está aberta ou fechada
				Veiculo.Distancia_porta = 1;               //e atribuindo um novo valor a ela
			}else Veiculo.Distancia_porta = 0;
			
			
			
			if (Veiculo.Luminosidade <=500)               //verificando se a luminosidade está dentro da faixa
			{                                              // desejada para ligar os faróis
				PORTC = 0b11111111;
			}else PORTC = 0b00000000;
			
				
			
			
		}
	}

	void anima_velocidade( uint16_t velocidade_carro, uint8_t *flag_disparo){
		
		static int8_t cont_dig = 0;
		
		
		if (*flag_disparo)
		{
			switch (cont_dig)
			{
				case 0:
				PORTB &= 0b00000001;
				PORTB |= 0b11000000;
				PORTB |= ((((velocidade_carro/1)%10)<<1) & 0b00011110);
				break;
				case 1:
				PORTB &= 0b10000001;
				PORTB |= 0b10100000;
				PORTB |= ((((velocidade_carro/10)%10)<<1) & 0b00011110);
				break;
				case 2:
				PORTB &= 0b00000001;
				PORTB |= 0b01100000;
				PORTB |=  ((((velocidade_carro/100)%10)<<1) & 0b00011110);
				cont_dig = -1;
				break;
			}
			cont_dig++;
			*flag_disparo = 0;
			
			
			
		}
		
		
	}



	void load_EEPROM(stc_veiculo *veiculo)
	{
		veiculo->Distancia_hodometro_km = eeprom_read_dword(ENDERECO_HODOMETRO);
		veiculo->Diametro_peneu_cm = eeprom_read_byte(ENDERECO_DIAMETRO_PENEU);
		veiculo->Temp_maxima_celsius = eeprom_read_byte(ENDERECO_TEMP_MAX);
		
	}
	
	
	//Aqui é onde é feito as alterações da bateria e da temperatura
	void leitura_sensores_ADC(stc_veiculo *Veiculo, uint8_t *flag_disparo) {
		
		static uint8_t cont_canal = 0;
		static uint8_t cont_temp = 0;
		
		if (*flag_disparo)
		{
			switch(cont_canal){
				//mudança do canal
				case 0: 
				Veiculo ->Curso_pedal = ADC;
				if (Veiculo->Distancia_sonar_cm>300)
				{
					OCR2B = Veiculo->Curso_pedal/4;
				}else if (Veiculo->Velocidade_carro_kmH > 20)
				{
					OCR2B = 25;
				}
				ADMUX = 0b01000001;
				
				break;
				//canal 1
				case 1:
				Veiculo->Bateria_percent_v = ((uint32_t)ADC*100)/1023;
				ADMUX = 0b01000010; 
				
				break;
				//canal 2
				case 2: 
				Veiculo->Temp_bateria_celsius = (uint32_t)ADC*2597/(1023-ADC) - 259;
				ADMUX = 0b01000000;  // formulazinha viu, vish maria = Vt=(5/1023)*ADC, Rt=((1000*Vt)/(5-Vt)) e T=2,6*Rt-260
				if (cont_temp == 0)
				{
					Veiculo->Temp_maxima_celsius = Veiculo ->Temp_bateria_celsius;
					cont_temp++;
				}else if (Veiculo->Temp_bateria_celsius > Veiculo->Temp_maxima_celsius)
				{
					Veiculo->Temp_maxima_celsius = Veiculo ->Temp_bateria_celsius;
				}
				eeprom_write_byte(ENDERECO_TEMP_MAX,Veiculo->Temp_maxima_celsius);
				
				break;
				case 3:   //criação de um novo canal para os calculos da luminosidade do seu video kkk
				ADMUX = 0b01000011; //mudança para o canal 3
				Veiculo -> Luminosidade = (float)(1023000/ADC-1000);  //ultilizando ponteiro
				break;
				
				
				
				
			}
			if (cont_canal < 3)
			{
				cont_canal++;
			}else{
				cont_canal = 0;
			}
			
			
			*flag_disparo = 0;
		}
	}

	
	void anima_LCD(stc_veiculo Veiculo, uint8_t *flag_disparo)  //depois boto no display oque quero
	{
		if (*flag_disparo)
		{	
			char luminosidade_string[4];  //novo
			char diametro_pneu_cm_string[4];
			char rpm_motor_string[6];
			char distancia_hodometro_km_string[8];
			char distancia_sonar_cm_string[8];
			char bateria_percent_v_string[4];
			char temp_bateria_celsius_string[4];
			char marcha_string[2];
			//diametro peneu era antes
			
			sprintf(luminosidade_string ,"%u",Veiculo.Luminosidade);
			sprintf(diametro_pneu_cm_string ,"%u",Veiculo.Diametro_peneu_cm );
			sprintf(rpm_motor_string ,"%u",Veiculo.RPM_motor );
			sprintf(distancia_hodometro_km_string ,"%u",(uint16_t)Veiculo.Distancia_hodometro_km );
			sprintf(distancia_sonar_cm_string ,"%u",Veiculo.Distancia_sonar_cm );
			sprintf(bateria_percent_v_string ,"%u",Veiculo.Bateria_percent_v );
			sprintf(temp_bateria_celsius_string ,"%u",Veiculo.Temp_bateria_celsius );
			sprintf(marcha_string ,"%u",Veiculo.Marcha );
			
			GLCD_Clear();
			
			GLCD_GotoXY(1,1);
			GLCD_PrintString("LASD CAR");
			GLCD_DrawLine(1,10,50,10, GLCD_Black);
			
			GLCD_GotoXY(75,1);
			GLCD_PrintString(luminosidade_string);
			
			GLCD_GotoXY(100,15);
			GLCD_PrintString(temp_bateria_celsius_string);
			GLCD_PrintString("C");
			
			
			GLCD_GotoXY(100,3);
			GLCD_PrintString(bateria_percent_v_string);
			GLCD_PrintString(" %");
			
			GLCD_DrawRectangle(98,1,126,25,GLCD_Black);
			
			GLCD_GotoXY(1,36);
			GLCD_PrintString("D. Pneu: ");
			GLCD_PrintString(diametro_pneu_cm_string);
			GLCD_PrintString(" cm");
			
			GLCD_GotoXY(110,52);
			GLCD_PrintString(" P");
			GLCD_DrawRectangle(108,48,125,62,GLCD_Black);
			
			GLCD_GotoXY(1,26);
			GLCD_PrintString("Sonar: ");
			GLCD_PrintString(distancia_sonar_cm_string);
			GLCD_PrintString(" cm");
			
			GLCD_GotoXY(1,16);
			
			GLCD_PrintString(rpm_motor_string);
			GLCD_PrintString(" rpm");
			
			GLCD_GotoXY(20,52);
			GLCD_PrintString(distancia_hodometro_km_string);
			GLCD_PrintString(" km");
			
			GLCD_DrawRectangle(10,48,70,62,GLCD_Black);
			
			
			
			if (Veiculo.Distancia_porta == 1)   //escrever no display caso o teste
			{
				GLCD_GotoXY(110,36);
				GLCD_PrintString("A");
				
				
			}else {                          //caso a porta fechada
				GLCD_GotoXY(110,36);
				GLCD_PrintString("F");
			}
			
			GLCD_Render();
			
		}
		
		
	}
	
	void USART_Init(unsigned int ubrr)
	{
		UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão
		UBRR0L = (unsigned char)ubrr;
		UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
		UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 2 de parada
		
		
	}
	
	// ||Função para envio de um frame de 5 a 8bits||
	void USART_Transmit(unsigned char data)
	{
		while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
		UDR0 = data; //Coloca o dado no registrador e o envia
	}

	// ||Função para recepção de um frame de 5 a 8bits||
	unsigned char USART_Receive(void)
	{
		while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
		return UDR0; //Lê o dado recebido e retorna
	}
