/*
 * Simulation Sensor for Kia Rio 2007
 *
 * Created: 2022 
 * Author: Chung
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

/* PWM timer2 _ PB3 in Atmega232P _ IO11 in Arduino Uno */
#define Pin_Out_Throttle_Position OCR2A
/* GPIO _ Pin_Out_Crank_Shaft_1 in Atmega232P _ IO08 in Arduino Uno */
#define Pin_Out_Crank_Shaft_1 PB0
/* GPIO _ Pin_Out_Crank_Shaft_2 in Atmega232P _ IO12 in Arduino Uno */
#define Pin_Out_Crank_Shaft_2 PB4
/* GPIO _ Pin_Out_Cam_Shaft in Atmega232P _ IO09 in Arduino Uno */
#define Pin_Out_Cam_Shaft PB1
/* Tooth in crank shaft of Kia Rio 2007 is 57*/ 
#define tooth_of_crank_shaft 57

void InitADC(void);
uint16_t ReadADC(uint8_t ADCchannel);
void InitPWM(void);
/*using timer 2*/
void WritePWM(int index);
void delay_test(unsigned long _time);
void delayMicro(unsigned long microseconds);


int main(void)
{    
  DDRB |= (1 << Pin_Out_Crank_Shaft_2),
  DDRB |= (1 << Pin_Out_Crank_Shaft_1);
  DDRB |= (1 << Pin_Out_Cam_Shaft);

  uint16_t index_speed = 0;

  uint8_t Count_tooth_of_crank_shaft = 0;
  bool Is_first_circle_of_Cam_Shaft = true;
  uint16_t Throttle_Position;

  InitADC();

  InitPWM();

  while(1)
  {
    //0 - 1023;
    Throttle_Position = ReadADC(0);
    if(Throttle_Position/4 <= 5){
      WritePWM(254);
      //Pin_Out_Throttle_Position = 254;
    }
    else if(Throttle_Position/4 >= 249){
      WritePWM(0);
      //Pin_Out_Throttle_Position = 0;
    }
    else{
      WritePWM(254 - (float)Throttle_Position/4);
      //Pin_Out_Throttle_Position = 254 - (float)Throttle_Position/4;
    }    

    index_speed = 401 - Throttle_Position/3;
    
    if (Is_first_circle_of_Cam_Shaft == true){
      if (Count_tooth_of_crank_shaft == tooth_of_crank_shaft){
      PORTB &= ~(1 << Pin_Out_Crank_Shaft_2),
      PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
      PORTB |= (1 << Pin_Out_Cam_Shaft);
      delay_test(3*index_speed);
      Count_tooth_of_crank_shaft = 0;
      Is_first_circle_of_Cam_Shaft = false;
      }
      else{
        if(Count_tooth_of_crank_shaft >= 15 && Count_tooth_of_crank_shaft < 22){
          PORTB |= (1 << Pin_Out_Crank_Shaft_2) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
          PORTB &= ~(1 << Pin_Out_Cam_Shaft) ;
          delay_test(index_speed);
          
          PORTB |= (1 << Pin_Out_Crank_Shaft_1) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_2);
          PORTB &= ~(1 << Pin_Out_Cam_Shaft);
          delay_test(index_speed);
          
          Count_tooth_of_crank_shaft ++ ;
        }
        else if(Count_tooth_of_crank_shaft >= 43 && Count_tooth_of_crank_shaft <50){
          PORTB |= (1 << Pin_Out_Crank_Shaft_2) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
          PORTB &= ~(1 << Pin_Out_Cam_Shaft) ;
          delay_test(index_speed);
          
          PORTB |= (1 << Pin_Out_Crank_Shaft_1) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_2);
          PORTB &= ~(1 << Pin_Out_Cam_Shaft);
          delay_test(index_speed);
          
          Count_tooth_of_crank_shaft ++ ;
        }
        else{
          PORTB |= (1 << Pin_Out_Crank_Shaft_2) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
          PORTB |= (1 << Pin_Out_Cam_Shaft) ;
          delay_test(index_speed);
          PORTB |= (1 << Pin_Out_Crank_Shaft_1) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_2);
          PORTB |= (1 << Pin_Out_Cam_Shaft) ;
          delay_test(index_speed);
          Count_tooth_of_crank_shaft ++ ;
        }
      }
    }

    // Count_tooth_of_crank_shaft = false -> chu ki 2
    else{
      if (Count_tooth_of_crank_shaft == tooth_of_crank_shaft){
      PORTB &= ~(1 << Pin_Out_Crank_Shaft_2),
      PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
      PORTB &= ~(1 << Pin_Out_Cam_Shaft) ;
      delay_test(3*index_speed);
      Count_tooth_of_crank_shaft = 0;
      Is_first_circle_of_Cam_Shaft = true;
      }
      // Count_tooth_of_crank_shaft != 65
      else{
        if(Count_tooth_of_crank_shaft >= 23 && Count_tooth_of_crank_shaft <28){
          PORTB |= (1 << Pin_Out_Crank_Shaft_2) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
          PORTB |= (1 << Pin_Out_Cam_Shaft);
          delay_test(index_speed);
          
          PORTB |= (1 << Pin_Out_Crank_Shaft_1) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_2);
          PORTB |= (1 << Pin_Out_Cam_Shaft);
          delay_test(index_speed);
          
          Count_tooth_of_crank_shaft ++ ;
        }
        else if(Count_tooth_of_crank_shaft >= 50 && Count_tooth_of_crank_shaft <55){
          PORTB |= (1 << Pin_Out_Crank_Shaft_2) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
          PORTB |= (1 << Pin_Out_Cam_Shaft);
          delay_test(index_speed);
          
          PORTB |= (1 << Pin_Out_Crank_Shaft_1) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_2);
          PORTB |= (1 << Pin_Out_Cam_Shaft);
          delay_test(index_speed);
          
          Count_tooth_of_crank_shaft ++ ;
        }
        else{
          PORTB |= (1 << Pin_Out_Crank_Shaft_2) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_1);
          PORTB &= ~(1 << Pin_Out_Cam_Shaft) ;
          delay_test(index_speed);
          PORTB |= (1 << Pin_Out_Crank_Shaft_1) ,
          PORTB &= ~(1 << Pin_Out_Crank_Shaft_2);
          PORTB &= ~(1 << Pin_Out_Cam_Shaft) ;
          delay_test(index_speed);
          Count_tooth_of_crank_shaft ++ ;
        }
      }
    }
    
  }
    
  return 0;
}

void InitADC(){
   /*
    * ADC Multiplexer Selection Register
    * Bit 7:6 – REFS1:0: Reference Selection Bits _ 0 1 AVCC with external capacitor at AREF pin */
  ADMUX |= (1 << REFS0);
  /*  
   *  ADC Control and Status Register A
   *  Bit 7 - ADEN : ADC Enable
   *  Bits 2:0 – ADPS2:0: ADC Prescaler Select Bits _ 1 1 1 -> 128 */  
  ADCSRA |= (1 << ADEN)|(1<<ADPS2) |(1<<ADPS1) |(1<<ADPS0);
}

uint16_t ReadADC(uint8_t ADCchannel){
  /*
   * Bits 3:0 – MUX3:0: Analog Channel Selection Bit _ 0000 -> ADC0 */
   ADMUX = (ADMUX & 0xF0)|(ADCchannel & 0x0F);
   
   /*
   * Bit 6 – ADSC: ADC Start Conversion _ 
   * When the conversion is complete, it returns to zero */
   ADCSRA |= (1 << ADSC);

   // ADSC will read as one as long as a conversion is in progress so wait until ADC conversion is complete
   while(ADCSRA & (1 <<ADSC));
   return ADC;
}

void InitPWM()
{
    // initialize TCCR2 as: fast pwm mode, non inverting
    TCCR2A |= (1<<COM2A1) | (1<<COM2A0) | (1<<WGM21) | (1<<WGM20);
    TCCR2B |= (1<<CS21); // clkT2S/8 prescale
    // OC2RA pin made as output _ pin 11 in arduino broad.
    DDRB |= (1<<PB3);

}
void WritePWM(int index){
  Pin_Out_Throttle_Position = index;
}

void delayMicro(unsigned long microseconds)
{

    unsigned long i = 0;

    TCCR0A = 0;
    TCCR0B = 1 << CS01; // Prescalar of 8

    while (i < microseconds)
    {
        TCNT0 = 254;
        TIFR0 = 1 << TOV0;

        while (!(TIFR0 & (1 << TOV0)));
        i++;

    }

}



void delay_test(unsigned long _time){
  for(int i = 0; i <= _time ; i+=1){
    _delay_us(1);
  }
}
