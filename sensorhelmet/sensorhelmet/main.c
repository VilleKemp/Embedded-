/*
 * Sensor_helmet.c
 *
 * Created: 2.3.2017 9:28:23
 * Author : Group4
 */ 
#define F_CPU 8000000 // Clock Speed
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


#define channels 4 // käy läpi vain flexiforcet. jos nostetaan 6 niin lukee myös kiihtyvyysanturin
//define eeprom instructions
#define WREN 6
//0b00000110
#define WRITE 0b00000010
#define READ 0b00000011
#define BAUDRATE 9600

#define NOP asm("nop");
//uint16_t adc_result[channels]; //pointless. remove later
char high;
char low;

//Interrupt flag
static volatile uint8_t newIntFlag = 0;


void init(){

//Input/ouput asetus 0=input 1=output
DDRA=0x00;//Portti A asetetaan inputiksi. Sisältää sensorit ja kiihtyvyysanturin
DDRB=0b10110011;//2 tyhjää 2 nappia ja eeprom.  11 00 1101
DDRC=0b11010011;// JTAG pinneissä 2-5, muuten tyhjä 11	0010 11
DDRD=0b11110001;//Bluetooth, 2 nappia, ledi. 10 00 111 1

//Internal pull-ups on=1 off=0
//MCUCR =; //Enable pull-ups
PORTD=0b00001100;//nappien pull-up 00110000

//MCUCR asetus
MCUCR = (0<<JTD)|(1<<PUD); //JTAG enable ja pull-up enable. kaksi kertaa koska jostain syystä ei mene muuten
MCUCR = (0<<JTD)|(1<<PUD);
//ADC
//ADMUX biteillä 0-4 valitaan mistä ADC pinnistä luetaan tietoa
ADMUX |=(1<<REFS0) | (1<<ADLAR);// aseta AREF referenssi jänniteeksi ja älä left adjusti
ADCSRA |=(1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable adc ja prescaler asetettu 128. Taajuuden pitää olla välillä 50k-200k. 128 arvo on 156k 20MHz MCU taajuudella


//SPI
SPCR0 |= (1<<SPE0)|(1<<MSTR0);// SPI enable ja set master
PORTB |= (1 << PINB4); //set CS high as default


//USART
UCSR0B |= (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01) | (0<<UMSEL00) | (0<<UMSEL01); // Use 8-bit character sizes

UBRR0L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
UBRR0H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
//Oscilator calibration for 8MHz
OSCCAL = 0xC1;

//Enable button interrupts
PCMSK3 = 0b00000100;
PCICR = 0b00001000;
}

char * read_sensor(int channel){
		static char result[2];
        ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
        ADMUX |= channel;

        // Start the conversion.
        ADCSRA |= (1<<ADSC);

        // Wait for the result, then read it.
        while(ADCSRA & (1<<ADSC));
        //adc_result[channel] =ADC;
        //high=ADCH;
        //low=ADCL;
        result[0]=ADCH;
        result[1]=ADCL;	
	
		return result;
}

char * read_sensors(){
	int channel=0;
	int position;
	position=0;
	        ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
	static	char results[channels*2];//holds values for one loop of read_sensors()
	for(channel=0;channel<channels;channel++)
    {
        // This is the code that selects the channel. AND out the entire mux area,
        // then OR in the desired analog channel.
        ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
        ADMUX |= channel;

        // Start the conversion.
        ADCSRA |= (1<<ADSC);

        // Wait for the result, then read it.
        while(ADCSRA & (1<<ADSC));
        //adc_result[channel] =ADC;
		//high=ADCH;
		//low=ADCL;
		results[position]=ADCH;
		results[position+1]=ADCL;
		position=position+2;
		
}
        ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
return results;
}

void sync_eeprom(){
	/*synchronization?????*/
	SPDR0 = 0xFF;
	while(!(SPSR0 & (1<<SPIF0))){
			;
		}
		
	
}

void send_to_eeprom(char data, char location_H, char location_L){
	int i;

	PORTB &= ~(1 << PINB4); // Pin 4 goes low. Chip select
	
	//send WREn
	SPDR0=WREN;
	/* Wait for transmission complete */
	 while(!(SPSR0 & (1<<SPIF0))){;}	
	 // Pin  goes high Chip select		 
	 PORTB |= (1 << PINB4); 
	 //pause
	 for(i=0;i<1000;i++){
		 	NOP
	 	}
		 
	sync_eeprom();	 
	// Pin 4 goes low. Chip select	
	PORTB &= ~(1 << PINB4);

	SPDR0 = WRITE;								/* send WRITE opCode */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}

	/* send 2-byte address */
	SPDR0 = location_H;							/* send upper address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}

	SPDR0 = location_L;		 					/* send lower address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}

	SPDR0 = data;								/* send data */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}

	 // Pin  goes high Chip select
	 PORTB |= (1 << PINB4);

}

char read_eeprom(char location_H, char location_L){
	int i;
	char data;
	/* --- pull CS low --- */
		PORTB &= ~(1 << PINB4);

	SPDR0 = READ;								/* send WRITE opCode */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}

	/* send 2-byte address */
	SPDR0 = location_H;							/* send upper address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
	
	SPDR0 = location_L		; 					/* send lower address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}

	/* --- slight pause --- */
	for(i=0;i<100;i++){
		NOP
	}
	
	

	/* get returned data = read SPDR */
	data = SPDR0;
	
	/* --- pull CS high ---- */
	 PORTB |= (1 << PINB4);
	
	return data; 
	
	
}

/*Led functions*/
 void green_led_on(){
	 
	PORTD &= ~(1 << PIND5); // Pin 5 goes low
	PORTD &= ~(1 << PIND4); // Pin 4 goes low
	PORTD |= (1 << PIND6); // Pin 6 goes high	 
 }
 void led_off(){
	 
	PORTD &= ~(1 << PIND5); // Pin 5 goes low
	PORTD &= ~(1 << PIND4); // Pin 4 goes low
	PORTD &= ~(1 << PIND6); // Pin 6 goes low	 
 }
 
  void red_led_on(){
	 
	PORTD &= ~(1 << PIND5); // Pin 5 goes low
	PORTD &= ~(1 << PIND6); // Pin 6 goes low
	PORTD |= (1 << PIND4); // Pin 4 goes high	 
 }
 
   void blue_led_on(){
	 
	PORTD &= ~(1 << PIND4); // Pin 4 goes low
	PORTD &= ~(1 << PIND6); // Pin 6 goes low
	PORTD |= (1 << PIND5); // Pin 5 goes high	 
 }


void blink(){
		_delay_ms(500);
	green_led_on();	
		_delay_ms(500);
	red_led_on();
		_delay_ms(500);
	blue_led_on();
	_delay_ms(500);
	led_off();
	
}
/*Bluetooth funtions*/
void bluetooth_transmit(char data){

	while ((UCSR0A & (1 << UDRE0)) == 0) {
		
		}; // Do nothing until UDR is ready for more data to be written to it
		
		
		UDR0 = data; // Echo back the received byte back to the computer	

}

char bluetooth_receive(){
	char ReceivedByte;
	while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
		ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"
	return ReceivedByte;
}

/*Interrupts*/
//Sync button interrupt
ISR(PCINT3_vect)
{
	newIntFlag = 1;
}


int main(void)
{
init();
sei();
char ep;
char ReceivedByte;
uint16_t result;
char *adc_results;
char location_L;
char location_H;
int channel;
    while (1) 
    {
		location_L=0x00000;
		location_H=0x00001;
	ep="A";
	
	send_to_eeprom(ep,location_H,location_L);
	ReceivedByte=read_eeprom(location_H,location_L);
	
	if(ReceivedByte!=0){
	bluetooth_transmit(ReceivedByte);
	blink();
	}
		if (newIntFlag)
		{
			blink();
			newIntFlag = 0;
		}		
/*
//working code for adc
		for(channel=0;channel<channels;channel++){
        ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));//all muxes to 0
        ADMUX |= channel;
	  // Start the conversion.
        ADCSRA |= (1<<ADSC);

        // Wait for the result, then read it.

        
		while(ADCSRA & (1<<ADSC));
       // result =ADC;
		low=ADCL;
		high=ADCH;
		
		bluetooth_transmit(channel);
		bluetooth_transmit(high);
		bluetooth_transmit(low);

		
		
		}
		*/
		//bluetooth_transmit(result);
		
		
    }
}

