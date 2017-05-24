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
//largest memoryaddress in the external eeprom
#define MEMORY_RANGE 0x0001FFFF
#define channels 4 // käy läpi vain flexiforcet. jos nostetaan 6 niin lukee myös kiihtyvyysanturin
//define eeprom instructions
#define WREN 6
#define WRITE 2
#define READ 3
#define BAUDRATE 9600
//no operation used in SPI
#define NOP asm("nop");
/*
char high;
char low;
*/
//Interrupt flag
static volatile uint8_t newIntFlag = 0;
static volatile uint8_t full_flag = 0;
//timer interrupt variables
uint16_t sensor_result;
static volatile uint32_t memory_address=0x00000000;
int channel;
char high;
char low;


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
ADCSRA |=(1<<ADEN);// | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0); //enable adc ja prescaler asetettu 128. Taajuuden pitää olla välillä 50k-200k. 128 arvo on 156k 20MHz MCU taajuudella


//SPI
SPCR0 |= (1<<SPE0)|(1<<MSTR0);// | (0<<SPR10) | (0<<SPR00);// SPI enable ja set master. 
SPSR0 |= (1<<SPI2X0);
PORTB |= (1 << PINB4); //set CS high as default


//USART
UCSR0B |= (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);// | (0<<UMSEL00) | (0<<UMSEL01); // Use 8-bit character sizes

UBRR0L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
UBRR0H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
//Oscilator calibration for 8MHz
OSCCAL = 0xC1;

//Enable button interrupts
PCMSK3 = 0b00000100;
PCICR = 0b00001000;

//timer
TCCR1B |= (1<<CS12) |(1<<CS10); // prescaler. 1024 should be the right value but it runs too slow
TCCR1A |=  (1<<WGM12); // CTC	on
TCNT1 = 0; //init timer to 0
TIMSK1 |= (1<< OCIE1A); //enable compare with OCR1A
OCR1A = 800; //Required steps until 100 ms is reached using 1024 prescaler (0.001/(1/(8000000/1024))-1 ) (required delay/clock time period )-1

//sets memory address to 0 on startup
memory_address=0x00000000;
}
//Returns a uint_16 value containing the sensor result
uint16_t read_sensor(int channel){
		uint16_t result;
        ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
        ADMUX |= channel;

        // Start the conversion.
        ADCSRA |= (1<<ADSC);

        // Wait for the result, then read it.
        while(ADCSRA & (1<<ADSC));
        result =ADC;
        	
	
		return result;
}


//Sends a dummy byte to eeprom
void sync_eeprom(){
	/*synchronization?????*/
	SPDR0 = 0xFF;
	while(!(SPSR0 & (1<<SPIF0))){
			;
		}
		
	
}
//sends inval to eeprom.
void send_to_eeprom(char inval, uint32_t memory_address){
	int i;
	 /*pause*/
	 for(i=0;i<1000;i++){
		 	NOP
	 	}
	PORTB &= ~(1 << PINB4); // Pin 4 goes low. Chip select
	
	//send WREn
	SPDR0=WREN;
	/* Wait for transmission complete */
	 while(!(SPSR0 & (1<<SPIF0))){;}	
	 // Pin  goes high Chip select		 
	 PORTB |= (1 << PINB4); 
	 /*pause*/
	 for(i=0;i<1000;i++){
		 	NOP
	 	}
		 
	
	// Pin 4 goes low. Chip select	
	PORTB &= ~(1 << PINB4);


	SPDR0 = WRITE;								/* send WRITE opCode */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}
NOP;
NOP;
NOP;
NOP;
	/* send address */
	SPDR0 = (memory_address & 0x00ff0000);							/* send upper address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
NOP;
NOP;
NOP;
NOP;
	SPDR0 = (memory_address & 0x0000ff00);		 					/* send lower address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
NOP;
NOP;
NOP;
NOP;
	SPDR0 = (memory_address & 0x000000ff);		 					/* send lower address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}
NOP;
NOP;
NOP;
NOP;	
	

	SPDR0 = inval;								/* send data */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}

	 // Pin  goes high Chip select
	 PORTB |= (1 << PINB4);

}

char read_eeprom(uint32_t memory_address){
	int i;
	char outval;
	
	for(i=0;i<100;i++){
		NOP
	}
		
	/* --- pull CS low --- */
	PORTB &= ~(1 << PINB4);

	SPDR0 = READ;								/* send READ opCode */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
	/*
	for(i=0;i<10;i++){
		NOP
	}
	*/
	/* send address */
	SPDR0 = (memory_address & 0x00ff0000);							/* send upper address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
		
	SPDR0 = (memory_address & 0x0000ff00);		 					/* send middle address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
			;
		}

	
	SPDR0 = (memory_address & 0x000000ff)		; 					/* send lower address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}

	// --- slight pause --- 
	for(i=0;i<100;i++){
		NOP
	}
sync_eeprom();	

	/* get returned data = read SPDR */
	outval = SPDR0;
	
	
	/* --- pull CS high ---- */
	 PORTB |= (1 << PINB4);
		
	return outval; 
	
	
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

//test function that blinks all the leds once
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
		
		
		UDR0 = data; // Send data	

}

char bluetooth_receive(){
	char ReceivedByte;
	while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
		ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"
	return ReceivedByte;
}

void send_all_data(uint32_t memory_address){
				uint32_t j;
				char ReceivedByte;
				blue_led_on();
				bluetooth_transmit(0x53);//S
				bluetooth_transmit(0x54);//T
		//		bluetooth_transmit(0x41);//A
	//			bluetooth_transmit(0x52);//R
//				bluetooth_transmit(0x54);//T
				
				for(j=0;j<memory_address;j=j+1){
					ReceivedByte=read_eeprom(j);
					bluetooth_transmit(ReceivedByte);
				}
				
				bluetooth_transmit(0x45);//E
				bluetooth_transmit(0x4e);//N
	//			bluetooth_transmit(0x44);//D
				led_off();
				newIntFlag = 0;
	
}

/*Interrupts*/
//Sync button interrupt
ISR(PCINT3_vect)
{
	
	newIntFlag = 1;
	
}

//timer interrupt

ISR (TIMER1_COMPA_vect)
{
	
	//loop through all the channels and write the sensor readings to the eeprom
	for(channel=0;channel<channels;channel++){
		sensor_result=read_sensor(channel);
		//parses the 16 bit sensor result to two 8 bit results for transfer purposes
		low = sensor_result & 0xFF;
		high = sensor_result >> 8;
		if((memory_address + 2) < 0x0001FFFF){
			send_to_eeprom(high,memory_address);
			memory_address=memory_address+1;
			send_to_eeprom(low,memory_address);
			memory_address=memory_address+1;
		}
	}
	//shouldn't be necessary
	TCNT1 = 0;
	
	
}


/*/test
ISR (TIMER1_COMPA_vect)
{
	
	
	
		sensor_result=read_sensor(channels);
		//parses the 16 bit sensor result to two 8 bit results for transfer purposes
		low = sensor_result & 0xFF;
		high = sensor_result >> 8;
		
		send_to_eeprom(low,memory_address);
		memory_address=memory_address+1;
		send_to_eeprom(high,memory_address);
		memory_address=memory_address+1;
		
	
	
}
*/

int main(void)
{
init();
blink();
sei();

    while (1) 
    {
		green_led_on();

	//when button is pressed sends data from eeprom via bluetooth 
	if (newIntFlag)
		{
			TIMSK1 &= ~(1<< OCIE1A);
//		cli();
			send_all_data(memory_address);
		//sei();
			TIMSK1 |= (1<< OCIE1A);
		}
	//resets the memory address when eeproms last memory address is reached		
	if (memory_address >=MEMORY_RANGE-1)
	{
		
			TIMSK1 &= ~(1<< OCIE1A); //disable timer compare interrupt becouse eeprom is full and we shouldn't write anymore
			full_flag=1;
			red_led_on();
			while(full_flag==1){
				if (newIntFlag)
				{
					send_all_data(memory_address);
					full_flag=0;
					memory_address=0x00000000;
					TIMSK1 |= (1<< OCIE1A);//enable timer compare interrupts
				}
				
			}
		}

		
    }
}
