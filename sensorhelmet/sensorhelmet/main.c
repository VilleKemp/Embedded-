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
//largest memory address in the external eeprom
#define MEMORY_RANGE 0x0001FFFF
#define channels 4 // Amount of ADC pins the read loop goes through. currently goes over the ADC pins 0-3. 
//define eeprom instructions
#define WREN 6
#define WRITE 2
#define READ 3

#define BAUDRATE 9600
//no operation used in SPI
#define NOP asm("nop");

//Interrupt flag
static volatile uint8_t newIntFlag = 0;

//variables
uint16_t sensor_result;
static volatile uint32_t memory_address;
uint32_t address;
int channel;
char high;
char low;

uint8_t h_add;
uint8_t m_add;
uint8_t l_add;

static volatile uint32_t j ;



void init(){

//Input/ouput asetus 0=input 1=output
DDRA=0x00;//Portti A asetetaan inputiksi. Sis‰lt‰‰ sensorit ja kiihtyvyysanturin
DDRB=0b10110011;//2 tyhj‰‰ 2 nappia ja eeprom.  11 00 1101
DDRC=0b11010011;// JTAG pinneiss‰ 2-5, muuten tyhj‰ 11	0010 11
DDRD=0b11110001;//Bluetooth, 2 nappia, ledi. 10 00 111 1

//Internal pull-ups on=1 off=0
PORTD=0b00001100;//nappien pull-up 00110000

//MCUCR asetus
MCUCR = (0<<JTD)|(1<<PUD); //JTAG enable ja pull-up enable. kaksi kertaa koska jostain syyst‰ ei mene muuten
MCUCR = (0<<JTD)|(1<<PUD);
//ADC
//ADMUX biteill‰ 0-4 valitaan mist‰ ADC pinnist‰ luetaan tietoa
ADMUX |=(1<<REFS0) | (1<<ADLAR);// aseta AREF referenssi j‰nniteeksi ja ‰l‰ left adjusti
ADCSRA |=(1<<ADEN); //enable adc ja prescaler asetettu 128. Taajuuden pit‰‰ olla v‰lill‰ 50k-200k. 128 arvo on 156k 20MHz MCU taajuudella
//SPI
SPCR0 |= (1<<SPE0)|(1<<MSTR0) | (1<<SPR00);// SPI enable ja set master. clock = 1/16
PORTB |= (1 << PINB4); //set CS high as default
//USART
UCSR0B |= (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes
UBRR0L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
UBRR0H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
//Oscilator calibration for 8MHz
OSCCAL = 0xC1;
//Enable button interrupts
PCMSK3 = 0b00000100;
PCICR = 0b00001000;
//timer
TCCR1B |= (1<<CS12) |(1<<CS10); // prescaler. 1024 
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
//Sends a dummy byte to eeprom to clock the data
void sync_eeprom(){
	/*synchronization*/
	SPDR0 = 0x00;
	while(!(SPSR0 & (1<<SPIF0))){
			;
		}
}
//sends inval to eeprom.
void send_to_eeprom(char inval, uint32_t memory_address){
	int i;
	//address is parsed in to 3 8-bit variables
	h_add = (memory_address & 0x00ff0000) >> 16 ;
	m_add = (memory_address & 0x0000ff00) >> 8;
	l_add = memory_address & 0x000000ff;
	//without this nop the eeprom only writes every 4th value to memory. Might be replaceable by asking eeprom for it's state before every write	 
		 for(i=0;i<5000;i++){
			 NOP
		 } 
	PORTB &= ~(1 << PINB4); // Pin 4 goes low. Chip select
/*pause*/
	 for(i=0;i<100;i++){
		 	NOP
	 	}	
	//send WREN
	SPDR0=WREN;
	/* Wait for transmission complete */
	 while(!(SPSR0 & (1<<SPIF0))){;}	
	 // Pin  goes high Chip select		 
	 PORTB |= (1 << PINB4); 
	 /*pause*/
	 for(i=0;i<100;i++){
		 	NOP
	 	}
	// Pin 4 goes low. Chip select	
	PORTB &= ~(1 << PINB4);
	SPDR0 = WRITE;								/* send WRITE opCode */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}
	/* send address */
	SPDR0 = h_add;							/* send upper address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
	SPDR0 = m_add;		 					/* send middle address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
	SPDR0 = l_add;		 					/* send lower address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}
	SPDR0 = inval;								/* send data */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
	;
	}
	 // Pin  goes high Chip select
	 PORTB |= (1 << PINB4);
}
//read value from eeprom
char read_eeprom(uint32_t memory_address){
	int i;
	char outval;
	//parse 32 bit memory_address variable in to three 8-bit variables
	h_add = (memory_address & 0x00ff0000) >> 16 ;
	m_add = (memory_address & 0x0000ff00) >> 8;
	l_add = memory_address & 0x000000ff;
	for(i=0;i<100;i++){
		NOP
	}
	/* --- pull CS low --- */
	PORTB &= ~(1 << PINB4);
	SPDR0 = READ;								/* send READ opCode */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
	/* send address */
	SPDR0 = h_add;							/* send upper address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
		;
	}
	
	SPDR0 = m_add;		 					/* send middle address */
	while(!(SPSR0 & (1<<SPIF0))){					/* Wait for transmission complete */
			;
		}
	SPDR0 = l_add		; 					/* send lower address */
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

//function that blinks all the leds once
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
/*Bluetooth functions*/
void bluetooth_transmit(char data){
	while ((UCSR0A & (1 << UDRE0)) == 0) {
		}; // Do nothing until UDR is ready for more data to be written to it
		UDR0 = data; // Send data	
}

char bluetooth_receive(){
	char ReceivedByte;
	while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
		ReceivedByte = UDR0; // Fetch the received byte value into the variable "ByteReceived"
	return ReceivedByte;
}

void send_all_data(uint32_t memory_address){ 
				uint32_t value;
				value=0x000000;
				char ReceivedByte;
				blue_led_on();
				//Send S and T to tell the client that the data is about be sent
				bluetooth_transmit(0x53);//S
				bluetooth_transmit(0x54);//T				
				for(j=0;j<memory_address;j=j+1){
					ReceivedByte=read_eeprom(value);
					bluetooth_transmit(ReceivedByte);
					value=value+1;
				}
				//Send E nd N to signal the end of transmission
				bluetooth_transmit(0x45);//E
				bluetooth_transmit(0x4e);//N
				led_off();
				//reset the send flag
				newIntFlag = 0;
}
/*Interrupts*/
//Sync button interrupt
ISR(PCINT3_vect)
{
	//To avoid button bouncing interrupt sets the variable to one. Variable is later polled at while loop.
	newIntFlag = 1;	
}
//timer interrupt
//happens every time counter hits 800. So roughly every 100ms
ISR (TIMER1_COMPA_vect)
{
	//loop through all the channels and write the sensor readings to the eeprom
	for(channel=0;channel<channels;channel=channel+1){
		sensor_result=read_sensor(channel);
		//parses the 16 bit sensor result to two 8 bit results for transfer purposes
		low = sensor_result & 0xFF;
		high = sensor_result >> 8;
		send_to_eeprom(high,memory_address);
		memory_address=memory_address+1;
		send_to_eeprom(low,memory_address);
		memory_address=memory_address+1;
		}
	//set counter to zero
	TCNT1 = 0;	
}

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
			//cancels timer compare interrupt during transmission.
			TIMSK1 &= ~(1<< OCIE1A);
			send_all_data(memory_address);
			TIMSK1 |= (1<< OCIE1A);
		}
	//resets the memory address when eeproms last memory address is reached		
	if (memory_address >=MEMORY_RANGE-1)
	{
			TIMSK1 &= ~(1<< OCIE1A); //disable timer compare interrupt becouse eeprom is full and we shouldn't write anymore

			//infinite loop when memory is full. Data can still be sent duing this but only way to continue writing is to reset the system
			while(1){
				red_led_on();
				if (newIntFlag)
				{
					send_all_data(memory_address);
				}
				
			}
		}

		
    }
}
