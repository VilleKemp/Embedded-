/*
 * Sensor_helmet.c
 *
 * Created: 2.3.2017 9:28:23
 * Author : Group4
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define F_CPU 1000000 // Clock Speed
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


#define channels 4 // käy läpi vain flexiforcet. jos nostetaan 6 niin lukee myös kiihtyvyysanturin
//define eeprom instructions
#define WREN 0b00000110
#define WRITE 0b00000010
#define READ 0b00000011
#define BAUDRATE 9600

uint16_t adc_result[channels];


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
ADMUX |=(0<<REFS0) | (0<<REFS1) | (0<<ADLAR);// aseta AREF referenssi jänniteeksi ja älä left adjusti
ADCSRA |=(1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable adc ja prescaler asetettu 128. Taajuuden pitää olla välillä 50k-200k. 128 arvo on 156k 20MHz MCU taajuudella


//SPI
SPCR0 |= (1<<SPE0)|(1<<MSTR0);// SPI enable ja set master


//USART
 /*Set baud rate */
 UBRR0H = (unsigned char)(MYUBRR>>8);
 UBRR0L = (unsigned char)MYUBRR;
  /* Enable receiver and transmitter */
  UCSR0B = (1<<RXEN0) | (1<<TXEN0) ;
  /* Set frame format: 8data, 1stop bit */
  UCSR0C = (1<<UCSZ00) | (1<<UCSZ01)| (0<<UMSEL00) | (0<<UMSEL01);	


}


void readSensors(){
	int channel;

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
        adc_result[channel] =ADC;
		
}
}
/*
void send_to_eeprom(char data){
	int i=0;
	/*synchronization?????
	SPDR0 = data; 
	while(!(SPSR0 & (1<<SPIF))){ 
		;
	}
	
	PINB4=0;//chip select low
		
	// /* Start transmission */
	 //SPDR0 = data;
	// /* Wait for transmission complete */
	 //while(!(SPSR0 & (1<<SPIF0))){;}	//PINB4=1; 	 //chip select high
	
//}

 void led_on(){
	 
	PORTD &= ~(1 << PIND5); // Pin 5 goes low
	PORTD &= ~(1 << PIND4); // Pin 4 goes low
	PORTD |= (1 << PIND6); // Pin 6 goes high	 

 }
 void led_off(){
	 
	PORTD &= ~(1 << PIND5); // Pin 5 goes low
	PORTD &= ~(1 << PIND4); // Pin 4 goes low
	PORTD &= ~(1 << PIND6); // Pin 6 goes low
	 

 }

void blink(){
		_delay_ms(500);
	led_on();
	_delay_ms(500);
	led_off();
	
}

void bluetooth_transmit(uint8_t data){

 /* Wait for empty transmit buffer */
 while ( !( UCSR0A & (1<<UDRE0)) )
 ;
 /* Put data into buffer, sends the data */
 blink();
 UDR0 = data;blink();



		

}
/*
ISR(USART_RX_vect) //Receive complete
{
	data = UDR0;
	UDR0 = (data);
	led_on();
	_delay_ms(500);
	led_off();
}

ISR(USART_TX_vect) //Transmit complete
{
	data = 0;
}
*/
int main(void)
{
sei();
init();
uint8_t data = 111;
unsigned char ReceivedByte;
    while (1) 
    {
		
	//	readSensors();
	
	//led();
//	bluetooth_transmit(data);
	
//	blink();
while ((UCSR0A & (1 << RXC0)) == 0) {
	
}; // Do nothing until data have been received and is ready to be read from UDR

ReceivedByte = UDR0; // Fetch the received byte value into the variable "ByteReceived"

/* Wait for empty transmit buffer */
while ( !( UCSR0A & (1<<UDRE0)) )
;
/* Put data into buffer, sends the data */
UDR0 = ReceivedByte;
    }
}

