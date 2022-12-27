; AVR peripherals registers
PINB   = $0823
DDRB   = $0824
PORTB  = $0825
PB0    = 0
PB1    = 1
PB2    = 2
PB3    = 3
PB4    = 4
PB5    = 5
PB6    = 6
PB7    = 7

PINC   = $0826
DDRC   = $0827
PORTC  = $0828
PC0    = 0
PC1    = 1
PC2    = 2
PC3    = 3
PC4    = 4
PC5    = 5
PC6    = 6

PIND   = $0829
DDRD   = $082A
PORTD  = $082B
PD0    = 0
PD1    = 1
PD2    = 2
PD3    = 3
PD4    = 4
PD5    = 5
PD6    = 6
PD7    = 7


UCSR0A = $08C0
UCSR0B = $08C1
UCSR0C = $08C2
UBRR0L = $08C4
UBRR0H = $08C5
UDR0   = $08C6
U2X0   = 1
TXEN0  = 3
USBS0  = 3
UCSZ00 = 1
UDRE0  = 5
UDRIE0 = 5

TCCR1A = $0880

TCCR1B = $0881
CS10   = 0
CS11   = 0
CS12   = 0

TCCR1C = $0882

TCNT1L = $0884
TCNT1H = $0885
TCNT1  = $0884

ICR1L  = $0886
ICR1H  = $0887
ICR1   = $0886

OCR1AL = $0888
OCR1AH = $0889
OCR1A  = $0888

OCR1BL = $088a
OCR1BH = $088b
OCR1B  = $088a

TIMSK1 = $086f
TOIE1  = 0

TIFR1  = $0836

GPIOR0 = $083e
GPIOR1 = $084a
GPIOR2 = $084b

; Interrupt controller registers
INT0_clear          = $0900
INT1_clear          = $0901
PCINT0_clear        = $0902
PCINT1_clear        = $0903
PCINT2_clear        = $0904
WDT_clear           = $0905
TIMER2_COMPA_clear  = $0906
TIMER2_COMPB_clear  = $0907
TIMER2_OVF_clear    = $0908
TIMER1_CAPT_clear   = $0909
TIMER1_COMPA_clear  = $090a
TIMER1_COMPB_clear  = $090b
TIMER1_OVF_clear    = $090c
TIMER0_COMPA_clear  = $090d
TIMER0_COMPB_clear  = $090e
TIMER0_OVF_clear    = $090f
SPI_STC_clear       = $0910
USART_RX_clear      = $0911
USART_UDRE_clear    = $0912
USART_TX_clear      = $0913
ADC_clear           = $0914
EE_READY_clear      = $0915
ANALOG_COMP_clear   = $0916
TWI_clear           = $0917
IRQ_vector          = $0918
