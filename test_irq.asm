    .org $ff80

PINB  = $0829
DDRB  = $082a
PORTB = $082b
PB5   = 5

PIND  = $0829
DDRD  = $082a
PORTD = $082b
PD3   = 3

GPIOR0 = $083e
GPIOR1 = $084a
GPIOR2 = $084b

EIMSK = $083d
INT1  = 1

EICRA = $0869
ISC10 = 2

reset:
    ldx #$00
    txs
main:
    lda #(1 << PB5)
    sta DDRB
    lda #(1 << PD3)
    sta PORTD
    lda #(2 << ISC10)
    sta EICRA
    lda #(1 << INT1)
    sta EIMSK
    cli
loop:
    jmp loop

irq:
    pha
    lda #(1 << PB5)
    sta PINB
    lda #$00   ; clear interrupt
    sta GPIOR0
    pla
    rti

    .org $fffa
    .word reset
    .word reset
    .word irq
