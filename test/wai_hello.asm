    .org $ff80
    .include "registers.asm"

; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

rst:
    ldx #$ff
    txs
    lda #BAUDRATE_L
    sta UBRR0L
    lda #BAUDRATE_H
    sta UBRR0H
    lda #(1 << U2X0)
    sta UCSR0A
    lda #(1 << TXEN0) | (1 << UDRIE0)
    sta UCSR0B
    lda #(1 << USBS0) | (3 << UCSZ00)
    sta UCSR0C
    sei
    ldy #$00
tx_next:
    lda hello_world,y
    beq exit
    sta UDR0
    iny
    wai
    bit USART_UDRE_clear
    bra tx_next
exit:
    stp

hello_world:
    .byte "Hello, world!\n\0"

;Setup interrupt table
    org $fffa
    word rst	;Non-maskable interrupt vector
    word rst	;Reset interrupt vector
    word rst	;Interrupt request vector
