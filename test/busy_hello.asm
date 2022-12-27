
    .org $ff80
    .include "registers.asm"

; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

delay_l = $0022
delay_h = $0023

rst:
    ldx #$ff
    txs
    lda #BAUDRATE_L
    ldx #BAUDRATE_H
    jsr uart_init
    lda #(1 << PB5)
    sta DDRB
loop:
    lda #(1 << PB5)
    sta PINB

    ldy #$00
tx_next:
    lda hello_world,y
    beq cont
    iny
    jsr uart_tx
    bra tx_next
cont:
    lda #$FF
    sta delay_l
    sta delay_h
    jsr delay
    jmp loop

delay:
    lda delay_l
    bne label$
    lda delay_h
    beq end$
    dec delay_h
label$:
    dec delay_l
    bra delay
end$:
    rts

uart_init:
    sta UBRR0L
    stx UBRR0H
    lda #(1 << U2X0)
    sta UCSR0A
    lda #(1 << TXEN0)
    sta UCSR0B
    lda #(1 << USBS0) | (3 << UCSZ00)
    sta UCSR0C
    rts

uart_tx:
    pha
    lda #(1 << UDRE0)
wait$:
    bit UCSR0A
    beq wait$
    pla
    sta UDR0
    rts

hello_world:
    .byte "Hello, world!\n\0"

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word rst	;Interrupt request vector
