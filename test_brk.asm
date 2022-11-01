
DELAY  = $0200 ; 2 bytes

; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

    .org $ff40
    .include "registers.asm"

reset:
    ldx #$00
    txs
main:
    lda #BAUDRATE_L
    ldx #BAUDRATE_H
    jsr uart_init
    lda #(1 << PB5)
    sta DDRB
loop:
    lda #(1 << PB5)
    sta PINB
    brk
    nop
    lda #$FF
    sta DELAY
    sta DELAY+1
    jsr delay
    jmp loop

delay:
    lda DELAY
    bne label$
    lda DELAY+1
    beq end$
    dec DELAY+1
label$:
    dec DELAY
    jmp delay
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
wait$:
    lda #(1 << UDRE0)
    bit UCSR0A
    beq wait$
    pla
    sta UDR0
    rts

hello_world:
    .byte "Hello, world!", $0a, $00

irq:
    pha
    tya
    pha
    ldy #$00
tx_next:
    lda hello_world,y
    jsr uart_tx
    iny
    cmp #$00
    bne tx_next
    pla
    tay
    pla
    rti

    .org $fffa
    .word reset
    .word reset
    .word irq
