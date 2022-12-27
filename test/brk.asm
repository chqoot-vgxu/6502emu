
; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

    .org $ff40
    .include "registers.asm"

reset:
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
    brk
    nop
    lda #$FF
    sta R0
    sta R1
    jsr delay
    jmp loop

delay:
    lda R0
    bne label$
    lda R1
    beq end$
    dec R1
label$:
    dec R0
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
    lda #(1 << UDRE0)
    bit UCSR0A
    beq uart_tx
    lda R0
    sta UDR0
    rts

hello_world:
    .byte "Hello, world!\n\0"

irq:
    phy
    ldy #$00
tx_next:
    lda hello_world,y
    sta R0
    jsr uart_tx
    iny
    cmp #$00
    bne tx_next
    ply
    rti

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word irq	;Interrupt request vector