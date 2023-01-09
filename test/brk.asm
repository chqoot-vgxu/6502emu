    .org $ff40
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
    lda #(1 << TXEN0)
    sta UCSR0B
    lda #(1 << USBS0) | (3 << UCSZ00)
    sta UCSR0C
    brk #0
    .byte 'B'
    brk
    .byte 'R'
    brk
    .byte 'K'
    lda #'\n'
    jsr uart_tx
    stp

uart_tx:
    pha
    lda #(1 << UDRE0)
wait:
    bit UCSR0A
    beq wait
    pla
    sta UDR0
    rts

brk_ptr = $00fe
irq:
    pha
    phx
    tsx
    lda $0103,x
    and #$10
    beq irq_end
    lda $0104,x
    sec
    sbc #1
    sta brk_ptr
    lda $0105,x
    sbc #0
    sta brk_ptr+1
    lda (brk_ptr)
    jsr uart_tx
irq_end:
    plx
    pla
    rti

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word irq	;Interrupt request vector