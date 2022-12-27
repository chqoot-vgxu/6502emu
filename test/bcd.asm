    .org $ff00
    .include "registers.asm"

; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

rst:
    ldx #$ff
    txs

    lda #BAUDRATE_L
    ldx #BAUDRATE_H
    jsr uart_init

    ldx #0
    lda #1
    sta R0
    lda #$99
    sed
loop_adc:
    clc
    adc R0
    pha
    jsr packed_bcd_to_ascii
    lda R2
    jsr uart_tx
    lda R1
    jsr uart_tx
    lda #$0a
    jsr uart_tx
    pla
    inx
    cpx #100
    bne loop_adc

    lda #$0a
    jsr uart_tx

    ldx #0
    lda #1
    sta R0
    lda #$00
    sed
loop_sbc
    sec
    sbc R0
    pha
    jsr packed_bcd_to_ascii
    lda R2
    jsr uart_tx
    lda R1
    jsr uart_tx
    lda #$0a
    jsr uart_tx
    pla
    inx
    cpx #100
    bne loop_sbc
    stp

packed_bcd_to_ascii:
    sta R1
    clc
    lsr a
    lsr a
    lsr a
    lsr a
    and #$0f
    ora #$30
    sta R2
    lda R1
    and #$0f
    ora #$30
    sta R1
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

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word rst	;Interrupt request vector
