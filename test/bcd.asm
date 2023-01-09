    .org $ff00
    .include "registers.asm"

; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

counter = $0200
ascii_l = $0201
ascii_h = $0202

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

    sed

    ldx #100
    lda #$99
    sta counter
loop_adc:
    lda counter
    clc
    adc #1
    sta counter
    jsr packed_bcd_to_ascii
    lda ascii_h
    jsr uart_tx
    lda ascii_l
    jsr uart_tx
    lda #'\n'
    jsr uart_tx
    dex
    bne loop_adc

    lda #'\n'
    jsr uart_tx

    ldx #100
    lda #$00
    sta counter
loop_sbc
    lda counter
    sec
    sbc #1
    sta counter
    jsr packed_bcd_to_ascii
    lda ascii_h
    jsr uart_tx
    lda ascii_l
    jsr uart_tx
    lda #'\n'
    jsr uart_tx
    dex
    bne loop_sbc
    stp

packed_bcd_to_ascii:
    sta ascii_l
    lsr a
    lsr a
    lsr a
    lsr a
    ora #$30
    sta ascii_h
    lda ascii_l
    and #$0f
    ora #$30
    sta ascii_l
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
