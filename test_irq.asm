    .include "registers.asm"
    .org $ff80

reset:
    ldx #$00
    txs
main:
    lda #(1 << PB5)
    sta DDRB
    lda #$e6
    sta TCNT1L
    lda #$f9
    sta TCNT1H
    lda #(5 << CS10)
    sta TCCR1B
    lda #(1 << TOIE1)
    sta TIMSK1
    cli

loop:
    jmp loop

irq:
    pha
    lda #(1 << PB5)
    sta PINB
    lda #$00   ; clear interrupt
    sta GPIOR1
    pla
    rti

    .org $fffa
    .word reset
    .word reset
    .word irq
