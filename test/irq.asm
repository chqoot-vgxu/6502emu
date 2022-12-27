    .org $ff80
    .include "registers.asm"

rst:
    ldx #$ff
    txs
    lda #(1 << PB5)
    sta DDRB
    lda #(4 << CS10)
    sta TCCR1B
    lda #(1 << TOIE1)
    sta TIMSK1
    cli

loop:
    bra loop

irq:
    pha
    lda #(1 << PB5)
    sta PINB
    bit TIMER1_OVF_vect_clear
    pla
    rti

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word irq	;Interrupt request vector