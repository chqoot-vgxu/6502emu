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
    bit TIMER1_OVF_clear
    pha
    lda #(1 << PB5)
    sta PINB
    pla
    rti

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word irq	;Interrupt request vector