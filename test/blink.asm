    .org $ff80
    .include "registers.asm"

rts:
    lda #(1 << PB5)
    sta DDRB
loop:
    sta PINB
    jsr delay
    bra loop

delay_l = $0200
delay_h = $0201

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

;Setup interrupt table
    org $fffa
    word rts	;Non-maskable interrupt vector
    word rst	;Reset interrupt vector
    word rts	;Interrupt request vector