    .org $fff0

rst:
    lda #1
    sec
    sbc #1
    lda #1
    sbc #0
    stp

;Setup interrupt table
    org $fffa
    word rst	;Non-maskable interrupt vector
    word rst	;Reset interrupt vector
    word rst	;Interrupt request vector