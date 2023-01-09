    .org $ff00
    .include "registers.asm"

buffer = $0200
buffer_size  = 32
buffer_head  = $0220
buffer_tail  = $0221
uart_tx_char = $0222

index   = $0223

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
    stz buffer_head
    stz buffer_tail
    stz index
tx_next:
    ldy index
    lda hello_world,y
    beq exit
    inc index
    jsr uart_tx
    bra tx_next
exit:
    lda buffer_head
wait$:
    cmp buffer_tail
    bne wait$
    stp

uart_tx:
    sta uart_tx_char
    lda buffer_head
    cmp buffer_tail
    bne load_into_buffer$
    lda #(1 << UDRE0)
    bit UCSR0A
    beq load_into_buffer$
    lda uart_tx_char
    sta UDR0
    rts
load_into_buffer$:
    lda buffer_head
    inc a
    and #buffer_size-1
wait$:
    cmp buffer_tail
    beq wait$
    tax
    lda uart_tx_char
    ldy buffer_head
    sta buffer,y
    sei
    stx buffer_head
    lda #(1 << UDRIE0)
    ora UCSR0B
    sta UCSR0B
    cli
    rts

irq:
    bit USART_UDRE_clear
    pha
    phy
    ldy buffer_tail
    lda buffer,y
    sta UDR0
    tya
    inc a
    and #buffer_size-1
    sta buffer_tail
    cmp buffer_head
    bne end_irq$
    lda #~(1 << UDRIE0)
    and UCSR0B
    sta UCSR0B
end_irq$:
    ply
    pla
    rti

hello_world:
    .byte "Hello, world!\n\0"

    ;Setup interrupt table
    .org $fffa
    .word rst	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word irq	;Interrupt request vector
