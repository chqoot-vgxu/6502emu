    .org $ff00
    .include "registers.asm"

buffer = $0200
buffer_size = 32
buffer_head = $0020
buffer_tail = $0021

delay_l = $0022
delay_h = $0023

uart_tx_char = $0024

; baud 115200
BAUDRATE_L = $10
BAUDRATE_H = $00

rst:
    ldx #$ff
    txs
    lda #BAUDRATE_L
    ldx #BAUDRATE_H
    jsr uart_init
    lda #(1 << PB5)
    sta DDRB
    cli
loop:
    lda #(1 << PB5)
    sta PINB

    ldy #$00
tx_next:
    lda hello_world,y
    beq cont
    iny
    jsr uart_tx
    bra tx_next
cont:
    lda #$FF
    sta delay_l
    sta delay_h
    jsr delay
    jmp loop

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

uart_init:
    sta UBRR0L
    stx UBRR0H
    lda #(1 << U2X0)
    sta UCSR0A
    lda #(1 << TXEN0)
    sta UCSR0B
    lda #(1 << USBS0) | (3 << UCSZ00)
    sta UCSR0C
    stz buffer_head
    stz buffer_tail
    rts

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
    phy
    pha
    lda uart_tx_char
    ldy buffer_head
    sta buffer,y
    pla
    sei
    sta buffer_head
    lda #(1 << UDRIE0)
    ora UCSR0B
    sta UCSR0B
    cli
    ply
    rts

irq:
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
    bit USART_UDRE_vect_clear
    ply
    pla
    rti

nmi:
    jmp rst

hello_world:
    .byte "Hello, world!\n\0"

    ;Setup interrupt table
    .org $fffa
    .word nmi	;Non-maskable interrupt vector
    .word rst	;Reset interrupt vector
    .word irq	;Interrupt request vector
