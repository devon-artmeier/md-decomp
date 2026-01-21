; ------------------------------------------------------------------------------
; Copyright (c) 2025-2026 Devon Artmeier
;
; Permission to use, copy, modify, and/or distribute this software
; for any purpose with or without fee is hereby granted.
;
; THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
; WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIE
; WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
; AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
; DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
; PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER 
; TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
; PERFORMANCE OF THIS SOFTWARE.
; ------------------------------------------------------------------------------

nemesis_code_table	equ $FFFFAA00				; Code table buffer ($200 bytes)

; ------------------------------------------------------------------------------
; Decompress Nemesis compressed graphics data
; ------------------------------------------------------------------------------
; Format details: https://segaretro.org/Nemesis_compression
; ------------------------------------------------------------------------------
; When writing to VDP memory, set the VDP command first before calling.
; Requires $200 bytes allocated in RAM for the code table.
; ------------------------------------------------------------------------------
; ARGUMENTS:
;	a0.l - Pointer to source graphics data
;	a4.l - Pointer to destination buffer (RAM write only)
; ------------------------------------------------------------------------------

DecompNemesis:
	movem.l	d0-a5,-(sp)					; Save registers
	lea	WriteNemesisRow(pc),a3				; Write to RAM
	bsr.s	DecompNemesisMain				; Decompress data
	movem.l	(sp)+,d0-a5					; Restore registers
	rts

; ------------------------------------------------------------------------------

DecompNemesisVram:
	movem.l	d0-a5,-(sp)					; Save registers
	lea	WriteNemesisRowVram(pc),a3			; Write to VRAM
	lea	$C00000,a4					; VDP data port
	bsr.s	DecompNemesisMain				; Decompress data
	movem.l	(sp)+,d0-a5					; Restore registers
	rts
	
; ------------------------------------------------------------------------------

DecompNemesisMain:
	lea	nemesis_code_table,a1				; Code table buffer
	
	move.w	(a0)+,d0					; Get number of tiles
	bpl.s	.NotXor						; If XOR mode is not set, branch
	lea	$A(a3),a3					; Use XOR version of data writer
	
.NotXor:
	lsl.w	#3,d0						; Get number of 8 pixel rows
	movea.w	d0,a5
	
	bsr.w	BuildNemesisCodeTable				; Build code table
	
	moveq	#8,d3						; Reset pixel count
	moveq	#0,d2						; Clear XOR pixel row data
	moveq	#0,d4						; Clear pixel row data
	
	move.b	(a0)+,-(sp)					; Get first word
	move.w	(sp)+,d5
	move.b	(a0)+,d5
	moveq	#16,d6

; ------------------------------------------------------------------------------

GetNemesisCode:
	cmpi.w	#%1111110000000000,d5				; Are the high 6 bits set in the code?
	bcc.s	GetNemesisInline				; If so, branch
	
	moveq	#0,d1						; Get code table entry index
	move.w	d5,-(sp)
	move.b	(sp)+,d1
	add.w	d1,d1
	
	moveq	#0,d0						; Advance bitstream past code
	move.b	(a1,d1.w),d0
	sub.w	d0,d6
	rol.w	d0,d5
	
	move.b	1(a1,d1.w),d1					; Get pixel value and repeat count
	
StartNemesisPixelCopy:
	cmpi.w	#8,d6						; Should we get another byte?
	bhi.s	.GetPixel					; If not, branch

	move.w	d6,d7						; Get number of bits read past byte
	subq.w	#8,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5
	addq.w	#8,d6

.GetPixel:
	move.w	d1,d0						; Get pixel value
	andi.w	#$F,d1
	andi.w	#$70,d0						; Get repeat count
	lsr.w	#4,d0
	
WriteNemesisPixel:
	lsl.l	#4,d4						; Write pixel
	or.b	d1,d4
	
	subq.w	#1,d3						; Decrement number of pixels in row
	beq.s	.WriteRow					; If the row is fully written, branch

	dbf	d0,WriteNemesisPixel				; Loop until repeated pixels are written
	bra.s	GetNemesisCode					; Process next code
	
.WriteRow:
	jmp	(a3)						; Write pixel row to memory

NewNemesisRow:
	moveq	#8,d3						; Reset pixel count
	moveq	#0,d4						; Reset pixel row data

	dbf	d0,WriteNemesisPixel				; Loop until repeated pixels are written
	bra.s	GetNemesisCode					; Process next code

; ------------------------------------------------------------------------------

GetNemesisInline:
	subq.w	#6,d6						; Advance bitstream past code
	rol.w	#6,d5

	cmpi.w	#8,d6						; Should we get another byte?
	bhi.s	.GetInlineData					; If not, branch

	move.w	d6,d7						; Get number of bits read past byte
	subq.w	#8,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5
	addq.w	#8,d6

.GetInlineData:
	subq.w	#7,d6						; Get inline data
	rol.w	#7,d5
	
	move.w	d5,d1						; Start copying pixel from inline data
	bra.s	StartNemesisPixelCopy
	
; ------------------------------------------------------------------------------

WriteNemesisRowVram:
	move.l	d4,(a4)						; Write pixel row
	subq.w	#1,a5						; Decrement number of pixel rows left
	move.w	a5,d7
	bne.s	NewNemesisRow					; If there's still pixel rows to write, branch
	rts

WriteNemesisRowVramXor:
	eor.l	d4,d2						; XOR previous pixel row with current pixel row
	move.l	d2,(a4)						; Write pixel row
	subq.w	#1,a5						; Decrement number of pixel rows left
	move.w	a5,d7
	bne.s	NewNemesisRow					; If there's still pixel rows to write, branch
	rts
	
; ------------------------------------------------------------------------------

WriteNemesisRow:
	move.l	d4,(a4)+					; Write pixel row
	subq.w	#1,a5						; Decrement number of pixel rows left
	move.w	a5,d7
	bne.s	NewNemesisRow					; If there's still pixel rows to write, branch
	rts

WriteNemesisRowXor:
	eor.l	d4,d2						; XOR previous pixel row with current pixel row
	move.l	d2,(a4)+					; Write pixel row
	subq.w	#1,a5						; Decrement number of pixel rows left
	move.w	a5,d7
	bne.s	NewNemesisRow					; If there's still pixel rows to write, branch
	rts
	
; ------------------------------------------------------------------------------

BuildNemesisCodeTable:
	move.b	(a0)+,d0					; Get byte
	bpl.s	.NotPaletteIndex				; If it's not a pixel value, branch
	
	cmpi.b	#$FF,d0						; Are we at the end?
	beq.s	.End						; If so, branch
	
	move.b	d0,d2						; Get pixel value
	bra.s	BuildNemesisCodeTable				; Get next byte

.NotPaletteIndex:
	moveq	#$F,d1						; Mask out pixel value and code length
	and.w	d1,d2
	and.w	d0,d1
	
	ext.w	d0						; Form code table entry
	add.w	d0,d0
	or.w	.ShiftedCodes(pc,d0.w),d2
	
	subq.w	#8,d1						; Get shift value based on code length
	neg.w	d1
	
	move.b	(a0)+,d0					; Get code table index
	lsl.w	d1,d0
	add.w	d0,d0
	
	lea	(a1,d0.w),a2					; Get first code table entry
	move.b	.EntryCounts(pc,d1.w),d1			; Get entry count
	
.StoreCode:
	move.w	d2,(a2)+					; Store code table entry
	dbf	d1,.StoreCode					; Loop until finished
	
	bra.s	BuildNemesisCodeTable				; Get next byte
		
.End:
	rts

; ------------------------------------------------------------------------------

.EntryCounts:
	dc.b	(1<<0)-1, (1<<1)-1, (1<<2)-1, (1<<3)-1
	dc.b	(1<<4)-1, (1<<5)-1, (1<<6)-1, (1<<7)-1
	dc.b	(1<<8)-1
	even

.ShiftedCodes:
	dc.w	$000, $100, $200, $300, $400, $500, $600, $700
	dc.w	$800, $900, $A00, $B00, $C00, $D00, $E00, $F00
	dc.w	$010, $110, $210, $310, $410, $510, $610, $710
	dc.w	$810, $910, $A10, $B10, $C10, $D10, $E10, $F10
	dc.w	$020, $120, $220, $320, $420, $520, $620, $720
	dc.w	$820, $920, $A20, $B20, $C20, $D20, $E20, $F20
	dc.w	$030, $130, $230, $330, $430, $530, $630, $730
	dc.w	$830, $930, $A30, $B30, $C30, $D30, $E30, $F30
	dc.w	$040, $140, $240, $340, $440, $540, $640, $740
	dc.w	$840, $940, $A40, $B40, $C40, $D40, $E40, $F40
	dc.w	$050, $150, $250, $350, $450, $550, $650, $750
	dc.w	$850, $950, $A50, $B50, $C50, $D50, $E50, $F50
	dc.w	$060, $160, $260, $360, $460, $560, $660, $760
	dc.w	$860, $960, $A60, $B60, $C60, $D60, $E60, $F60
	dc.w	$070, $170, $270, $370, $470, $570, $670, $770
	dc.w	$870, $970, $A70, $B70, $C70, $D70, $E70, $F70

; ------------------------------------------------------------------------------