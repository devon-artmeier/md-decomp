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

; ------------------------------------------------------------------------------
; Decompress Enigma compressed tilemap data
; ------------------------------------------------------------------------------
; Format details: https://segaretro.org/Enigma_compression
; ------------------------------------------------------------------------------
; ARGUMENTS:
;	a0.l - Pointer to source tilemap data
;	a1.l - Pointer to destination buffer
;	d0.w - Base tile properties
; ------------------------------------------------------------------------------
; RETURNS:
;	a0.l - Pointer to end of source tilemap data
;	a1.l - Pointer to end of destination buffer
; ------------------------------------------------------------------------------

DecompEnigma:
	movem.l	d0-d7/a2-a5,-(sp)				; Save registers
	movea.w	d0,a2						; Save base tile properties

	moveq	#0,d4						; Get number of tile bits
	move.b	(a0)+,d4
	move.b	(a0)+,d0					; Get tile flags
	lsl.b	#3,d0
	movea.w	d0,a3
	movea.w	(a0)+,a4					; Get incrementing tile
	adda.w	a2,a4
	movea.w	(a0)+,a5					; Get static tile
	adda.w	a2,a5

	move.w	(a0)+,d5					; Get first word
	moveq	#16,d6

; ------------------------------------------------------------------------------

GetEnigmaCode:
	subq.w	#1,d6						; Does the next code involve using an inline tile?
	rol.w	#1,d5
	bcs.s	.InlineTileCode					; If so, branch

	subq.w	#1,d6						; Should we copy the static tile?
	rol.w	#1,d5
	bcs.s	.Mode01						; If so, branch

.Mode00:
	subq.w	#4,d6						; Get copy length
	rol.w	#4,d5
	move.w	d5,d0
	andi.w	#$F,d0
	
.Mode00Copy:
	move.w	a4,(a1)+					; Copy incrementing tile
	addq.w	#1,a4						; Increment
	dbf	d0,.Mode00Copy					; Loop until enough is copied
	bra.s	.NextCode					; Process next code

.Mode01:
	subq.w	#4,d6						; Get copy length
	rol.w	#4,d5
	move.w	d5,d0
	andi.w	#$F,d0
	
.Mode01Copy:
	move.w	a5,(a1)+					; Copy static tile
	dbf	d0,.Mode01Copy					; Loop until enough is copied
	
.NextCode:
	cmpi.w	#8,d6						; Should we get another byte?
	bhi.s	.GoToNextCode					; If not, branch

	move.w	d6,d7						; Get number of bits read past byte
	subq.w	#8,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5
	addq.w	#8,d6

.GoToNextCode:
	bra.s	GetEnigmaCode					; Process next code

.InlineTileCode:
	subq.w	#2,d6						; Get code
	rol.w	#2,d5
	move.w	d5,d1
	andi.w	#%11,d1
	
	subq.w	#4,d6						; Get copy length
	rol.w	#4,d5
	move.w	d5,d0
	andi.w	#$F,d0
	
	cmpi.w	#8,d6						; Should we get another byte?
	bhi.s	.HandleCode					; If not, branch

	move.w	d6,d7						; Get number of bits read past byte
	subq.w	#8,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5
	addq.w	#8,d6

.HandleCode:
	add.w	d1,d1						; Handle code
	jsr	.InlineCodes(pc,d1.w)
	
	bra.s	GetEnigmaCode					; Process next code

; ------------------------------------------------------------------------------

.InlineCodes:
	bra.s	EnigmaInlineMode00
	bra.s	EnigmaInlineMode01
	bra.s	EnigmaInlineMode10
	
; ------------------------------------------------------------------------------

EnigmaInlineMode11:
	cmpi.w	#$F,d0						; Are we at the end?
	beq.s	EnigmaDone					; If so, branch

.Copy:
	bsr.s	GetEnigmaInline					; Get tile
	move.w	d1,(a1)+					; Store tile
	dbf	d0,.Copy					; Loop until enough is copied
	rts
	
; ------------------------------------------------------------------------------

EnigmaDone:
	addq.w	#4,sp						; Discard return address
	
	subq.w	#1,a0						; Discard trailing byte
	cmpi.w	#16,d6						; Are there 2 trailing bytes?
	bne.s	.End						; If not, branch
	subq.w	#1,a0						; If so, discard the other byte
	
.End:
	movem.l	(sp)+,d0-d7/a2-a5				; Restore registers
	rts

; ------------------------------------------------------------------------------

EnigmaInlineMode00:
	bsr.s	GetEnigmaInline					; Get tile

.Copy:
	move.w	d1,(a1)+					; Copy tile
	dbf	d0,.Copy					; Loop until enough is copied
	rts
	
; ------------------------------------------------------------------------------

EnigmaInlineMode01:
	bsr.s	GetEnigmaInline					; Get tile

.Copy:
	move.w	d1,(a1)+					; Copy tile
	addq.w	#1,d1						; Increment
	dbf	d0,.Copy					; Loop until enough is copied
	rts
	
; ------------------------------------------------------------------------------

EnigmaInlineMode10:
	bsr.s	GetEnigmaInline					; Get tile

.Copy:
	move.w	d1,(a1)+					; Copy tile
	subq.w	#1,d1						; Decrement
	dbf	d0,.Copy					; Loop until enough is copied
	rts

; ------------------------------------------------------------------------------

GetEnigmaInline:
	move.w	a3,d7						; Get tile flags
	move.w	a2,d3						; Get base tile properties

	add.b	d7,d7						; Is the priority flag set?
	bcc.s	.CheckPalette0					; If not, branch
	subq.w	#1,d6						; Does this tile have its priority flag set?
	rol.w	#1,d5
	bcc.s	.CheckPalette0					; If not, branch
	ori.w	#1<<15,d3					; Set priority flag in base tile properties

.CheckPalette0:
	add.b	d7,d7						; Is the high palette bit set?
	bcc.s	.CheckPalette1					; If not, branch
	subq.w	#1,d6						; Does this tile have its high palette bit set?
	rol.w	#1,d5
	bcc.s	.CheckPalette1					; If not, branch
	addi.w	#1<<14,d3					; Offset palette in base tile properties

.CheckPalette1:
	add.b	d7,d7						; Is the low palette bit set?
	bcc.s	.CheckYFlip					; If not, branch
	subq.w	#1,d6						; Does this tile have its low palette bit set?
	rol.w	#1,d5
	bcc.s	.CheckYFlip					; If not, branch
	addi.w	#1<<13,d3					; Offset palette in base tile properties

.CheckYFlip:
	add.b	d7,d7						; Is the Y flip flag set?
	bcc.s	.CheckXFlip					; If not, branch
	subq.w	#1,d6						; Does this tile have its Y flip bit set?
	rol.w	#1,d5
	bcc.s	.CheckXFlip					; If not, branch
	ori.w	#1<<12,d3					; Set Y flip flag in base tile properties

.CheckXFlip:
	add.b	d7,d7						; Is the X flip flag set?
	bcc.s	.GotFlags					; If not, branch
	subq.w	#1,d6						; Does this tile have its X flip bit set?
	rol.w	#1,d5
	bcc.s	.GotFlags					; If not, branch
	ori.w	#1<<11,d3					; Set X flip flag in base tile properties

.GotFlags:
	cmpi.w	#8,d6						; Should we get another byte?
	bhi.s	.GetTileId					; If not, branch

	move.w	d6,d7						; Get number of bits read past byte
	subq.w	#8,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5
	addq.w	#8,d6

.GetTileId:
	moveq	#0,d2						; Reset upper bits
	move.w	d4,d1						; Get number of bits in a tile ID
	cmpi.w	#8,d1						; Is it more than 8 bits?
	bls.s	.GotTileId					; If not, branch
	
	rol.w	#8,d5						; Get first 8 bits of tile ID
	move.b	d5,d2
	
	subq.w	#8,d1						; Get remaining number of bits
	lsl.w	d1,d2
	
	move.w	d6,d7						; Get number of bits read past byte
	subi.w	#16,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5

.GotTileId:
	sub.w	d1,d6						; Get tile ID bits
	rol.w	d1,d5
	
	move.w	d1,d7						; Apply mask and base tile properties
	add.w	d7,d7
	move.w	d5,d1
	and.w	.Masks-2(pc,d7.w),d1
	or.w	d2,d1
	add.w	d3,d1
	
	cmpi.w	#8,d6						; Should we get another byte?
	bhi.s	.End						; If not, branch

	move.w	d6,d7						; Get number of bits read past byte
	subq.w	#8,d7
	neg.w	d7
	
	ror.w	d7,d5						; Read another byte
	move.b	(a0)+,d5
	rol.w	d7,d5
	addq.w	#8,d6

.End:
	rts

; ------------------------------------------------------------------------------

.Masks:
	dc.w	%0000000000000001
	dc.w	%0000000000000011
	dc.w	%0000000000000111
	dc.w	%0000000000001111
	dc.w	%0000000000011111
	dc.w	%0000000000111111
	dc.w	%0000000001111111
	dc.w	%0000000011111111
	dc.w	%0000000111111111
	dc.w	%0000001111111111
	dc.w	%0000011111111111
	dc.w	%0000111111111111
	dc.w	%0001111111111111
	dc.w	%0011111111111111
	dc.w	%0111111111111111
	dc.w	%1111111111111111
	
; ------------------------------------------------------------------------------