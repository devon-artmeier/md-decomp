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

KOS_QUEUE_COUNT		equ $20					; Queue slot count
KOS_QUEUE_LENGTH	equ KOS_QUEUE_COUNT*8			; Queue length

kos_variables		equ $FFFF8000				; Variables
kos_stack		equ kos_variables			; Stack (2 bytes)
kos_bookmark		equ kos_stack+2				; Bookmark ($2A bytes)
kos_moduled_left	equ kos_bookmark+$2A			; Moduled graphics bytes left (2 bytes)
kos_moduled_src		equ kos_moduled_left+2			; Moduled graphics source address (4 bytes)
kos_moduled_dma		equ kos_moduled_src+4			; Moduled graphics DMA command ($E bytes)
kos_queue_read		equ kos_moduled_dma+$E			; Queue read slot (2 bytes)
kos_queue_write		equ kos_queue_read+2			; Queue write slot (2 bytes)
kos_total_count		equ kos_queue_write+2			; Queue used amount (1 bytes)
kos_regular_count	equ kos_total_count+1			; Regular data count (1 bytes)
kos_moduled_count	equ kos_regular_count+1			; Moduled graphics count (1 bytes)
kos_queue		equ kos_moduled_count+2			; Queue (KOS_QUEUE_LENGTH bytes)
kos_queue_end		equ kos_queue+KOS_QUEUE_LENGTH		; End of queue
kos_variables_end	equ kos_queue_end			; End of variables

kos_moduled_buffer	equ $FFFF9000				; Decompression buffer ($1000 bytes)

; ------------------------------------------------------------------------------
; Initialize Kosinski queue
; ------------------------------------------------------------------------------

InitKosinskiQueue:
	movem.l	d0-d1/a0,-(sp)					; Save registers

	lea	kos_variables,a0				; Variables to clear
	move.w	#(kos_variables_end-kos_variables)/2-1,d0	; Length of variables
	moveq	#0,d1						; Zero

.ClearVariables:
	move.w	d1,(a0)+					; Clear variables
	dbf	d0,.ClearVariables				; Loop until finished
	
	movem.l	(sp)+,d0-d1/a0					; Restore registers
	rts

; ------------------------------------------------------------------------------
; Queue Kosinski data for loading
; ------------------------------------------------------------------------------
; ARGUMENTS:
;	a1.l - Pointer to Kosinski data
;	a2.l - Pointer to destination buffer
; ------------------------------------------------------------------------------

QueueKosinski:
	movem.l	a0/d0,-(sp)					; Save registers

	cmpi.b	#KOS_QUEUE_COUNT,kos_total_count		; Is the queue full?
	bcc.s	.End						; If so, branch
	addq.b	#1,kos_total_count				; Increment used amount

	move.w	kos_queue_write,d0				; Get write slot
	lea	kos_queue,a0					; Queue data
	move.l	a1,(a0,d0.w)
	move.l	a2,4(a0,d0.w)

	clr.b	(a0,d0.w)					; Mark as regular data
	addq.b	#1,kos_regular_count				; Increment regular data count

	addq.w	#8,d0						; Advance write slot
	cmpi.w	#KOS_QUEUE_LENGTH,d0				; Has it reached the end of the queue?
	bcs.s	.UpdateWriteSlot				; If not, branch
	moveq	#0,d0						; If so, wrap back to the start

.UpdateWriteSlot:
	move.w	d0,kos_queue_write				; Update write slot

.End:
	movem.l	(sp)+,a0/d0					; Restore registers
	rts

; ------------------------------------------------------------------------------
; Queue Kosinski moduled graphics for loading
; ------------------------------------------------------------------------------
; ARGUMENTS:
;	a1.l - Pointer to Kosinski moduled graphics
;	d2.w - VRAM address to load graphics at
; ------------------------------------------------------------------------------

QueueKosModuled:
	movem.l	a0/d0,-(sp)					; Save registers
	
	cmpi.b	#KOS_QUEUE_COUNT,kos_total_count		; Is the queue full?
	bcc.s	.End						; If so, branch
	addq.b	#1,kos_total_count				; Increment used amount

	move.w	kos_queue_write,d0				; Get write slot
	lea	kos_queue,a0					; Queue moduled graphics
	move.l	a1,(a0,d0.w)
	move.w	d2,4(a0,d0.w)
	
	st.b	(a0,d0.w)					; Mark as moduled graphics data
	addq.b	#1,kos_moduled_count				; Increment moduled graphics count

	addq.w	#8,d0						; Advance write slot
	cmpi.w	#KOS_QUEUE_LENGTH,d0				; Has it reached the end of the queue?
	bcs.s	.UpdateWriteSlot				; If not, branch
	moveq	#0,d0						; If so, wrap back to the start

.UpdateWriteSlot:
	move.w	d0,kos_queue_write				; Update write slot

.End:
	movem.l	(sp)+,a0/d0					; Restore registers
	rts

; ------------------------------------------------------------------------------
; Flush Kosinski queue
; ------------------------------------------------------------------------------

FlushKosinskiQueue:
	move.w	sr,-(sp)					; Save registers
	move.w	#$2700,sr					; Disable interrupts

.Flush:
	tst.b	kos_total_count					; Does the queue have entries?
	beq.s	.End						; If not, branch

	bsr.s	ProcessKosinskiQueue				; Process queue
	bsr.w	DmaKosModuledBuffer				; DMA decompressed moduled graphics
	bra.s	.Flush						; Loop

.End:
	move.w	(sp)+,sr					; Restore registers
	rts

; ------------------------------------------------------------------------------
; Process Kosinski queue
; ------------------------------------------------------------------------------

ProcessKosinskiQueue:
	movem.l	d0-d4/a0-a3,-(sp)				; Save registers
	
	move.l	kos_bookmark,d0					; Has a bookmark been made?
	beq.s	.NoBookmark					; If not, branch

	move.l	d0,-(sp)					; Restore bookmark
	clr.l	kos_bookmark
	movem.l	kos_bookmark+6,d0-d4/a0-a3
	move.w	kos_bookmark+4,sr
	rts

.NoBookmark:
	lea	kos_queue,a0					; Get queue
	move.w	kos_queue_read,d0				; Get read slot

	tst.b	(a0,d0.w)					; Are moduled graphics being processed?
	bne.s	.Moduled					; If not, branch
	
	movea.l	(a0,d0.w),a1					; Source address
	movea.l	4(a0,d0.w),a2					; Destination address

	tst.b	kos_total_count					; Is any data queued up?
	bne.s	QueuedKosDecompStart				; If so, branch
	bra.s	.End						; If not, exit

.Moduled:
	tst.w	kos_moduled_dma					; Is there a DMA transfer still queued up?
	bne.s	.End						; If so, branch

	movea.l	(a0,d0.w),a1					; Source address
	lea	kos_moduled_buffer,a2				; Decompression buffer

	tst.w	kos_moduled_left				; Are we in the middle of decompressing moduled graphics?
	bne.s	QueuedKosDecompStart				; If so, branch
	tst.b	kos_total_count					; Is any data queued up?
	bne.s	.NewModuled					; If so, branch

.End:
	movem.l	(sp)+,d0-d4/a0-a3				; Restore registers
	rts

.NewModuled:
	move.w	(a1)+,kos_moduled_left				; Get uncompressed size
	move.l	a1,(a0,d0.w)					; Set decompression source address
	move.l	a1,kos_moduled_src

; ------------------------------------------------------------------------------

QueuedKosDecompStart:
	lea	kos_stack+2,a3					; Stack

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

; ------------------------------------------------------------------------------

.GetCode:
	lsr.w	#1,d2						; Get code
	bcc.s	.Code0x						; If it's 0, branch

; ------------------------------------------------------------------------------

.Code1:
	dbf	d1,.Code1NoNewDesc				; Decrement bits left to process

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

.Code1NoNewDesc:
	move.b	(a1)+,(a2)+					; Copy uncompressed byte
	bra.s	.GetCode					; Process next code

; ------------------------------------------------------------------------------

.Code0x:
	dbf	d1,.Code0xNoNewDesc				; Decrement bits left to process

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

.Code0xNoNewDesc:
	moveq	#$FFFFFFFF,d3					; Copy offsets are always negative
	moveq	#0,d4						; Reset copy counter

	lsr.w	#1,d2						; Get 2nd code bit
	bcs.s	.Code01						; If the full code is 01, branch

; ------------------------------------------------------------------------------

.Code00:
	dbf	d1,.GetCopyLength1				; Decrement bits left to process

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

.GetCopyLength1:
	lsr.w	#1,d2						; Get number of bytes to copy (first bit)
	addx.w	d4,d4
	dbf	d1,.GetCopyLength2				; Decrement bits left to process

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

.GetCopyLength2:
	lsr.w	#1,d2						; Get number of bytes to copy (second bit)
	addx.w	d4,d4
	dbf	d1,.GetCopyOffset				; Decrement bits left to process

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

.GetCopyOffset:
	move.b	(a1)+,d3					; Get copy offset

; ------------------------------------------------------------------------------

.Copy:
	lea	(a2,d3.w),a3					; Get copy address
	move.b	(a3)+,(a2)+					; Copy a byte

.CopyLoop:
	move.b	(a3)+,(a2)+					; Copy a byte
	dbf	d4,.CopyLoop					; Loop until bytes are copied

	lea	kos_stack+2,a3					; Stack
	bra.w	.GetCode					; Process next code

; ------------------------------------------------------------------------------

.Code01:
	dbf	d1,.Code01NoNewDesc				; Decrement bits left to process

	move.b	(a1)+,-(a3)					; Read from data stream
	move.b	(a1)+,-(a3)
	move.w	(a3)+,d2
	moveq	#16-1,d1					; 16 bits to process

.Code01NoNewDesc:
	move.b	(a1)+,-(a3)					; Get copy offset
	move.b	(a1)+,d3
	move.b	d3,d4
	lsl.w	#5,d3
	move.b	(a3)+,d3

	andi.w	#7,d4						; Get 3-bit copy count
	bne.s	.Copy						; If this is a 3-bit copy count, branch

	move.b	(a1)+,d4					; Get 8-bit copy count
	beq.s	QueuedKosDecompEnd				; If it's 0, we are done decompressing
	subq.b	#1,d4						; Is it 1?
	bne.s	.Copy						; If not, start copying
	
	bra.w	.GetCode					; Process next code

; ------------------------------------------------------------------------------

QueuedKosDecompEnd:
	tst.b	(a0,d0.w)					; Are moduled graphics being decompressed?
	bne.s	.Moduled					; If not, branch

	subq.b	#1,kos_regular_count				; Decrement regular data count
	bra.s	.AdvanceReadSlot				; Advance read slot

.Moduled:
	move.l	a2,d2						; Get decompressed size
	subi.l	#kos_moduled_buffer,d2
	
	move.l	kos_moduled_src,d3				; Align to next $10 byte boundary
	sub.l	a1,d3
	andi.l	#$F,d3
	add.l	a1,d3
	move.l	d3,(a0,d0.w)

	lea	kos_moduled_dma,a1				; Set up DMA registers
	move.l	#$94009300,(a1)+
	move.w	#$9700,(a1)+
	move.l	#$96009500,(a1)+

	move.l	#(kos_moduled_buffer&$FFFFFF)/2,d1		; Set DMA source address
	movep.l	d1,-7(a1)

	move.w	d2,d1						; Set DMA length
	lsr.w	#1,d1
	movep.w	d1,-9(a1)

	moveq	#0,d1						; Set VDP command
	move.w	4(a0,d0.w),d1
	lsl.l	#2,d1
	lsr.w	#2,d1
	swap	d1
	ori.l	#$40000080,d1
	move.l	d1,(a1)+

	add.w	d2,4(a0,d0.w)					; Increment VRAM address
	sub.w	d2,kos_moduled_left				; Decrement bytes left
	bne.s	.End						; If there's still some left, branch
	subq.b	#1,kos_moduled_count				; Decrement moduled graphics count

.AdvanceReadSlot:
	subq.b	#1,kos_total_count				; Decrement used amount

	clr.l	(a0,d0.w)					; Clear slot
	clr.l	4(a0,d0.w)

	addq.w	#8,d0						; Advance read slot
	cmpi.w	#KOS_QUEUE_LENGTH,d0				; Has it reached the end of the queue?
	bcs.s	.UpdateReadSlot					; If not, branch
	moveq	#0,d0						; If so, wrap back to the start

.UpdateReadSlot:
	move.w	d0,kos_queue_read				; Update read slot

.End:
	movem.l	(sp)+,d0-d4/a0-a3				; Restore registers
	rts
	
; ------------------------------------------------------------------------------
; Set Kosinski queue bookmark
; ------------------------------------------------------------------------------

SetKosinskiBookmark:
	move.w	sr,kos_bookmark+4				; Bookmark status register
	movem.l	d0-d4/a0-a3,kos_bookmark+6			; Bookmark registers

	movem.l	(sp)+,d0-d4/a0-a3				; Restore registers
	rts

; ------------------------------------------------------------------------------
; Check for Kosinski queue bookmark (call during interrupt)
; ------------------------------------------------------------------------------
; ARGUMENTS:
;	a0.l - Pointer to interrupt return address
; ------------------------------------------------------------------------------

CheckKosinskiBookmark:
	cmpi.l	#QueuedKosDecompStart,(a0)			; Was decompression interrupted?
	bcs.s	.End						; If not, branch
	cmpi.l	#QueuedKosDecompEnd,(a0)
	bcc.s	.End						; If not, branch

	move.l	(a0),kos_bookmark				; Set bookmark address
	move.l	#SetKosinskiBookmark,(a0)			; Override return address

.End:
	rts

; ------------------------------------------------------------------------------
; Perform VDP DMA transfer of decompressed Kosinski moduled graphics
; ------------------------------------------------------------------------------

DmaKosModuledBuffer:
	movem.l	a0-a1,-(sp)					; Save registers
	
	lea	$C00004,a0					; VDP control port
	lea	kos_moduled_dma,a1				; DMA command

	tst.w	(a1)						; Should we perform a DMA transfer?
	beq.s	.End						; If not, branch

	move.l	(a1)+,(a0)					; Perform DMA
	move.l	(a1)+,(a0)
	move.l	(a1)+,(a0)
	move.w	(a1)+,(a0)
	
	clr.w	-$E(a1)						; Mark DMA as performed

.End:
	movem.l	(sp)+,a0-a1					; Restore registers
	rts

; ------------------------------------------------------------------------------