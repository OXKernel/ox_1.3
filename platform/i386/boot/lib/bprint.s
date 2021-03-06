;
; This program is free software; you can redistribute it and/or
; modify it under the terms of the GNU General Public License
; as published by the Free Software Foundation; either version 2
; of the License, or (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
; 
; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Copyright (C) Roger G. Doss. All Rights Reserved. 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; @module
; 	bprint.s
; @description
;	implementation of bprint
;
; @author
;	Roger G. Doss
;
%include "include/common.inc"

[BITS 16]

;
; bprint:
;	print ds:si to screen
;
ENTRY bprint
	lodsb			; load byte from ds:si to al
	cmp al,0		; test if at end of string
	je finished
		mov bx,0x0001	; request to write on screen
		mov ah,0xE	; in tele-type mode
		int 0x10	; call BIOS
	jmp bprint
finished:
	ret

;
; EOF
;
