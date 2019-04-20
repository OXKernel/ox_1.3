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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Copyright (C) Roger George Doss. All Rights Reserved.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; @module
;	memmove.s
; @description
;	Generic memory utilities
; @author
;	Roger George Doss
;
%include "include/common.inc"

[BITS 16]

;
; memmove:-
; Move contents in memory to another memory
; location.
; ds:esi source
; ds:edi destination
; cx     length
ENTRY memmove
	pusha
.move:
	mov byte al,[esi]
	mov byte [edi],al
	inc esi
	inc edi
	a32 loop .move
	popa
	ret

;
; EOF
;