\ 6809 Emulator in FORTH
\ Copyright (C) 2023 Alexandre Dumont <adumont@gmail.com>
\ SPDX-License-Identifier: GPL-3.0-only
\
\ Host Forth is gForth
\ Target CPU is Motorola 6809

HEX

\ Some compatibility words for gforth
: N. ( print_nibble ) $F AND DUP $A >= IF $67 + THEN $30 XOR EMIT ;
: X. ( a n ) >R R@ 0 DO $10 /MOD LOOP DROP R> 0 DO N. LOOP ; \ print n chars of a
: C. ( byte ) 2 X. SPACE ;
: W. ( word ) 4 X. SPACE ;
: BIN. 8 0 DO DUP $80 AND IF ." 1" ELSE ." 0" THEN 2* LOOP DROP ;

\ : GETC BEGIN KEY? UNTIL KEY ;   \ blocking
: GETC KEY? IF KEY EXIT THEN 0 ;  \ non-blocking
: 2+ 1+ 1+ ;
: EXEC EXECUTE ;
: NEG 0 SWAP - ;
: NOT NEG 1 - ;
: CLS CLEARSTACK ;
: W, ( word -- ) $100 /MOD SWAP C, C, ; \ word comma

: C? C@ C. ;
: W? W@ W. ;
\ : CELLS CELL * ;

\ 6809 Registers
CREATE _D  0 W, \ [ A ][ B ] Accumulators
: _A _D 1+ ; \ two helper words so we can operate on _A & B
: _B _D    ; \ using regular C! & C@.

CREATE _X  0 W,
CREATE _Y  0 W,
CREATE _S  0 W,
CREATE _U  0 W,
CREATE _PC 0 W,
CREATE _DP 0 C, \ Direct Page Register
CREATE _CC $50 C, \ Condition Code Register

\ Array _RR stores adresses of X, Y, U and S registers
\ It'll be used in Index Addressing Modes
CREATE _RR _X , _Y , _U , _S ,

: LDCC  ( byte -- ) _CC C! ;

CREATE OPCODES $100 CELLS ALLOT
CREATE RAM $10000 ALLOT \ Full 6502 memory space
\ 0000-00FF ZP
\ 0100-01FF 6502 Stack
\ 0200-02FD user memory
$F004 CONSTANT IN_CHAR
$F001 CONSTANT OU_CHAR

\ Target RAM operations
DEFER READ_HOOK
DEFER WRIT_HOOK

\ Target memory operations

: TC@ ( addr -- byte ) $FFFF AND   READ_HOOK ( ELSE ) RAM + C@ ;
: TC! ( byte addr -- ) $FFFF AND   WRIT_HOOK ( ELSE ) RAM + C! ;

: TW@  ( addr -- word )
  >R
  R@    TC@ 8 LSHIFT \ HI byte
  R> 1+ TC@          \ LO byte
  OR
;

: TW!  ( word addr -- )
  >R
  $100 /MOD ( LO HI )
  R@    TC!
  R> 1+ TC!
;

: CHAR_IN? ( addr -- byte ) DUP IN_CHAR  = IF DROP GETC RDROP THEN ;
: CHAR_OU? ( byte addr -- ) DUP OU_CHAR  = IF DROP EMIT RDROP THEN ;

' CHAR_IN? IS READ_HOOK
' CHAR_OU? IS WRIT_HOOK

0000 VALUE THERE \ Target HERE
: TC, ( b -- ) THERE TC!   THERE 1+ TO THERE ;

\ -- PC --

: _PC! _PC W! ;

\ Fetch a BYTE and advance PC by 1
: BYTE@ ( -- byte )
  _PC W@ DUP 1+ _PC! TC@
;

\ Fetch a WORD and advance PC by 2
: WORD@ ( -- byte )
  _PC W@ DUP 2+ _PC! TW@
;

: DUMPPC _PC W@ DUP 5 + SWAP DO I TC@ C. LOOP ;
: TDUMP ( t-addr n -- ) SWAP RAM + SWAP DUMP ;

1 VALUE TRACE  \ flag, show Status or not in NEXT

\ A  B  X    Y    U    S   DP EFHINZVC  PC
\ AABB FFFF AAAA AAAA AAAA 00 01011010 FFFF

: STATUS
  CR ." A  B X    Y    U    S    DP EFHINZVC CC PC"
  CR _D W? _X W? _Y W? _U W? _S W? _DP C? _CC C@ DUP BIN. SPACE C. _PC W? ." > " DUMPPC ;

-1 VALUE LASTINSTR

0 VALUE LASTPC

: NEXT
  ( Save PC ) _PC W@ TO LASTPC
  ( FETCH   ) BYTE@ DUP TO LASTINSTR
  ( DECODE  ) CELLS OPCODES + @
  ( EXECUTE ) EXEC
  TRACE IF STATUS THEN ;

DEFER BREAKPOINT

\ examples breakpoints words:
\ :NONAME _A C@ 1 = ; IS BREAKPOINT \ break when A=1
\ :NONAME C>    1 = ; IS BREAKPOINT \ break when C is set

\ Default breakpoint is when next instr. is BRK ($00)
: BPONBRK _PC W@ TC@ 0= ;
' BPONBRK IS BREAKPOINT

\ breakpoint on PC=PCBP
0 VALUE PCBP
: BPONPC ( -- f ) _PC W@ PCBP = ;

\ set breakpoint at addr
: BREAKAT ( addr -- ) TO PCBP ['] BPONPC IS BREAKPOINT ;

\ run until breakpoint is reached
: RUN ( n -- )
  TRACE >R         \ save TRACE
   0 TO TRACE      \ no trace
  -1 TO LASTINSTR  \ reset last instr to -1
  BEGIN
    BREAKPOINT 0=
  WHILE
    NEXT
  REPEAT
  R> TO TRACE      \ restore TRACE
  STATUS ;

: BIND ( xt opcode -- )   CELLS OPCODES + ! ; \ saves XT in OPCODES table
: BIND2 ( xt opcode -- )  2DROP ( TBD )  ; \ special case of 2 bytes opcodes $10xx & $11xx

\ -- Processor Flags handling
$80 VALUE 'E    \ Entire Flag
$40 VALUE 'F    \ FIRQ Mask
$20 VALUE 'H    \ Half Carry
$10 VALUE 'I    \ IRQ Mask
$08 VALUE 'N    \ Negative
$04 VALUE 'Z    \ Zero
$02 VALUE 'V    \ Overflow
$01 VALUE 'C    \ Carry

: CLEAR ( mask -- ) NOT _CC C@ AND _CC C! ;
: SET   ( mask -- )     _CC C@ OR  _CC C! ;
: UPDATE-FLAG ( b/f reg -- ) SWAP IF SET ELSE CLEAR THEN ;

: >N   ( b -- b ) DUP   $80 AND 'N UPDATE-FLAG ; \ non-droppy, byte version
: >NW  ( w -- w ) DUP $8000 AND 'N UPDATE-FLAG ; \ non-droppy, word version
: >Z   ( b -- b ) DUP        0= 'Z UPDATE-FLAG ; \ non-droppy
: >V   ( f --   )               'V UPDATE-FLAG ; \ this one is droppy
: >C   ( f --   )               'C UPDATE-FLAG ; \ this one is droppy

: C>   (   -- f ) _CC C@ 'C AND ;
: D>   (   -- f ) _CC C@ 'D AND ;
: I>   (   -- f ) _CC C@ 'I AND ;

\ Addressing modes words
: 'DP  ( -- addr ) BYTE@ _DP C@  8 LSHIFT OR ;   \ Direct Addressing
: 'EA  ( -- addr ) WORD@ ;                       \ Extended Addressing

: ?[] ( addr b -- addr ) $10 AND IF TW@ THEN ;   \ apply indirection if b=1

\ Sign extension words
: SIGNEX5 DUP $10 AND IF $FFE0 OR THEN ; \ 5 bits --> 16 bits
: SIGNEX8 DUP $80 AND IF $FF00 OR THEN ; \ 8 bits --> 16 bits

\ Indexed Addressing Modes
: 'IND ( -- addr )
  BYTE@ >R \ get postbyte
  R@ 5 RSHIFT %11 AND CELLS _RR + @ \ Addr of ,R
  R@ $80 AND 0= IF
    \ b7 = 0
    \ EA = ,R + 5 bit offset (twos complement)
    ( ,R ) W@
    R@ %11111 AND SIGNEX5 +
  ELSE
    \ b7 = 1
    R@ $F AND \ 4 lsb indicate mode
    CASE
      ( ,R+          ) %0000 OF DUP W@ DUP 1+ ROT W!          ENDOF
      ( ,R++         ) %0001 OF DUP W@ DUP 2+ ROT W!   R@ ?[] ENDOF
      ( ,-R          ) %0010 OF DUP W@ 1- DUP ROT W!          ENDOF
      ( ,--R         ) %0011 OF DUP W@ 1- DUP ROT W!   R@ ?[] ENDOF
      ( ,R           ) %0100 OF DUP W@                 R@ ?[] ENDOF
      ( A,R          ) %0110 OF DUP W@ _A C@ SIGNEX8 + R@ ?[] ENDOF
      ( B,R          ) %0101 OF DUP W@ _B C@ SIGNEX8 + R@ ?[] ENDOF
      ( ,R +  8b off ) %1000 OF DUP W@ BYTE@ SIGNEX8 + R@ ?[] ENDOF
      ( ,R + 16b off ) %1001 OF DUP W@ WORD@         + R@ ?[] ENDOF
      ( D,R          ) %1011 OF DUP W@ _D W@         + R@ ?[] ENDOF
      ( ,PC + 8b off ) %1100 OF LASTPC BYTE@ SIGNEX8 + R@ ?[] ENDOF \ LASTPC or _PC W@ ??
      ( ,PC +16b off ) %1101 OF LASTPC WORD@         + R@ ?[] ENDOF \ LASTPC or _PC W@ ??
      ( [,Addr]      ) %1111 OF WORD@                  TW@    ENDOF
    ENDCASE
  THEN
  R> DROP
  $FFFF AND \ trim to 16 bit
;
\ Opcodes definitions

:NONAME ( ABX   inh ) _X W@ _B C@ + _X W! ; $3A BIND

:NONAME ( ANDCC imm ) _CC C@ BYTE@ AND _CC C! ; $1C BIND
:NONAME ( ORCC  imm ) _CC C@ BYTE@ OR  _CC C! ; $1A BIND

:NONAME ( ADCA  imm ) ; $89 BIND
:NONAME ( ADCA  dir ) ; $99 BIND
:NONAME ( ADCA  ext ) ; $B9 BIND
:NONAME ( ADCA  ind ) ; $A9 BIND

:NONAME ( ADCB  imm ) ; $C9 BIND
:NONAME ( ADCB  dir ) ; $D9 BIND
:NONAME ( ADCB  ext ) ; $F9 BIND
:NONAME ( ADCB  ind ) ; $E9 BIND

:NONAME ( ADDA  imm ) ; $8B BIND
:NONAME ( ADDA  dir ) ; $9B BIND
:NONAME ( ADDA  ext ) ; $BB BIND
:NONAME ( ADDA  ind ) ; $AB BIND

:NONAME ( ADDB  imm ) ; $CB BIND
:NONAME ( ADDB  dir ) ; $DB BIND
:NONAME ( ADDB  ext ) ; $FB BIND
:NONAME ( ADDB  ind ) ; $EB BIND

:NONAME ( ADDD  imm ) ; $C3 BIND
:NONAME ( ADDD  dir ) ; $D3 BIND
:NONAME ( ADDD  ext ) ; $F3 BIND
:NONAME ( ADDD  ind ) ; $E3 BIND

:NONAME ( ANDA  imm ) ; $84 BIND
:NONAME ( ANDA  dir ) ; $94 BIND
:NONAME ( ANDA  ext ) ; $B4 BIND
:NONAME ( ANDA  ind ) ; $A4 BIND

:NONAME ( ANDB  imm ) ; $C4 BIND
:NONAME ( ANDB  dir ) ; $D4 BIND
:NONAME ( ANDB  ext ) ; $F4 BIND
:NONAME ( ANDB  ind ) ; $E4 BIND

:NONAME ( ASL   dir ) ; $08 BIND
:NONAME ( ASL   ind ) ; $68 BIND
:NONAME ( ASL   ext ) ; $78 BIND

:NONAME ( ASLA  inh ) ; $48 BIND
:NONAME ( ASLB  inh ) ; $58 BIND

:NONAME ( ASR   dir ) ; $07 BIND
:NONAME ( ASR   ext ) ; $77 BIND
:NONAME ( ASR   ind ) ; $67 BIND

:NONAME ( ASRA  inh ) ; $47 BIND
:NONAME ( ASRB  inh ) ; $57 BIND

:NONAME ( BCC   rel ) ; $24 BIND
:NONAME ( BCS   rel ) ; $25 BIND
:NONAME ( BEQ   rel ) ; $27 BIND
:NONAME ( BGE   rel ) ; $2C BIND
:NONAME ( BGT   rel ) ; $2E BIND
:NONAME ( BHI   rel ) ; $22 BIND
:NONAME ( BHS   rel ) ; $24 BIND

:NONAME ( BITA  imm ) ; $85 BIND
:NONAME ( BITA  dir ) ; $95 BIND
:NONAME ( BITA  ext ) ; $B5 BIND
:NONAME ( BITA  ind ) ; $A5 BIND

:NONAME ( BITB  imm ) ; $C5 BIND
:NONAME ( BITB  dir ) ; $D5 BIND
:NONAME ( BITB  ext ) ; $F5 BIND
:NONAME ( BITB  ind ) ; $E5 BIND

:NONAME ( BLE   rel ) ; $2F BIND
:NONAME ( BLO   rel ) ; $25 BIND
:NONAME ( BLS   rel ) ; $23 BIND
:NONAME ( BLT   rel ) ; $2D BIND
:NONAME ( BMI   rel ) ; $2B BIND
:NONAME ( BNE   rel ) ; $26 BIND
:NONAME ( BPL   rel ) ; $2A BIND
:NONAME ( BRA   rel ) ; $20 BIND
:NONAME ( BRN   rel ) ; $21 BIND
:NONAME ( BSR   rel ) ; $8D BIND
:NONAME ( BVC   rel ) ; $28 BIND
:NONAME ( BVS   rel ) ; $29 BIND

:NONAME ( CMPA  imm ) ; $81 BIND
:NONAME ( CMPA  dir ) ; $91 BIND
:NONAME ( CMPA  ext ) ; $B1 BIND
:NONAME ( CMPA  ind ) ; $A1 BIND

:NONAME ( CMPB  imm ) ; $C1 BIND
:NONAME ( CMPB  dir ) ; $D1 BIND
:NONAME ( CMPB  ext ) ; $F1 BIND
:NONAME ( CMPB  ind ) ; $E1 BIND

:NONAME ( CMPD  imm ) ; $1083 BIND2
:NONAME ( CMPD  dir ) ; $1093 BIND2
:NONAME ( CMPD  ext ) ; $10B3 BIND2
:NONAME ( CMPD  ind ) ; $10A3 BIND2

:NONAME ( CMPS  imm ) ; $118C BIND2
:NONAME ( CMPS  dir ) ; $119C BIND2
:NONAME ( CMPS  ext ) ; $11BC BIND2
:NONAME ( CMPS  ind ) ; $11AC BIND2

:NONAME ( CMPU  imm ) ; $1183 BIND2
:NONAME ( CMPU  dir ) ; $1193 BIND2
:NONAME ( CMPU  ext ) ; $11B3 BIND2
:NONAME ( CMPU  ind ) ; $11A3 BIND2

:NONAME ( CMPX  imm ) ; $8C BIND
:NONAME ( CMPX  dir ) ; $9C BIND
:NONAME ( CMPX  ext ) ; $BC BIND
:NONAME ( CMPX  ind ) ; $AC BIND

:NONAME ( CMPY  imm ) ; $108C BIND2
:NONAME ( CMPY  dir ) ; $109C BIND2
:NONAME ( CMPY  ext ) ; $10BC BIND2
:NONAME ( CMPY  ind ) ; $10AC BIND2

:NONAME ( COM   dir ) ; $03 BIND
:NONAME ( COM   ext ) ; $73 BIND
:NONAME ( COM   ind ) ; $63 BIND

:NONAME ( COMA  inh ) ; $43 BIND
:NONAME ( COMB  inh ) ; $53 BIND

:NONAME ( CWAI  imm ) ; $3C BIND

:NONAME ( DAA   inh ) ; $19 BIND

:NONAME ( DEC   dir ) ; $0A BIND
:NONAME ( DEC   ext ) ; $7A BIND
:NONAME ( DEC   ind ) ; $6A BIND

:NONAME ( DECA  inh ) ; $4A BIND
:NONAME ( DECB  inh ) ; $5A BIND

:NONAME ( EORA  imm ) ; $88 BIND
:NONAME ( EORA  dir ) ; $98 BIND
:NONAME ( EORA  ext ) ; $B8 BIND
:NONAME ( EORA  ind ) ; $A8 BIND

:NONAME ( EORB  imm ) ; $C8 BIND
:NONAME ( EORB  dir ) ; $D8 BIND
:NONAME ( EORB  ext ) ; $F8 BIND
:NONAME ( EORB  ind ) ; $E8 BIND

:NONAME ( EXG   imm ) ; $1E BIND

:NONAME ( INC   dir ) ; $0C BIND
:NONAME ( INC   ext ) ; $7C BIND
:NONAME ( INC   ind ) ; $6C BIND

:NONAME ( INCA  inh ) ; $4C BIND
:NONAME ( INCB  inh ) ; $5C BIND

:NONAME ( JMP   dir ) ; $0E BIND
:NONAME ( JMP   ext ) ; $7E BIND
:NONAME ( JMP   ind ) ; $6E BIND

:NONAME ( JSR   dir ) ; $9D BIND
:NONAME ( JSR   ext ) ; $BD BIND
:NONAME ( JSR   ind ) ; $AD BIND

:NONAME ( LBCC  rel ) ; $1024 BIND2
:NONAME ( LBCS  rel ) ; $1025 BIND2
:NONAME ( LBEQ  rel ) ; $1027 BIND2
:NONAME ( LBGE  rel ) ; $102C BIND2
:NONAME ( LBGT  rel ) ; $102E BIND2
:NONAME ( LBHI  rel ) ; $1022 BIND2
:NONAME ( LBHS  rel ) ; $1024 BIND2
:NONAME ( LBLE  rel ) ; $102F BIND2
:NONAME ( LBLO  rel ) ; $1025 BIND2
:NONAME ( LBLS  rel ) ; $1023 BIND2
:NONAME ( LBLT  rel ) ; $102D BIND2
:NONAME ( LBMI  rel ) ; $102B BIND2
:NONAME ( LBNE  rel ) ; $1026 BIND2
:NONAME ( LBPL  rel ) ; $102A BIND2
:NONAME ( LBRA  rel ) ; $16 BIND
:NONAME ( LBRN  rel ) ; $1021 BIND2
:NONAME ( LBSR  rel ) ; $17 BIND
:NONAME ( LBVC  rel ) ; $1028 BIND2
:NONAME ( LBVS  rel ) ; $1029 BIND2

: LDA ( b -- ) >N >Z 'V CLEAR _A C! ;
:NONAME ( LDA   imm ) BYTE@      LDA ; $86 BIND
:NONAME ( LDA   dir ) 'DP    TC@ LDA ; $96 BIND
:NONAME ( LDA   ext ) 'EA    TC@ LDA ; $B6 BIND
:NONAME ( LDA   ind ) 'IND   TC@ LDA ; $A6 BIND

: LDB ( b -- ) >N >Z 'V CLEAR _B C! ;
:NONAME ( LDB   imm ) BYTE@      LDB ; $C6 BIND
:NONAME ( LDB   dir ) 'DP    TC@ LDB ; $D6 BIND
:NONAME ( LDB   ext ) 'EA    TC@ LDB ; $F6 BIND
:NONAME ( LDB   ind ) 'IND   TC@ LDB ; $E6 BIND

: LDD ( w -- ) >NW >Z 'V CLEAR _D W! ;
:NONAME ( LDD   imm ) WORD@      LDD ; $CC BIND
:NONAME ( LDD   dir ) 'DP    TW@ LDD ; $DC BIND
:NONAME ( LDD   ext ) 'EA    TW@ LDD ; $FC BIND
:NONAME ( LDD   ind ) 'IND   TW@ LDD ; $EC BIND

: LDS ( w -- ) >NW >Z 'V CLEAR _S W! ;
:NONAME ( LDS   imm ) WORD@      LDS ; $10CE BIND2
:NONAME ( LDS   dir ) 'DP    TW@ LDS ; $10DE BIND2
:NONAME ( LDS   ext ) 'EA    TW@ LDS ; $10FE BIND2
:NONAME ( LDS   ind ) 'IND   TW@ LDS ; $10EE BIND2

: LDU ( w -- ) >NW >Z 'V CLEAR _U W! ;
:NONAME ( LDU   imm ) WORD@      LDU ; $CE BIND
:NONAME ( LDU   dir ) 'DP    TW@ LDU ; $DE BIND
:NONAME ( LDU   ext ) 'EA    TW@ LDU ; $FE BIND
:NONAME ( LDU   ind ) 'IND   TW@ LDU ; $EE BIND

: LDX ( w -- ) >NW >Z 'V CLEAR _X W! ;
:NONAME ( LDX   imm ) WORD@      LDX ; $8E BIND
:NONAME ( LDX   dir ) 'DP    TW@ LDX ; $9E BIND
:NONAME ( LDX   ext ) 'EA    TW@ LDX ; $BE BIND
:NONAME ( LDX   ind ) 'IND   TW@ LDX ; $AE BIND

: LDY ( w -- ) >NW >Z 'V CLEAR _Y W! ;
:NONAME ( LDY   imm ) WORD@      LDY ; $108E BIND2
:NONAME ( LDY   dir ) 'DP    TW@ LDY ; $109E BIND2
:NONAME ( LDY   ext ) 'EA    TW@ LDY ; $10BE BIND2
:NONAME ( LDY   ind ) 'IND   TW@ LDY ; $10AE BIND2

:NONAME ( CLRA  inh ) 0 LDA 'C CLEAR ; $4F BIND
:NONAME ( CLRB  inh ) 0 LDB 'C CLEAR ; $5F BIND

:NONAME ( CLR   dir ) ; $0F BIND
:NONAME ( CLR   ext ) ; $7F BIND
:NONAME ( CLR   ind ) ; $6F BIND

:NONAME ( LEAS  ind ) 'IND    _S W! ; $32 BIND
:NONAME ( LEAU  ind ) 'IND    _U W! ; $33 BIND
:NONAME ( LEAX  ind ) 'IND >Z _X W! ; $30 BIND
:NONAME ( LEAY  ind ) 'IND >Z _Y W! ; $31 BIND

:NONAME ( LSL   dir ) ; $08 BIND
:NONAME ( LSL   ind ) ; $68 BIND
:NONAME ( LSL   ext ) ; $78 BIND

:NONAME ( LSLA  inh ) ; $48 BIND
:NONAME ( LSLB  inh ) ; $58 BIND

:NONAME ( LSR   dir ) ; $04 BIND
:NONAME ( LSR   ext ) ; $74 BIND
:NONAME ( LSR   ind ) ; $64 BIND

:NONAME ( LSRA  inh ) ; $44 BIND
:NONAME ( LSRB  inh ) ; $54 BIND

:NONAME ( MUL   inh ) ; $3D BIND

:NONAME ( NEG   ext ) ; $70 BIND
:NONAME ( NEG   ind ) ; $60 BIND

:NONAME ( NEGA  inh ) _A C@ NEG $FF AND LDA ; $40 BIND
:NONAME ( NEGB  inh ) _B C@ NEG $FF AND LDB ; $50 BIND

:NONAME ( NOP   inh ) ; $12 BIND

:NONAME ( ORA   imm ) ; $8A BIND
:NONAME ( ORA   dir ) ; $9A BIND
:NONAME ( ORA   ext ) ; $BA BIND
:NONAME ( ORA   ind ) ; $AA BIND

:NONAME ( ORB   imm ) ; $CA BIND
:NONAME ( ORB   dir ) ; $DA BIND
:NONAME ( ORB   ext ) ; $FA BIND
:NONAME ( ORB   ind ) ; $EA BIND

:NONAME ( PSHS  imm ) ; $34 BIND
:NONAME ( PSHU  imm ) ; $36 BIND
:NONAME ( PULS  imm ) ; $35 BIND
:NONAME ( PULU  imm ) ; $37 BIND
:NONAME ( ROL   dir ) ; $09 BIND
:NONAME ( ROL   ind ) ; $69 BIND
:NONAME ( ROL   ext ) ; $79 BIND
:NONAME ( ROLA  inh ) ; $49 BIND
:NONAME ( ROLB  inh ) ; $59 BIND
:NONAME ( ROR   dir ) ; $06 BIND
:NONAME ( ROR   ind ) ; $66 BIND
:NONAME ( ROR   ext ) ; $76 BIND
:NONAME ( RORA  inh ) ; $46 BIND
:NONAME ( RORB  inh ) ; $56 BIND
:NONAME ( RTI   inh ) ; $3B BIND
:NONAME ( RTS   inh ) ; $39 BIND
:NONAME ( SBCA  imm ) ; $82 BIND
:NONAME ( SBCA  dir ) ; $92 BIND
:NONAME ( SBCA  ind ) ; $A2 BIND
:NONAME ( SBCA  ext ) ; $B2 BIND
:NONAME ( SBCB  imm ) ; $C2 BIND
:NONAME ( SBCB  dir ) ; $D2 BIND
:NONAME ( SBCB  ind ) ; $E2 BIND
:NONAME ( SBCB  ext ) ; $F2 BIND
:NONAME ( SEX   inh ) ; $1D BIND

: STA ( addr -- ) _A C@ SWAP TC! ;
:NONAME ( STA   dir ) 'DP        STA ; $97 BIND
:NONAME ( STA   ext ) 'EA        STA ; $B7 BIND
:NONAME ( STA   ind ) 'IND       STA ; $A7 BIND

: STB ( addr -- ) _B C@ SWAP TC! ;
:NONAME ( STB   dir ) 'DP        STB ; $D7 BIND
:NONAME ( STB   ext ) 'EA        STB ; $F7 BIND
:NONAME ( STB   ind ) 'IND       STB ; $E7 BIND

: STD ( addr -- ) _D W@ SWAP TW! ;
:NONAME ( STD   dir ) 'DP        STD ; $DD BIND
:NONAME ( STD   ext ) 'EA        STD ; $FD BIND
:NONAME ( STD   ind ) 'IND       STD ; $ED BIND

: STS ( addr -- ) _S W@ SWAP TW! ;
:NONAME ( STS   dir ) 'DP        STS ; $10DF BIND2
:NONAME ( STS   ext ) 'EA        STS ; $10FF BIND2
:NONAME ( STS   ind ) 'IND       STS ; $10EF BIND2

: STU ( addr -- ) _U W@ SWAP TW! ;
:NONAME ( STU   dir ) 'DP        STU ; $DF BIND
:NONAME ( STU   ext ) 'EA        STU ; $FF BIND
:NONAME ( STU   ind ) 'IND       STU ; $EF BIND

: STX ( addr -- ) _X W@ SWAP TW! ;
:NONAME ( STX   dir ) 'DP        STX ; $9F BIND
:NONAME ( STX   ext ) 'EA        STX ; $BF BIND
:NONAME ( STX   ind ) 'IND       STX ; $AF BIND

: STY ( addr -- ) _Y W@ SWAP TW! ;
:NONAME ( STY   dir ) 'DP        STY ; $109F BIND2
:NONAME ( STY   ext ) 'EA        STY ; $10BF BIND2
:NONAME ( STY   ind ) 'IND       STY ; $10AF BIND2

:NONAME ( SUBA  imm ) ; $80 BIND
:NONAME ( SUBA  dir ) ; $90 BIND
:NONAME ( SUBA  ext ) ; $B0 BIND
:NONAME ( SUBA  ind ) ; $A0 BIND

:NONAME ( SUBB  imm ) ; $C0 BIND
:NONAME ( SUBB  dir ) ; $D0 BIND
:NONAME ( SUBB  ext ) ; $F0 BIND
:NONAME ( SUBB  ind ) ; $E0 BIND

:NONAME ( SUBD  imm ) ; $83 BIND
:NONAME ( SUBD  dir ) ; $93 BIND
:NONAME ( SUBD  ext ) ; $B3 BIND
:NONAME ( SUBD  ind ) ; $A3 BIND

:NONAME ( SWI   inh ) ; $3F BIND
:NONAME ( SWI2  inh ) ; $103F BIND2
:NONAME ( SWI3  inh ) ; $113F BIND2
:NONAME ( SYNC  inh ) ; $13 BIND

:NONAME ( TFR   imm ) ; $1F BIND

:NONAME ( TST   dir ) ; $0D BIND
:NONAME ( TST   ext ) ; $7D BIND
:NONAME ( TST   ind ) ; $6D BIND

:NONAME ( TSTA  inh ) ; $4D BIND
:NONAME ( TSTB  inh ) ; $5D BIND

\ -- store a minimal program

: ORG   DUP TO THERE _PC! ;
: _     TC, ;

$4000 ORG