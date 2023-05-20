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

: GETC BEGIN KEY? UNTIL KEY ;   \ blocking
\ : GETC KEY? IF KEY EXIT THEN 0 ;  \ non-blocking
: 2+ 1+ 1+ ;
: EXEC EXECUTE ;
: NEG 0 SWAP - ;
: NOT NEG 1 - ;
: CLS CLEARSTACK ;
: W, ( word -- ) $100 /MOD SWAP C, C, ; \ word comma
: TF ( n -- f ) 0= 0= ; \ return TRUE (-1) or FALSE (0)
: RSWAP R> R> R> SWAP >R >R >R ;

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

\ fake registers used for invalid nibbles in TFR, EXG
CREATE _F $FFFF W,
CREATE _Z 0     C,

\ Array _RR stores adresses of X, Y, U and S registers
\ It'll be used in Index Addressing Modes
CREATE _RR _X , _Y , _U , _S ,

\ Array to store Address registers to decode Transfer Register to Register mode
CREATE REGS ( "undefined" also filled with valid registers...)
_D , _X , _Y  , _U  , _S , _PC , _F , _F , \ 16-bit registers
_A , _B , _CC , _DP , _F , _F  , _F , _F , \  8-bit registers

: LDCC  ( byte -- ) _CC C! ;

CREATE OPCODES  $100 CELLS ALLOT
CREATE OPCODES2 $200 CELLS ALLOT \ array for $10xx-$11-- extended opcodes
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

: TW? TW@ W. ;

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

CREATE CCSTR $10 ALLOT
s" efhinzvcEFHINZVC" DROP CCSTR $10 CMOVE

: .CC _CC C@ 8 0 DO DUP $80 AND IF 8 ELSE 0 THEN I + CCSTR + @ EMIT 2* LOOP DROP ;

: STATUS
  CR ." A  B X    Y    U    S    DP " .CC SPACE ." CC PC"
  CR _D W? _X W? _Y W? _U W? _S W? _DP C? _CC C@ DUP BIN. SPACE C. _PC W? ." > " DUMPPC
  CR 5 SPACES _X W@    TW? _Y W@    TW? _U W@    TW? _S W@    TW?
  CR 5 SPACES _X W@ 2+ TW? _Y W@ 2+ TW? _U W@ 2+ TW? _S W@ 2+ TW?
  ;

-1 VALUE LASTINSTR

0 VALUE LASTPC

: FETCH ( -- opcode )
  BYTE@
  DUP  10 =
  OVER 11 =
  OR IF 8 LSHIFT BYTE@ OR THEN
;

: DECODE
  DUP $FF00 AND IF
    $FFF AND
    CELLS OPCODES2
  ELSE
    CELLS OPCODES
  THEN
  + @
;

: NEXT
  ( Save PC ) _PC W@ TO LASTPC
  ( FETCH   ) FETCH DUP TO LASTINSTR
  ( DECODE  ) DECODE
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

: BIND  ( xt opcode -- )          CELLS OPCODES  + ! ; \ saves XT in OPCODES table
: BIND2 ( xt opcode -- ) $FFF AND CELLS OPCODES2 + ! ; \ special case of 2 bytes opcodes $10xx & $11xx

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

: >N   ( b -- b ) DUP   $80 AND    'N UPDATE-FLAG ; \ non-droppy, byte version
: >Z   ( b -- b ) DUP   $FF AND 0= 'Z UPDATE-FLAG ; \ non-droppy, byte version
: >NZ  >N  >Z ;

: >NW  ( w -- w ) DUP $8000 AND    'N UPDATE-FLAG ; \ non-droppy, word version
: >ZW  ( w -- w ) DUP $FFFF AND 0= 'Z UPDATE-FLAG ; \ non-droppy, word version
: >NZW >NW >ZW ;

: >H   ( f --   )                  'H UPDATE-FLAG ; \ this one is droppy
: >V   ( f --   )                  'V UPDATE-FLAG ; \ this one is droppy
: >C   ( f --   )                  'C UPDATE-FLAG ; \ this one is droppy

: H>   (   -- f ) _CC C@ 'H AND TF NEG ; \ return 0 / 1
: N>   (   -- f ) _CC C@ 'N AND TF NEG ; \ return 0 / 1
: Z>   (   -- f ) _CC C@ 'Z AND TF NEG ; \ return 0 / 1
: V>   (   -- f ) _CC C@ 'V AND TF NEG ; \ return 0 / 1
: C>   (   -- f ) _CC C@ 'C AND TF NEG ; \ return 0 / 1

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
      ( ,R+          ) %0000 OF DUP      W@ DUP  1+ ROT W!         ENDOF
      ( ,R++         ) %0001 OF DUP      W@ DUP  2+ ROT W!  R@ ?[] ENDOF
      ( ,-R          ) %0010 OF DUP      W@ 1-  DUP ROT W!         ENDOF
      ( ,--R         ) %0011 OF DUP      W@ 2 - DUP ROT W!  R@ ?[] ENDOF
      ( ,R           ) %0100 OF          W@                 R@ ?[] ENDOF
      ( A,R          ) %0110 OF          W@ _A C@ SIGNEX8 + R@ ?[] ENDOF
      ( B,R          ) %0101 OF          W@ _B C@ SIGNEX8 + R@ ?[] ENDOF
      ( ,R +  8b off ) %1000 OF          W@ BYTE@ SIGNEX8 + R@ ?[] ENDOF
      ( ,R + 16b off ) %1001 OF          W@ WORD@         + R@ ?[] ENDOF
      ( D,R          ) %1011 OF          W@ _D W@         + R@ ?[] ENDOF
      ( ,PC + 8b off ) %1100 OF DROP BYTE@ SIGNEX8 _PC W@ + R@ ?[] ENDOF
      ( ,PC +16b off ) %1101 OF DROP WORD@         _PC W@ + R@ ?[] ENDOF
      ( [,Addr]      ) %1111 OF DROP WORD@                  TW@    ENDOF
    ENDCASE
  THEN
  R> DROP
  $FFFF AND \ trim to 16 bit
;
\ Opcodes definitions

: LDA ( b -- ) >NZ 'V CLEAR _A C! ;
:NONAME ( LDA   imm ) BYTE@      LDA ; $86 BIND
:NONAME ( LDA   dir ) 'DP    TC@ LDA ; $96 BIND
:NONAME ( LDA   ext ) 'EA    TC@ LDA ; $B6 BIND
:NONAME ( LDA   ind ) 'IND   TC@ LDA ; $A6 BIND

: LDB ( b -- ) >NZ 'V CLEAR _B C! ;
:NONAME ( LDB   imm ) BYTE@      LDB ; $C6 BIND
:NONAME ( LDB   dir ) 'DP    TC@ LDB ; $D6 BIND
:NONAME ( LDB   ext ) 'EA    TC@ LDB ; $F6 BIND
:NONAME ( LDB   ind ) 'IND   TC@ LDB ; $E6 BIND

: LDD ( w -- ) >NW >ZW 'V CLEAR _D W! ;
:NONAME ( LDD   imm ) WORD@      LDD ; $CC BIND
:NONAME ( LDD   dir ) 'DP    TW@ LDD ; $DC BIND
:NONAME ( LDD   ext ) 'EA    TW@ LDD ; $FC BIND
:NONAME ( LDD   ind ) 'IND   TW@ LDD ; $EC BIND

: LDS ( w -- ) >NW >ZW 'V CLEAR _S W! ;
:NONAME ( LDS   imm ) WORD@      LDS ; $10CE BIND2
:NONAME ( LDS   dir ) 'DP    TW@ LDS ; $10DE BIND2
:NONAME ( LDS   ext ) 'EA    TW@ LDS ; $10FE BIND2
:NONAME ( LDS   ind ) 'IND   TW@ LDS ; $10EE BIND2

: LDU ( w -- ) >NW >ZW 'V CLEAR _U W! ;
:NONAME ( LDU   imm ) WORD@      LDU ; $CE BIND
:NONAME ( LDU   dir ) 'DP    TW@ LDU ; $DE BIND
:NONAME ( LDU   ext ) 'EA    TW@ LDU ; $FE BIND
:NONAME ( LDU   ind ) 'IND   TW@ LDU ; $EE BIND

: LDX ( w -- ) >NW >ZW 'V CLEAR _X W! ;
:NONAME ( LDX   imm ) WORD@      LDX ; $8E BIND
:NONAME ( LDX   dir ) 'DP    TW@ LDX ; $9E BIND
:NONAME ( LDX   ext ) 'EA    TW@ LDX ; $BE BIND
:NONAME ( LDX   ind ) 'IND   TW@ LDX ; $AE BIND

: LDY ( w -- ) >NW >ZW 'V CLEAR _Y W! ;
:NONAME ( LDY   imm ) WORD@      LDY ; $108E BIND2
:NONAME ( LDY   dir ) 'DP    TW@ LDY ; $109E BIND2
:NONAME ( LDY   ext ) 'EA    TW@ LDY ; $10BE BIND2
:NONAME ( LDY   ind ) 'IND   TW@ LDY ; $10AE BIND2

:NONAME ( ABX   inh ) _X W@ _B C@ + _X W! ; $3A BIND

: ANDCC ( byte ) _CC C@ AND _CC C! ;
: ORCC  ( byte ) _CC C@ OR  _CC C! ;
:NONAME ( ANDCC imm ) BYTE@ ANDCC  ; $1C BIND
:NONAME ( ORCC  imm ) BYTE@ ORCC   ; $1A BIND

: ADD>H ( b b c -- b b c   ) >R OVER $0F AND OVER $0F AND + R@ + $10 AND >H R>      ; \ sets H for addition
: ADD?V ( b b c -- b b c f ) >R OVER $7F AND OVER $7F AND + R@ + $80 AND TF R> SWAP ; \ returns flag used to set V

: ADD ( b b -- b ) \ sets flags
  ADD>H
  ADD?V >R
  + + DUP
  $100 AND TF DUP >C
  R> = 0= >V
  >NZ
;

: ADCA ( b c -- )  _A C@ SWAP ADD _A C! ;
:NONAME ( ADDA  imm ) BYTE@      0  ADCA ; $8B BIND
:NONAME ( ADDA  dir ) 'DP    TC@ 0  ADCA ; $9B BIND
:NONAME ( ADDA  ext ) 'EA    TC@ 0  ADCA ; $BB BIND
:NONAME ( ADDA  ind ) 'IND   TC@ 0  ADCA ; $AB BIND

:NONAME ( ADCA  imm ) BYTE@      C> ADCA ; $89 BIND
:NONAME ( ADCA  dir ) 'DP    TC@ C> ADCA ; $99 BIND
:NONAME ( ADCA  ext ) 'EA    TC@ C> ADCA ; $B9 BIND
:NONAME ( ADCA  ind ) 'IND   TC@ C> ADCA ; $A9 BIND

: ADCB ( b c -- )  _B C@ SWAP ADD _B C! ;
:NONAME ( ADDB  imm ) BYTE@      0  ADCB ; $CB BIND
:NONAME ( ADDB  dir ) 'DP    TC@ 0  ADCB ; $DB BIND
:NONAME ( ADDB  ext ) 'EA    TC@ 0  ADCB ; $FB BIND
:NONAME ( ADDB  ind ) 'IND   TC@ 0  ADCB ; $EB BIND

:NONAME ( ADCB  imm ) BYTE@      C> ADCB ; $C9 BIND
:NONAME ( ADCB  dir ) 'DP    TC@ C> ADCB ; $D9 BIND
:NONAME ( ADCB  ext ) 'EA    TC@ C> ADCB ; $F9 BIND
:NONAME ( ADCB  ind ) 'IND   TC@ C> ADCB ; $E9 BIND

: ?VW  ( w w -- f ) OVER $7FFF AND OVER $7FFF AND + $8000 AND TF ; \ returns flag used to set V

: ADD16 ( w w -- w ) \ sets flags
  ?VW >R
  + DUP
  $10000 AND TF DUP >C
  R> = 0= >V
  >NW >ZW
;

: ADDD  ( w -- )  _D W@ ADD16 _D W! ;
:NONAME ( ADDD  imm ) WORD@      ADDD ; $C3 BIND
:NONAME ( ADDD  dir ) 'DP    TW@ ADDD ; $D3 BIND
:NONAME ( ADDD  ext ) 'EA    TW@ ADDD ; $F3 BIND
:NONAME ( ADDD  ind ) 'IND   TW@ ADDD ; $E3 BIND

: ANDA  ( b -- ) _A C@ AND LDA ;
:NONAME ( ANDA  imm ) BYTE@      ANDA ; $84 BIND
:NONAME ( ANDA  dir ) 'DP    TC@ ANDA ; $94 BIND
:NONAME ( ANDA  ext ) 'EA    TC@ ANDA ; $B4 BIND
:NONAME ( ANDA  ind ) 'IND   TC@ ANDA ; $A4 BIND

: ANDB  ( b -- ) _B C@ AND LDB ;
:NONAME ( ANDB  imm ) BYTE@      ANDB ; $C4 BIND
:NONAME ( ANDB  dir ) 'DP    TC@ ANDB ; $D4 BIND
:NONAME ( ANDB  ext ) 'EA    TC@ ANDB ; $F4 BIND
:NONAME ( ANDB  ind ) 'IND   TC@ ANDB ; $E4 BIND

: EORA  ( b -- ) _A C@ XOR LDA ;
:NONAME ( EORA  imm ) BYTE@     EORA ; $88 BIND
:NONAME ( EORA  dir ) 'DP   TC@ EORA ; $98 BIND
:NONAME ( EORA  ext ) 'EA   TC@ EORA ; $B8 BIND
:NONAME ( EORA  ind ) 'IND  TC@ EORA ; $A8 BIND

: EORB  ( b -- ) _B C@ XOR LDB ;
:NONAME ( EORB  imm ) BYTE@     EORB ; $C8 BIND
:NONAME ( EORB  dir ) 'DP   TC@ EORB ; $D8 BIND
:NONAME ( EORB  ext ) 'EA   TC@ EORB ; $F8 BIND
:NONAME ( EORB  ind ) 'IND  TC@ EORB ; $E8 BIND

: ORA   ( b -- ) _A C@ OR LDA ;
:NONAME ( ORA   imm ) BYTE@     ORA ; $8A BIND
:NONAME ( ORA   dir ) 'DP   TC@ ORA ; $9A BIND
:NONAME ( ORA   ext ) 'EA   TC@ ORA ; $BA BIND
:NONAME ( ORA   ind ) 'IND  TC@ ORA ; $AA BIND

: ORB ( b -- ) _B C@ OR LDB ;
:NONAME ( ORB   imm ) BYTE@     ORB ; $CA BIND
:NONAME ( ORB   dir ) 'DP   TC@ ORB ; $DA BIND
:NONAME ( ORB   ext ) 'EA   TC@ ORB ; $FA BIND
:NONAME ( ORB   ind ) 'IND  TC@ ORB ; $EA BIND

: BITA ( addr reg -- ) _A C@ AND >NZ 'V CLEAR ;
:NONAME ( BITA  imm ) BYTE@     BITA ; $85 BIND
:NONAME ( BITA  dir ) 'DP   TC@ BITA ; $95 BIND
:NONAME ( BITA  ext ) 'EA   TC@ BITA ; $B5 BIND
:NONAME ( BITA  ind ) 'IND  TC@ BITA ; $A5 BIND

: BITB ( addr reg -- ) _B C@ AND >NZ 'V CLEAR ;
:NONAME ( BITB  imm ) BYTE@     BITB ; $C5 BIND
:NONAME ( BITB  dir ) 'DP   TC@ BITB ; $D5 BIND
:NONAME ( BITB  ext ) 'EA   TC@ BITB ; $F5 BIND
:NONAME ( BITB  ind ) 'IND  TC@ BITB ; $E5 BIND

: ASL>V ( byte -- byte ) DUP  $80 AND TF OVER $40 AND TF XOR >V ; \ set V for ASL, non-droppy
: ASL>C ( byte -- byte ) DUP $100 AND >C ; \ set C for ASL, non-droppy

: ASL ( addr -- ) DUP C@ ASL>V 2* ASL>C >NZ SWAP C! ;
:NONAME ( ASLA  inh ) _A          ASL ; $48 BIND
:NONAME ( ASLB  inh ) _B          ASL ; $58 BIND
:NONAME ( ASL   dir ) 'DP   RAM + ASL ; $08 BIND
:NONAME ( ASL   ext ) 'EA   RAM + ASL ; $78 BIND
:NONAME ( ASL   ind ) 'IND  RAM + ASL ; $68 BIND

: ASR>C DUP 1 AND >C ;

: ASR ( addr -- ) DUP C@ ASR>C DUP $80 AND SWAP 2/ OR >NZ SWAP C! ;
:NONAME ( ASRA  inh ) _A          ASR ; $47 BIND
:NONAME ( ASRB  inh ) _B          ASR ; $57 BIND
:NONAME ( ASR   dir ) 'DP   RAM + ASR ; $07 BIND
:NONAME ( ASR   ext ) 'EA   RAM + ASR ; $77 BIND
:NONAME ( ASR   ind ) 'IND  RAM + ASR ; $67 BIND

: LSR ( addr -- ) DUP C@ ASR>C 2/ >NZ SWAP C! ;
:NONAME ( LSRA  inh ) _A          LSR ; $44 BIND
:NONAME ( LSRB  inh ) _B          LSR ; $54 BIND
:NONAME ( LSR   dir ) 'DP   RAM + LSR ; $04 BIND
:NONAME ( LSR   ext ) 'EA   RAM + LSR ; $74 BIND
:NONAME ( LSR   ind ) 'IND  RAM + LSR ; $64 BIND

: ROL ( addr -- ) DUP C@ 2* C> IF 1 ELSE 0 THEN OR ASL>C >NZ SWAP C! ;
:NONAME ( ROLA  inh ) _A          ROL ; $49 BIND
:NONAME ( ROLB  inh ) _B          ROL ; $59 BIND
:NONAME ( ROL   dir ) 'DP   RAM + ROL ; $09 BIND
:NONAME ( ROL   ext ) 'EA   RAM + ROL ; $79 BIND
:NONAME ( ROL   ind ) 'IND  RAM + ROL ; $69 BIND

: ROR ( addr -- ) DUP C@ C> IF $80 ELSE 0 THEN SWAP ASR>C 2/ OR >NZ SWAP C! ;
:NONAME ( RORA  inh ) _A          ROR ; $46 BIND
:NONAME ( RORB  inh ) _B          ROR ; $56 BIND
:NONAME ( ROR   dir ) 'DP   RAM + ROR ; $06 BIND
:NONAME ( ROR   ext ) 'EA   RAM + ROR ; $76 BIND
:NONAME ( ROR   ind ) 'IND  RAM + ROR ; $66 BIND

: ?BRA ( f -- ) BYTE@ SWAP IF SIGNEX8 _PC W@ + _PC! ELSE DROP THEN ; \ branch if flag is set
:NONAME ( BRA   rel ) 1                    ?BRA ; $20 BIND \ Always
:NONAME ( BRN   rel ) 0                    ?BRA ; $21 BIND \ Never
:NONAME ( BHI   rel ) Z> 0=      C> 0= AND ?BRA ; $22 BIND \ Z and C clear
:NONAME ( BLS   rel ) Z>         C>    OR  ?BRA ; $23 BIND \ Z or  C set
:NONAME ( BCC   rel ) C> 0=                ?BRA ; $24 BIND \ C clear
:NONAME ( BCS   rel ) C>                   ?BRA ; $25 BIND \ C set
:NONAME ( BNE   rel ) Z> 0=                ?BRA ; $26 BIND \ Z clear
:NONAME ( BEQ   rel ) Z>                   ?BRA ; $27 BIND \ Z set
:NONAME ( BVC   rel ) V> 0=                ?BRA ; $28 BIND \ V clear
:NONAME ( BVS   rel ) V>                   ?BRA ; $29 BIND \ V set
:NONAME ( BPL   rel ) N> 0=                ?BRA ; $2A BIND \ N clear
:NONAME ( BMI   rel ) N>                   ?BRA ; $2B BIND \ N set
:NONAME ( BGE   rel ) V> N> =    Z>    OR  ?BRA ; $2C BIND \ V==N or  Z set
:NONAME ( BGT   rel ) V> N> =    Z> 0= AND ?BRA ; $2E BIND \ V==N and Z clear
:NONAME ( BLE   rel ) V> N> = 0= Z>    OR  ?BRA ; $2F BIND \ V!=N or  Z set
:NONAME ( BLT   rel ) V> N> = 0= Z> 0= AND ?BRA ; $2D BIND \ V!=N and Z clear

: CLR ( addr -- ) 0 SWAP C! $F0 ANDCC 'Z SET ;
:NONAME ( CLRA  inh ) _A         CLR ; $4F BIND
:NONAME ( CLRB  inh ) _B         CLR ; $5F BIND
:NONAME ( CLR   dir ) 'DP  RAM + CLR ; $0F BIND
:NONAME ( CLR   ext ) 'EA  RAM + CLR ; $7F BIND
:NONAME ( CLR   ind ) 'IND RAM + CLR ; $6F BIND

: COM ( addr -- ) DUP C@ NOT >NZ 'V CLEAR 'C SET SWAP C! ;
:NONAME ( COMA  inh ) _A          COM ; $43 BIND
:NONAME ( COMB  inh ) _B          COM ; $53 BIND
:NONAME ( COM   dir ) 'DP   RAM + COM ; $03 BIND
:NONAME ( COM   ext ) 'EA   RAM + COM ; $73 BIND
:NONAME ( COM   ind ) 'IND  RAM + COM ; $63 BIND

:NONAME ( CWAI  imm ) ; $3C BIND

\ Decimal Addition Adjust
\ Logic described in The MC6809 Cookbook
:NONAME ( DAA   inh )
  C> >R
  _A C@ $0F AND DUP >R
  9 > H> OR IF $06 ELSE 0 THEN
  C> _A C@ $F0 AND 4 RSHIFT DUP >R 9 > OR
  R> 8 > R> 9 > AND OR IF $60 ELSE 0 THEN
  OR _A C@ + >NZ DUP $100 AND R> OR >C _A C!
; $19 BIND

: DEC ( addr -- ) DUP C@ DUP $80 = IF 'V SET ELSE 'V CLEAR THEN 1- >NZ SWAP  C! ;
:NONAME ( DECA  inh ) _A          DEC ; $4A BIND
:NONAME ( DECB  inh ) _B          DEC ; $5A BIND
:NONAME ( DEC   dir ) 'DP   RAM + DEC ; $0A BIND
:NONAME ( DEC   ext ) 'EA   RAM + DEC ; $7A BIND
:NONAME ( DEC   ind ) 'IND  RAM + DEC ; $6A BIND

: INC ( addr -- ) DUP C@ DUP $7F = IF 'V SET ELSE 'V CLEAR THEN 1+ >NZ SWAP  C! ;
:NONAME ( INCA  inh ) _A          INC ; $4C BIND
:NONAME ( INCB  inh ) _B          INC ; $5C BIND
:NONAME ( INC   dir ) 'DP   RAM + INC ; $0C BIND
:NONAME ( INC   ext ) 'EA   RAM + INC ; $7C BIND
:NONAME ( INC   ind ) 'IND  RAM + INC ; $6C BIND

: TST ( addr -- ) C@ >NZ 'V CLEAR ;
:NONAME ( TSTA  inh ) _A          TST ; $4D BIND
:NONAME ( TSTB  inh ) _B          TST ; $5D BIND
:NONAME ( TST   dir ) 'DP   RAM + TST ; $0D BIND
:NONAME ( TST   ext ) 'EA   RAM + TST ; $7D BIND
:NONAME ( TST   ind ) 'IND  RAM + TST ; $6D BIND

:NONAME ( JMP   dir ) 'DP  _PC! ; $0E BIND
:NONAME ( JMP   ext ) 'EA  _PC! ; $7E BIND
:NONAME ( JMP   ind ) 'IND _PC! ; $6E BIND

: ?LBR ( f -- ) WORD@ SWAP IF _PC W@ + _PC! ELSE DROP THEN ; \ long branch if flag is set
:NONAME ( LBRA  rel ) 1                    ?LBR ; $16   BIND  \ Always
:NONAME ( LBRN  rel ) 0                    ?LBR ; $1021 BIND2 \ Never
:NONAME ( LBHI  rel ) Z> 0= C> 0= AND      ?LBR ; $1022 BIND2 \ Z and C clear
:NONAME ( LBLS  rel ) Z>    C>    OR       ?LBR ; $1023 BIND2 \ Z or  C set
:NONAME ( LBCC  rel ) C> 0=                ?LBR ; $1024 BIND2 \ C clear
:NONAME ( LBCS  rel ) C>                   ?LBR ; $1025 BIND2 \ C set
:NONAME ( LBNE  rel ) Z> 0=                ?LBR ; $1026 BIND2 \ Z clear
:NONAME ( LBEQ  rel ) Z>                   ?LBR ; $1027 BIND2 \ Z set
:NONAME ( LBVC  rel ) V> 0=                ?LBR ; $1028 BIND2 \ V clear
:NONAME ( LBVS  rel ) V>                   ?LBR ; $1029 BIND2 \ V set
:NONAME ( LBPL  rel ) N> 0=                ?LBR ; $102A BIND2 \ N clear
:NONAME ( LBMI  rel ) N>                   ?LBR ; $102B BIND2 \ N set
:NONAME ( LBGE  rel ) V> N> =    Z>    OR  ?LBR ; $102C BIND2 \ V==N or  Z set
:NONAME ( LBGT  rel ) V> N> =    Z> 0= AND ?LBR ; $102E BIND2 \ V==N and Z clear
:NONAME ( LBLE  rel ) V> N> = 0= Z>    OR  ?LBR ; $102F BIND2 \ V!=N or  Z set
:NONAME ( LBLT  rel ) V> N> = 0= Z> 0= AND ?LBR ; $102D BIND2 \ V!=N and Z clear

:NONAME ( LEAS  ind ) 'IND     _S W! ; $32 BIND
:NONAME ( LEAU  ind ) 'IND     _U W! ; $33 BIND
:NONAME ( LEAX  ind ) 'IND >ZW _X W! ; $30 BIND
:NONAME ( LEAY  ind ) 'IND >ZW _Y W! ; $31 BIND

:NONAME ( MUL   inh ) _A C@ _B C@ * >ZW DUP $80 AND >C _D W! ; $3D BIND

: MEMNEG ( addr -- )
  DUP C@
  DUP $80 = IF 'V SET ELSE 'V CLEAR THEN
  DUP >C
  SIGNEX8 NEG
  >NZ
  SWAP C!
;
:NONAME ( NEGA  inh ) _A          MEMNEG ; $40 BIND
:NONAME ( NEGB  inh ) _B          MEMNEG ; $50 BIND
:NONAME ( NEG   ext ) 'EA   RAM + MEMNEG ; $70 BIND
:NONAME ( NEG   ind ) 'IND  RAM + MEMNEG ; $60 BIND

:NONAME ( NOP   inh ) ; $12 BIND

: PUSH ( addr -- value ) DUP W@ 1- DUP ROT W! TC! ; \ pre-decrement register R (S/U), leave value on ToS, save to

\ Push multiple regs (as indicated by postbyte) to the stack provided on ToS
: PUSHREGS ( reg ) >R BYTE@
  ( PC ) DUP %10000000 AND IF _PC C@ ( PCL ) R@ PUSH _PC 1 + C@ ( PCH ) R@ PUSH THEN
  ( U/S) DUP %01000000 AND IF
    R@ _S =
    IF \ pushing to S stack?
      _U  C@ ( UL  ) R@ PUSH _U  1 + C@ ( UH  ) R@ PUSH
    ELSE \ pushing to U (user) stack
      _S  C@ ( SL  ) R@ PUSH _S  1 + C@ ( SH  ) R@ PUSH
    THEN
  THEN
  ( Y  ) DUP %00100000 AND IF _Y  C@ ( YL  ) R@ PUSH _Y  1 + C@ ( YH  ) R@ PUSH THEN
  ( X  ) DUP %00010000 AND IF _X  C@ ( XL  ) R@ PUSH _X  1 + C@ ( XH  ) R@ PUSH THEN
  ( DP ) DUP %00001000 AND IF _DP C@ ( DP  ) R@ PUSH THEN
  ( B  ) DUP %00000100 AND IF _B  C@ ( B   ) R@ PUSH THEN
  ( A  ) DUP %00000010 AND IF _A  C@ ( A   ) R@ PUSH THEN
  ( CC ) DUP %00000001 AND IF _CC C@ ( CC  ) R@ PUSH THEN
  R> 2DROP
;

:NONAME ( PSHS  imm ) _S PUSHREGS ; $34 BIND
:NONAME ( PSHU  imm ) _U PUSHREGS ; $36 BIND

: PULL ( addr -- value ) DUP W@ DUP 1+ ROT W! TC@ ; \ leave reg R value on ToS, post-increment R, fetch from

\ Pull multiple regs (as indicated by postbyte) to the stack provided on ToS
: PULLREGS ( reg ) >R BYTE@
  ( CC ) DUP %00000001 AND IF R@ PULL _CC C! THEN
  ( A  ) DUP %00000010 AND IF R@ PULL _A  C! THEN
  ( B  ) DUP %00000100 AND IF R@ PULL _B  C! THEN
  ( DP ) DUP %00001000 AND IF R@ PULL _DP C! THEN
  ( X  ) DUP %00010000 AND IF R@ PULL ( XH ) _X  1+ C! R@ PULL ( XL ) _X  C! THEN
  ( Y  ) DUP %00100000 AND IF R@ PULL ( YH ) _Y  1+ C! R@ PULL ( YL ) _Y  C! THEN
  ( U/S) DUP %01000000 AND IF
    R@ _S =
    IF \ pushing to S stack?
      R@ PULL ( UH ) _U  1+ C! R@ PULL ( UL ) _U  C!
    ELSE \ pushing to U (user) stack
      R@ PULL ( SH ) _S  1+ C! R@ PULL ( SL ) _S  C!
    THEN
  THEN
  ( PC ) DUP %10000000 AND IF R@ PULL ( PCH) _PC 1+ C! R@ PULL ( PCL) _PC C! THEN

  R> 2DROP
;

:NONAME ( PULS  imm ) _S PULLREGS ; $35 BIND
:NONAME ( PULU  imm ) _U PULLREGS ; $37 BIND

: JSR ( addr ) _PC C@ ( PCL ) _S PUSH _PC 1 + C@ ( PCH ) _S PUSH _PC! ;
:NONAME ( JSR   dir ) 'DP    JSR ; $9D BIND
:NONAME ( JSR   ext ) 'EA    JSR ; $BD BIND
:NONAME ( JSR   ind ) 'IND   JSR ; $AD BIND

:NONAME ( BSR   rel ) BYTE@ SIGNEX8 _PC W@ + JSR ; $8D BIND
:NONAME ( LBSR  rel ) WORD@         _PC W@ + JSR ; $17 BIND

:NONAME ( RTI   inh ) ; $3B BIND
:NONAME ( RTS   inh ) _S PULL ( PCH) _PC 1+ C! _S PULL ( PCL) _PC C! ; $39 BIND

:NONAME ( SEX   inh ) _B C@ >NZ $80 AND IF $FF ELSE 0 THEN _A C! ; $1D BIND

: STA ( addr -- ) _A C@ >NZ 'V CLEAR SWAP TC! ;
:NONAME ( STA   dir ) 'DP        STA ; $97 BIND
:NONAME ( STA   ext ) 'EA        STA ; $B7 BIND
:NONAME ( STA   ind ) 'IND       STA ; $A7 BIND

: STB ( addr -- ) _B C@ >NZ 'V CLEAR SWAP TC! ;
:NONAME ( STB   dir ) 'DP        STB ; $D7 BIND
:NONAME ( STB   ext ) 'EA        STB ; $F7 BIND
:NONAME ( STB   ind ) 'IND       STB ; $E7 BIND

: STD ( addr -- ) _D W@ >NZW 'V CLEAR SWAP TW! ;
:NONAME ( STD   dir ) 'DP        STD ; $DD BIND
:NONAME ( STD   ext ) 'EA        STD ; $FD BIND
:NONAME ( STD   ind ) 'IND       STD ; $ED BIND

: STS ( addr -- ) _S W@ >NZW 'V CLEAR SWAP TW! ;
:NONAME ( STS   dir ) 'DP        STS ; $10DF BIND2
:NONAME ( STS   ext ) 'EA        STS ; $10FF BIND2
:NONAME ( STS   ind ) 'IND       STS ; $10EF BIND2

: STU ( addr -- ) _U W@ >NZW 'V CLEAR SWAP TW! ;
:NONAME ( STU   dir ) 'DP        STU ; $DF BIND
:NONAME ( STU   ext ) 'EA        STU ; $FF BIND
:NONAME ( STU   ind ) 'IND       STU ; $EF BIND

: STX ( addr -- ) _X W@ >NZW 'V CLEAR SWAP TW! ;
:NONAME ( STX   dir ) 'DP        STX ; $9F BIND
:NONAME ( STX   ext ) 'EA        STX ; $BF BIND
:NONAME ( STX   ind ) 'IND       STX ; $AF BIND

: STY ( addr -- ) _Y W@ >NZW 'V CLEAR SWAP TW! ;
:NONAME ( STY   dir ) 'DP        STY ; $109F BIND2
:NONAME ( STY   ext ) 'EA        STY ; $10BF BIND2
:NONAME ( STY   ind ) 'IND       STY ; $10AF BIND2

: SUB ( m c r -- r-m-c )
  ROT >R R@ -ROT \ push m to R  ( R: m   )
  >R + R@ SWAP -                ( R: m r )
  DUP $100 AND >C
  DUP R@ XOR $80 AND
  R>  R> XOR $80 AND
  AND >V
  >NZ
;

: SBCA ( m c -- ) _A C@ SUB _A C! ;
:NONAME ( SUBA  imm ) BYTE@      0  SBCA ; $80 BIND
:NONAME ( SUBA  dir ) 'DP    TC@ 0  SBCA ; $90 BIND
:NONAME ( SUBA  ext ) 'EA    TC@ 0  SBCA ; $B0 BIND
:NONAME ( SUBA  ind ) 'IND   TC@ 0  SBCA ; $A0 BIND

:NONAME ( SBCA  imm ) BYTE@      C> SBCA ; $82 BIND
:NONAME ( SBCA  dir ) 'DP    TC@ C> SBCA ; $92 BIND
:NONAME ( SBCA  ext ) 'EA    TC@ C> SBCA ; $B2 BIND
:NONAME ( SBCA  ind ) 'IND   TC@ C> SBCA ; $A2 BIND

: SBCB ( m c -- ) _B C@ SUB _B C! ;
:NONAME ( SUBB  imm ) BYTE@      0  SBCB ; $C0 BIND
:NONAME ( SUBB  dir ) 'DP    TC@ 0  SBCB ; $D0 BIND
:NONAME ( SUBB  ext ) 'EA    TC@ 0  SBCB ; $F0 BIND
:NONAME ( SUBB  ind ) 'IND   TC@ 0  SBCB ; $E0 BIND

:NONAME ( SBCB  imm ) BYTE@      C> SBCB ; $C2 BIND
:NONAME ( SBCB  dir ) 'DP    TC@ C> SBCB ; $D2 BIND
:NONAME ( SBCB  ext ) 'EA    TC@ C> SBCB ; $F2 BIND
:NONAME ( SBCB  ind ) 'IND   TC@ C> SBCB ; $E2 BIND

\ TODO: we could remove the carry, there's no SBC16 instruction
: SUB16 ( m c r -- r-m-c )
  ROT >R R@ -ROT \ push m to R  ( R: m   )
  >R + R@ SWAP -                ( R: m r )
  DUP $10000 AND >C
  DUP R@ XOR $8000 AND
  R>  R> XOR $8000 AND
  AND >V
  >NZW
;

: SUBD ( w -- ) _D W@ SUB16 _D W! ;
:NONAME ( SUBD  imm ) WORD@      0 SUBD ; $83 BIND
:NONAME ( SUBD  dir ) 'DP    TW@ 0 SUBD ; $93 BIND
:NONAME ( SUBD  ext ) 'EA    TW@ 0 SUBD ; $B3 BIND
:NONAME ( SUBD  ind ) 'IND   TW@ 0 SUBD ; $A3 BIND

: CMPA ( -- reg ) 0 _A C@ SUB DROP ;
:NONAME ( CMPA  imm ) BYTE@      CMPA ; $81 BIND
:NONAME ( CMPA  dir ) 'DP    TC@ CMPA ; $91 BIND
:NONAME ( CMPA  ext ) 'EA    TC@ CMPA ; $B1 BIND
:NONAME ( CMPA  ind ) 'IND   TC@ CMPA ; $A1 BIND

: CMPB ( -- reg ) 0 _B C@ SUB DROP ;
:NONAME ( CMPB  imm ) BYTE@      CMPB ; $C1 BIND
:NONAME ( CMPB  dir ) 'DP    TC@ CMPB ; $D1 BIND
:NONAME ( CMPB  ext ) 'EA    TC@ CMPB ; $F1 BIND
:NONAME ( CMPB  ind ) 'IND   TC@ CMPB ; $E1 BIND

: CMPD ( -- reg ) 0 _D W@ SUB16 ;
:NONAME ( CMPD  imm ) WORD@      CMPD ; $1083 BIND2
:NONAME ( CMPD  dir ) 'DP    TW@ CMPD ; $1093 BIND2
:NONAME ( CMPD  ext ) 'EA    TW@ CMPD ; $10B3 BIND2
:NONAME ( CMPD  ind ) 'IND   TW@ CMPD ; $10A3 BIND2

: CMPS ( -- reg ) 0 _S W@ SUB16 ;
:NONAME ( CMPS  imm ) WORD@      CMPS ; $118C BIND2
:NONAME ( CMPS  dir ) 'DP    TW@ CMPS ; $119C BIND2
:NONAME ( CMPS  ext ) 'EA    TW@ CMPS ; $11BC BIND2
:NONAME ( CMPS  ind ) 'IND   TW@ CMPS ; $11AC BIND2

: CMPU ( -- reg ) 0 _U W@ SUB16 ;
:NONAME ( CMPU  imm ) WORD@      CMPU ; $1183 BIND2
:NONAME ( CMPU  dir ) 'DP    TW@ CMPU ; $1193 BIND2
:NONAME ( CMPU  ext ) 'EA    TW@ CMPU ; $11B3 BIND2
:NONAME ( CMPU  ind ) 'IND   TW@ CMPU ; $11A3 BIND2

: CMPX ( -- reg ) 0 _X W@ SUB16 ;
:NONAME ( CMPX  imm ) WORD@      CMPX ; $8C BIND
:NONAME ( CMPX  dir ) 'DP    TW@ CMPX ; $9C BIND
:NONAME ( CMPX  ext ) 'EA    TW@ CMPX ; $BC BIND
:NONAME ( CMPX  ind ) 'IND   TW@ CMPX ; $AC BIND

: CMPY ( -- reg ) 0 _Y W@ SUB16 ;
:NONAME ( CMPY  imm ) WORD@      CMPY ; $108C BIND2
:NONAME ( CMPY  dir ) 'DP    TW@ CMPY ; $109C BIND2
:NONAME ( CMPY  ext ) 'EA    TW@ CMPY ; $10BC BIND2
:NONAME ( CMPY  ind ) 'IND   TW@ CMPY ; $10AC BIND2

:NONAME ( SWI   inh ) ; $3F BIND
:NONAME ( SWI2  inh ) ; $103F BIND2
:NONAME ( SWI3  inh ) ; $113F BIND2
:NONAME ( SYNC  inh ) ; $13 BIND

0 VALUE SRC_ADDR    0 VALUE SRC_WIDTH \ %1000 (8 bit) or 0 (16 bit)
0 VALUE DST_ADDR    0 VALUE DST_WIDTH \ %0100 (8 bit) or 0 (16 bit)

: SREG ( -- src-addr f )
  $F0 AND 4 RSHIFT DUP
  CELLS REGS + @ TO SRC_ADDR
  $08 AND        TO SRC_WIDTH ;

: DREG ( -- src-addr f )
  $0F AND DUP
  CELLS REGS + @ TO DST_ADDR
  $08 AND 2/     TO DST_WIDTH ;

: 'REG DUP DREG SREG ;

: OR<<8 DUP 8 LSHIFT OR ;

:NONAME ( TFR   imm )
  BYTE@ 'REG
  SRC_ADDR SRC_WIDTH IF C@ OR<<8 ELSE W@ THEN
  DST_ADDR DST_WIDTH IF C! ELSE W! THEN
; $1F BIND

:NONAME ( EXG   imm )
  BYTE@ 'REG
  \ Place both register values on the stack
  SRC_ADDR SRC_WIDTH IF C@ OR<<8 ELSE W@ THEN
  DST_ADDR DST_WIDTH IF C@ OR<<8 ELSE W@ THEN
  \ Save both values to the other register
  SRC_ADDR SRC_WIDTH IF C! ELSE W! THEN
  DST_ADDR DST_WIDTH IF C! ELSE W! THEN
; $1E BIND

\ -- store a minimal program

: ORG   DUP TO THERE _PC! ;
: _     TC, ;
: N NEXT ;

: GO 0 DO NEXT LOOP ;

0 VALUE FD
0 VALUE LEN

\ load binary rom file to target addr
: load-rom ( t-addr filename )
  0 to fd
  R/O OPEN-FILE THROW TO FD
  DUP RAM +     ( addr in host )
  $10000 ROT -  ( max chars to read )
  FD            ( fd )
  read-file throw TO LEN
;

$8100 s" tests/cputest.bin" load-rom
$8100 ORG

\ :NONAME ." ** 00 is not a valid opcode! **" ; $00 BIND

\ :NONAME _X W@ 83D9 = ; IS BREAKPOINT

\ When breakpoint, use LASTPC to locate error
925D BREAKAT