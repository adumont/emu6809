# Emu6809: Motorola 6809 Emulator in FORTH

## Introduction

## Challenges

Some things that make this emulator more challenging than Emu6502:

- Indirect addressing & the postbyte register bit assignments, Fig 16 page 17 of the datasheet
- Some opcodes are 2 bytes long (10xx and 11xx), which break the simple opcode table and opcode decoding mechanism implemented in Emu6502.

## Reference

- Motorola 6809 datasheet
- [The MC6809 CookBook (TAB BOOKS Inc)](https://colorcomputerarchive.com/repo/Documents/Books/The%20MC6809%20CookBook%20(TAB%20BOOKS%20Inc).pdf)
- [MC6809 CPU Test Suite by W. Schwotzer](https://github.com/aladur/flexemu/blob/master/src/tools/cputest.txt)
- [6809 ISA in Python](https://github.com/craigthomas/CoCoAssembler/blob/main/cocoasm/instruction.py) from which I generated my instructions words templates