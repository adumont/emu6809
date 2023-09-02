# Emu6809: Motorola 6809 Emulator in FORTH

- [Emu6809: Motorola 6809 Emulator in FORTH](#emu6809-motorola-6809-emulator-in-forth)
- [Introduction](#introduction)
- [How to use](#how-to-use)
- [Challenges](#challenges)
- [References](#references)


# Introduction

This is a 6809 emulator written in gForth. You can load a 6809 binary rom and run it or step through.

Interupt related instructions are not implemented (CWAI, RTI,SWI, SWI2, SWI3, SYNC).

It is not cycle accurate.

Status: Passes [MC6809 CPU Test Suite by W. Schwotzer](https://github.com/aladur/flexemu/blob/master/src/tools/cputest.txt)

# How to use

I have written some [instructions here](tutorial.md).

# Challenges

Some things that make this emulator more challenging than [Emu6502](https://github.com/adumont/emu6502):

- Indirect addressing & the postbyte register bit assignments, Fig 16 page 17 of the datasheet
- Some opcodes are 2 bytes long (10xx and 11xx), which break the simple opcode table and opcode decoding mechanism implemented in Emu6502. Impacts: OPCODES2 & BIND2 + new logic for FETCH and DECODE
- Some instructions manipulate 16 bit values while other manipulate 8 bit values. Impacts low level words like >N which now needs a 16-bit version (>NW)
- Some instructions manipulate both 8 and 16 bit registers. Impacts is additional logic is needed in higher level words (like in TFR, EXG...)
- Flag handling (like overflow V or half-carry H) in Condition Code Register
- Books from the era have many uncorrected erratas (from wrong information about flags used in instructions to non-existant addressing modes!)

# References

- Motorola 6809 datasheet
- [The MC6809 CookBook (TAB BOOKS Inc)](https://colorcomputerarchive.com/repo/Documents/Books/The%20MC6809%20CookBook%20(TAB%20BOOKS%20Inc).pdf)
- MO6809 - MC6809E Microprocessor Programming Manual
- [6809 ISA in Python](https://github.com/craigthomas/CoCoAssembler/blob/main/cocoasm/instruction.py) from which I generated my instructions words templates

Tests suites:
- [MC6809 CPU Test Suite by W. Schwotzer](https://github.com/aladur/flexemu/blob/master/src/tools/cputest.txt)
- [Sean Riddle's Williams Emulation Test Suite](https://seanriddle.com/willyemutest04.asm)
