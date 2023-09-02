# EMU6809 User manual

- [EMU6809 User manual](#emu6809-user-manual)
- [Loading a program](#loading-a-program)
  - [Loading a binary rom file](#loading-a-binary-rom-file)
  - [Manually keying a program](#manually-keying-a-program)
- [Running a program](#running-a-program)
- [Breakpoints](#breakpoints)
  - [Breakpoint at ADDR](#breakpoint-at-addr)
  - [Implementing custom breakpoints](#implementing-custom-breakpoints)
- [Target memory Dump](#target-memory-dump)
- [6809 Register Operations](#6809-register-operations)
    - [6809 stack operations](#6809-stack-operations)


# Loading a program

## Loading a binary rom file

The following phrase will load the `forth.bin` file at address `$8000` in the emulator, and set the `PC` to `$8000`.

``` $8000 s" forth.bin" load-rom
```

Then we have to set the `PC` to `$8000`, we do that with:

``` $8000 ORG
```

Now we can start running the rom step by step or in continuous mode, see "Running a program" below.

## Manually keying a program

We can also type a machine language program manually (byte by byte). For that we will set the PC, and use `_` to commit every new byte:

```
$8000 ORG
4F _
1F _ 8B _
CE _ 04 _ 00 _
```

Once we have finished to enter every byte of the program, we can start running it step by step or in continuous mode, see "Running a program" below.

# Running a program

Use N (or NEXT) to run a single instruction (at PC)

Use k GO to run k instructions

Use RUN to run until a breakpoint is found

# Breakpoints

## Breakpoint at ADDR

One easy way to debug our program is to set a breakpoint at the start of the routine we want to debug.

For that we just have to type the address of the breakpoint, and run BREAKAT:

```
$802A BREAKAT
```

`BREAKAT` sets the value of the breakpoint.

By default, teh

## Implementing custom breakpoints

EMU6809 supports any kind of breakpoint. `BREAKPOINT` is a defered word. It's implementation should return a boolean.

`RUN`, the main loop of EMU6809 will keep calling the `NEXT` command as long as `BREAKPOINT` returns 0 (`FALSE`), and exit when `BREAKPOINT` returns something not 0.

For example if we want to break when register A is equal to 1, we could do it like this:

```
:NONAME _A C@ 1 = ;
IS BREAKPOINT
```

# Target memory Dump

The `TDUMP` word will help us inspect the target memory:

For example, dump 48 bytes ($30) starting at $8000:

```
8000 $30 tduMP
7FF4E6322800: 4F 1F 8B CE  04 00 10 CE - 03 00 8E 04  86 9F 02 8E  O...............
7FF4E6322810: 81 CA 9F 00  8E 04 80 BF - 04 80 8E 04  00 BF 04 82  ................
7FF4E6322820: 8E 04 00 BF  04 84 10 8E - 82 0E 6E B1  00 00 05 43  ..........n....C
```

Notice: The address is show is the absolute address in the host, not the address in teh target.

# 6809 Register Operations

The easiest way to see the registers is to use the STATUS word:

```
A  B X    Y    U    S    DP eFhINzvc CC PC
AB12 0400 0000 03FA 0300 00 01011000 58 CD00 > 00 00 00 00 00
     0000 81CA 0400 0000
     0000 0486 0000 0000
```

We can retrive the value of every register using the FORTH variables:

_A, _B, _D, _X, _Y, _U, _S, _CC and _PC

For 8 bits registers use C@, C! or C?
For 16 bits registers use W@, W! or W?

### 6809 stack operations

We can push a byte to the any of the two stack using, for example:

AB _U PUSH
C1 _S PUSH

Likewise we can also pull a byte from teh stacks using PULL:

_U PULL

or _S PULL





