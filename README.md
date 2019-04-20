# Project Title

OX is a simple embeddable kernel released under the GPL. This
version contains LWIP networking code, but this has not been
tested. This version (1.3) is an Alpha release. The kernel is
small and written in C and NASM assembler.

## Getting Started

To build this system, first edit make\_links.sh
to change the root work area OXHOME environment
variable. If you installed the code in /home/foo/ox,
then that should be your OXHOME.

After editing your OXHOME environment variable,
run make\_links.sh and then run make.

### Prerequisites

A version of NASM is needed to build as well as a version of gcc.
You do not need C++ as no C++ code is in the kernel.

### And coding style tests

Not much to say about style, but try to keep curly braces
starting on the same line, for example:

for(...) {
}

As opposed to

for(...)
{
}

And, the code aims to use snake case where applicable.

## Deployment


The executable can then be run inside an emulator
such as QEMU. It may not run in other emulators,
and at one point was tested on real hardware, but
the system is not complete yet.

For notes on using it with QEMU, see the
notes/qemu.txt file.

## Built With

* NASM the main assembler files are all in NASM, and we avoided
  using AT&T syntax, favoring Intel.

* ANSI C, there was an attempt to keep the C code ANSI as much as
  possible.

## Contributing

Anyone wishing to contribute may do so under the terms of the GPL license.
Simple clone the repo and code. When you are done, do a pull request and
if your changes are good, I will accept them.

## Versioning

This repo is version ox 1.3, it differs from the original ox branch
and includes an attempt to implement a TCP/IP stack. Much work is needed
to get this working correctly.

## Authors

* This kernel was started by Roger Doss, PhD. However, several other
developers have contributed.

## License

This project is licensed under GPL - see the COPYING file for details.
The networking code is from LWIP project which is licensed under BSD style license.

## Acknowledgments

* Thanks to everyone whose code was used/contributed.
  See the CREDITS file for credits.
