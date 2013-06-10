Altera Nios II Managing Node {#altera-mn}
============================

## Introduction

The Altera Nios II Managing Node uses the POWERLINK IP-Core,
which consists of an optimized MAC for POWERLINK (openMAC) and
a hub for daisy chaining several nodes.
Additionally, the MN demos apply two separate processors named:
- POWERLINK Communication Processor (Pcp)
- Host Processor

The two processors are connected by the Host Interface IP-Core,
which supports the following interface implementations:
- Altera Avalon interconnect
- External (de-)multiplexed address-/data-bus

## Contents

- FPGA designs with Nios II CPU, POWERLINK and Host Interface IP-Cores.
 - Pcp and Host in single FPGA
 - Pcp and Host in separate FPGAs
- openCONFIGURATOR project for 10 CNs

## Requirements

- Altera Quartus II v12.1 SP1 (Web Edition is also possible)
  - <https://www.altera.com/download/archives/arc-index.jsp>
  - <ftp://ftp.altera.com/outgoing/release/>
- Experiences with this development environment is required.
- Development Board TERASIC_DE2-115 (INK Board)
- Optional: Second development Board TERASIC_DE2-115 (INK Board) if you want to run the separate FPGA demo.
- Optional: 40-line ribbon cable to connect two INK boards via JP5 (length as short as possible).

## Hardware Setup

- Setup for TERASIC_DE2-115 (INK)
 - Download the user guide for the board from the
 - [Terasic Website](http://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=502&PartNo=4)
 - Set the jumpers JP1, JP2, JP3, JP6 and JP7.
 - Set switch SW 19 to Run.

- Single FPGA demo
 - Connect one Ethernet port of the INK-board to the POWERLINK network.
 - Switch on the board with the Power button (SW 18).
- Two FPGA demo
 - Connect one Ethernet port of one INK-board to the POWERLINK network.
 - Make sure that JP6 on both INK boards are at same position (3.3V).
 - Connect the two INK boards with a 40-line ribbon cable through JP5.
 - Switch on the two boards with the Power button (SW 18).

## How to build the binaries

The Altera Quartus II projects are located in `/fpga/boards/altera/TERASIC_DE2-115.`\n
The subdirectories starting with `mn_` refer to designs for POWERLINK MN demos:
- The single FPGA demo can be found in `mn_dual_nios2`.
- The external host interface demo is implemented in `mn_par_host` and `mn_par_pcp`.

Steps 1-5 are only necessary if you want to change the FPGA designs. Otherwise you can use the supplied SOF file and go directly to step 6.

1. Open the Quartus project file `POWERLINK_MN.qpf` with Altera Quartus II according to the demo you want to use.
2. Open Qsys via menu *Tools* -> *Qsys*.
3. Open the top-level Qsys `top_mn.qsys`, change to *Generation* page and press the button *Generate* to generate the Nios II system.
4. Close Qsys when the generation has finished (shown as information output).
5. Start the compilation in the Quartus II window via menu *Processing* -> *Start Compilation*.
6. Open "Nios II Command Shell"
7. There are two software designs available in the openPOWERLINK subdirectory \n
   `/stack/make/driver/altera_nios2/mn_pcp` (for the Pcp) and \n
   `Examples/arch/altera_nios2/no_os/gnu/demo_mn (for the host)`. \n
   Change to each of the directories and open the create-this-app.settings file.
   Make sure that the SOPC_DIR variable is set to the board design of your choice. \n
   If you want to run the single FPGA solution set `SOPC_DIR=../../../../../fpga/boards/altera/TERASIC_DE2-115/mn_dual_nios2/` \n
   If you want to run the two FPGA demo set `SOPC_DIR=../../../../../fpga/boards/altera/TERASIC_DE2-115/mn_par_pcp/`
8. Run script *create-this-app* in `mn_pcp` and `demo_mn` to create the Makefile for application and BSP. \n
   `$ ./create-this-app`
9. Run make to build the ELF file after changing the sources. (The script create-this-app will do this automatically) \n
   `$ make`
10. Rebuild the Makefile for `mn_pcp` and `demo_mn` also, if the Nios II Design was changed inside Qsys. \n
    `$ ./create-this-app --rebuild`

## How to run the demo

1. Program SOF file with Quartus II Programmer into FPGA.
   It is located in the following subdirectory of openPOWERLINK main directory: \n
   `/fpga/boards/altera/TERASIC_DE2-115/mn_dual_nios2/POWERLINK_MN.sof` \n
   or
   `/fpga/boards/altera/TERASIC_DE2-115/mn_par_pcp/POWERLINK_MN.sof` \n
   and
   `/fpga/boards/altera/TERASIC_DE2-115/mn_par_host/POWERLINK_MN.sof`
2. Download the ELF file to the target:
   Always follow the sequence by downloading the Pcp ELF file first, afterwards the host ELF. \n
   For the two FPGA demo enter in both directories in the "Nios II Command Shell" \n
   `$ make download-elf` \n
   For the single FPGA demo enter in the pcp directory in the "Nios II Command Shell" \n
   `$ nios2-download -i 0 epl.elf --go` \n
   For the host enter in the host directory in the "Nios II Command Shell" \n
   `$ nios2-download -i 1 epl.elf --go` \n
3. Enjoy the running POWERLINK network.

## How to import the project into Nios II Software Build Tools for Eclipse for debugging purposes

Requirement: Steps in the previous section *How to build the binary* are completed.

1. Start the Nios II Software Build Tools for Eclipse
2. Select menu *File -> Import...*
3. Select the import source *General* -> *Existing Projects into Workspace*
4. Browse to `Examples/arch/altera_nios2/no_os/gnu/demo_mn` (via the button *Browse...*)
5. Press the button *Finish*.
6. Browse to `stack/make/driver/altera_nios2/mn_pcp` (via the button *Browse...*)
7. Press the button *Finish*.

## How to write the program to local flash

Requirement: Steps in the previous section *How to build the binary* are completed.

1. After successfully building the design use the makefile to program the flash:\n
   `$ make program-epcs`

## Troubleshooting

1. It is adviced to clean all generated files after switching from one demo to the other.
2. Always download the Pcp ELF file before the host ELF file.

