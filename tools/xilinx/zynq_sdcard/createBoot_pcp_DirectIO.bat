cp ../../../fpga/xilinx/Xilinx_Z702/Zynq_pcp_DirectIO/implementation/system.bit .
cp ../../../apps/zynq/digitalIO-Init/digitalIO-Init.elf .
cp ../../../zynq_fsbl_digitalIO/Debug/zynq_fsbl_digitalIO.elf .
cp ../../../powerlink/pcp_DirectIO/directIO.elf .
arm-xilinx-eabi-objcopy -I elf32-little -O binary directIO.elf directIO.bin
bootgen -image bootimage_pcp_DirectIO.bif -o i BOOT.BIN -w on 