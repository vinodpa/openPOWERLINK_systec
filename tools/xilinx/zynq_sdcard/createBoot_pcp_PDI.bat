cp ../../../fpga/xilinx/Xilinx_Z702/Zynq_ap_pcp_intaxi-axi/implementation/system.bit .
cp ../../../zynq_fsbl_dualProcessor/Debug/zynq_fsbl_dualProcessor.elf .
cp ../../../apps/ap_PDI/ap_pdi.elf .
cp ../../../powerlink/pcp_PDI/pcp_PDI.elf .
arm-xilinx-eabi-objcopy -I elf32-little -O binary pcp_PDI.elf pcp_PDI.bin
bootgen -image bootimage_pcp_PDI.bif -o i BOOT.BIN -w on 