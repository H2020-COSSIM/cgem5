# Copyright (c) 2022 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This gem5 configuration script runs the RISCVMatchedBoard in FS mode with a
an Ubuntu 20.04 image and calls m5 exit after the simulation has booted the OS.

Usage
---

```
scons build/RISCV/gem5.opt

./build/RISCV/gem5.opt configs/example/gem5_library/riscvmatched-fs.py
```
"""

from gem5.prebuilt.riscvmatched.riscvmatched_board import RISCVMatchedBoard
from gem5.utils.requires import requires
from gem5.isas import ISA
from gem5.simulate.simulator import Simulator
from gem5.resources.workload import Workload

import argparse

from gem5.resources.resource import Resource, CustomDiskImageResource #COSSIM

import os #COSSIM

from m5.objects import EtherLink, COSSIMEtherLink, EtherDump #COSSIM

default_kernel = 'riscv-bootloader-vmlinux-5.10-PCI' #COSSIM
default_disk   = 'riscv-ubuntu.img' #COSSIM

requires(isa_required=ISA.RISCV)

#parser = argparse.ArgumentParser(
#    description="A script which uses the RISCVMatchedBoard in FS mode."
#)

#parser.add_argument(
#    "-i",
#    "--to-init",
#    action="store_true",
#    help="Exit the simulation after the Linux Kernel boot.",
#)


# ----------------------------- Add Options (COSSIM) ---------------------------- #
parser = argparse.ArgumentParser()
parser.add_argument("--kernel", type=str, default=default_kernel,
                        help="Linux kernel")

parser.add_argument("--disk-image", type=str,
                        default=default_disk,
                        help="Disk to instantiate")

parser.add_argument("--cossim", action="store_true",
                      help="COSSIM distributed gem5 simulation.")
    
parser.add_argument("--nodeNum", action="store", type=int, dest="nodeNum", default=0,
                      help="Specify the number of node")
    
parser.add_argument("--SynchTime", action="store", type=str, dest="SynchTime",
                      help="Specify the Synchronization Time. For example: --SynchTime=1ms")
    
parser.add_argument("--RxPacketTime", action="store", type=str, dest="RxPacketTime",
                      help="Specify the minimum time in which the node can accept packet from the OMNET++. For example: --SynchTime=1ms")
    
parser.add_argument("--TotalNodes", action="store", type=str, dest="TotalNodes", default=1,
                      help="Specify the total number of nodes")
    
parser.add_argument("--sys-clock", action="store", type=str, dest="sys_clock", 
                      default="1GHz",
                      help = """Top-level clock for blocks running at system
                      speed""")

parser.add_argument("--etherdump", action="store", type=str, default="",
                      help="Specify the filename to dump a pcap capture of"\
                      " the ethernet traffic")

parser.add_argument("--script", type=str, default="",
                        help = "Linux bootscript")

parser.add_argument("--num-cores", type=int, default=2,
                        help="Number of CPU cores")

parser.add_argument("--mem-size", action="store", type=str,
                        default="3GB",
                        help="Specify the physical memory size")

args = parser.parse_args()
# ---------------------------- Parse Options --------------------------- #

# instantiate the riscv matched board with default parameters
board = RISCVMatchedBoard(
    clk_freq="1.2GHz",
    l2_size="2MB",
    is_fs=True,
)

board.platform.attachRISCVTerminal(args.cossim, args.nodeNum) #COSSIM

board.readScript(args.script) #COSSIM

# Here we a full system workload: "riscv-ubuntu-20.04-boot" which boots
# Ubuntu 20.04. Once the system successfully boots it encounters an `m5_exit`
# instruction which stops the simulation. When the simulation has ended you may
# inspect `m5out/system.pc.com_1.device` to see the stdout.
#
# In the case where the `-i` flag is passed, we add the kernel argument
# `init=/root/exit.sh`. This means the simulation will exit after the Linux
# Kernel has booted.
#workload = Workload("riscv-ubuntu-20.04-boot")
#kernel_args = board.get_default_kernel_args()
#if args.to_init:
#    kernel_args.append("init=/root/exit.sh")
#workload.set_parameter("kernel_args", kernel_args)
#board.set_workload(workload)


# ----------------------------- Add specific kernel & image (COSSIM) ---------------------------- #
kernel_path = os.getenv('M5_PATH') + "/binaries/" + args.kernel
image_path  = os.getenv('M5_PATH') + "/disks/"    + args.disk_image

kernel_custom=CustomDiskImageResource( #COSSIM PCI Kernel
    local_path = kernel_path
)

image_custom = CustomDiskImageResource(
    local_path = image_path,
    disk_root_partition = "1",
)

# Set the Full System workload.
board.set_kernel_disk_workload(
                   #kernel=Resource("riscv-bootloader-vmlinux-5.10"),
                   kernel=kernel_custom, #COSSIM
                   #disk_image=Resource("riscv-disk-img"),
                   disk_image=image_custom,
                   fast_boot_ubuntu=True,
)

# ----------------------------- END Add specific image (COSSIM) ---------------------------- #

if args.cossim: #COSSIM
    board.etherlink = COSSIMEtherLink(nodeNum=args.nodeNum, TotalNodes=args.TotalNodes, sys_clk=args.sys_clock,SynchTime=args.SynchTime, RxPacketTime=args.RxPacketTime) #system_clock is used for synchronization     
    board.etherlink.interface = board.ethernet.interface
else:
    board.etherlink = EtherLink()
    board.etherlink.int0 = board.ethernet.interface

if args.etherdump: #COSSIM
    board.etherdump = EtherDump(file=args.etherdump)
    board.etherlink.dump = board.etherdump


simulator = Simulator(board=board)
simulator.run()
