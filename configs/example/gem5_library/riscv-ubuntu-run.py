# Copyright (c) 2021 The Regents of the University of California
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
This script shows an example of running a full system RISCV Ubuntu boot
simulation using the gem5 library. This simulation boots Ubuntu 20.04 using
2 TIMING CPU cores. The simulation ends when the startup is completed
successfully.

Usage
-----

```
scons build/RISCV/gem5.opt
./build/RISCV/gem5.opt \
    configs/example/gem5_library/riscv-ubuntu-run.py
```
"""

import m5
from m5.objects import Root

from gem5.utils.requires import requires
from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.simulate.simulator import Simulator
from gem5.resources.workload import Workload

from gem5.resources.resource import Resource, CustomDiskImageResource #COSSIM

import os, argparse #COSSIM

from m5.objects import EtherLink, COSSIMEtherLink, EtherDump #COSSIM

default_kernel = 'riscv-bootloader-vmlinux-5.10-PCI' #COSSIM
default_disk   = 'riscv-ubuntu.img'

# This runs a check to ensure the gem5 binary is compiled for RISCV.

requires(isa_required=ISA.RISCV)

# With RISCV, we use simple caches.
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)

# Here we setup the parameters of the l1 and l2 caches.
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="16kB", l1i_size="16kB", l2_size="256kB"
)


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

# Memory: Dual Channel DDR4 2400 DRAM device.

memory = DualChannelDDR4_2400(size=args.mem_size)

# Here we setup the processor. We use a simple processor.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=ISA.RISCV, num_cores=args.num_cores
)

# Here we setup the board. The RiscvBoard allows for Full-System RISCV
# simulations.
board = RiscvBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.platform.attachRISCVTerminal(args.cossim, args.nodeNum) #COSSIM

board.readScript(args.script) #COSSIM

# Here we a full system workload: "riscv-ubuntu-20.04-boot" which boots
# Ubuntu 20.04. Once the system successfully boots it encounters an `m5_exit`
# instruction which stops the simulation. When the simulation has ended you may
# inspect `m5out/system.pc.com_1.device` to see the stdout.
#board.set_workload(Workload("riscv-ubuntu-20.04-boot"))

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
