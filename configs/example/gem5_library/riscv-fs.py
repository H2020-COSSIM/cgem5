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
This example runs a simple linux boot. It uses the 'riscv-disk-img' resource.
It is built with the sources in `src/riscv-fs` in [gem5 resources](
https://gem5.googlesource.com/public/gem5-resources).

Characteristics
---------------

* Runs exclusively on the RISC-V ISA with the classic caches
* Assumes that the kernel is compiled into the bootloader
* Automatically generates the DTB file
* Will boot but requires a user to login using `m5term` (username: `root`,
  password: `root`)
"""

from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.utils.requires import requires
from gem5.resources.resource import Resource, CustomDiskImageResource #COSSIM
from gem5.simulate.simulator import Simulator

import os, argparse #COSSIM

from m5.objects import EtherLink, COSSIMEtherLink, EtherDump #COSSIM

default_kernel = 'riscv-bootloader-vmlinux-5.10-PCI' #COSSIM
default_disk   = 'riscv-disk-img'

# Run a check to ensure the right version of gem5 is being used.
requires(isa_required=ISA.RISCV)

# Setup the cache hierarchy.
# For classic, PrivateL1PrivateL2 and NoCache have been tested.
# For Ruby, MESI_Two_Level and MI_example have been tested.
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB"
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

parser.add_argument("--num-cores", type=int, default=1,
                        help="Number of CPU cores")

parser.add_argument("--mem-size", action="store", type=str,
                        default="3GB",
                        help="Specify the physical memory size")


args = parser.parse_args()
# ---------------------------- Parse Options --------------------------- #

# Setup the system memory.
memory = SingleChannelDDR3_1600(size = args.mem_size)

# Setup a single core Processor.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=ISA.RISCV, num_cores=args.num_cores
)

# Setup the board.
board = RiscvBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.platform.attachRISCVTerminal(args.cossim, args.nodeNum) #COSSIM

board.readScript(args.script) #COSSIM

# Set the Full System workload.
board.set_kernel_disk_workload(
    kernel=Resource("riscv-bootloader-vmlinux-5.10"),
    disk_image=Resource("riscv-disk-img"),
)

# ----------------------------- Add specific kernel & image (COSSIM) ---------------------------- #
kernel_path = os.getenv('M5_PATH') + "/binaries/" + args.kernel
image_path  = os.getenv('M5_PATH') + "/disks/"    + args.disk_image

kernel_custom=CustomDiskImageResource( #COSSIM PCI Kernel
    local_path = kernel_path
)

image_custom = CustomDiskImageResource(
    local_path = image_path
)

# Set the Full System workload.
board.set_kernel_disk_workload(
                   #kernel=Resource("riscv-bootloader-vmlinux-5.10"),
                   kernel=kernel_custom, #COSSIM
                   #disk_image=Resource("riscv-disk-img"),
                   disk_image=image_custom,
)

if args.cossim: #COSSIM
    board.etherlink = COSSIMEtherLink(nodeNum=args.nodeNum, TotalNodes=args.TotalNodes, sys_clk=args.sys_clock,SynchTime=args.SynchTime, RxPacketTime=args.RxPacketTime) #system_clock is used for synchronization     
    board.etherlink.interface = board.ethernet.interface
else:
    board.etherlink = EtherLink()
    board.etherlink.int0 = board.ethernet.interface

if args.etherdump: #COSSIM
    board.etherdump = EtherDump(file=args.etherdump)
    board.etherlink.dump = board.etherdump

# ----------------------------- END Add specific kernel & image (COSSIM) ---------------------------- #

simulator = Simulator(board=board)
print("Beginning simulation!")
# Note: This simulation will never stop. You can access the terminal upon boot
# using m5term (`./util/term`): `./m5term localhost <port>`. Note the `<port>`
# value is obtained from the gem5 terminal stdout. Look out for
# "system.platform.terminal: Listening for connections on port <port>".
simulator.run()
