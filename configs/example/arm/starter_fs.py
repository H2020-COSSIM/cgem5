# Copyright (c) 2016-2017, 2020, 2022 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

"""This script is the full system example script from the ARM
Research Starter Kit on System Modeling. More information can be found
at: http://www.arm.com/ResearchEnablement/SystemModeling
"""

import os
import m5
from m5.util import addToPath
from m5.objects import *
from m5.options import *
import argparse

m5.util.addToPath("../..")

from common import SysPaths
from common import ObjectList
from common import MemConfig
from common.cores.arm import O3_ARM_v7a, HPI

import devices


default_kernel = "vmlinux.arm64"
default_disk = "linaro-minimal-aarch64.img"
default_root_device = "/dev/vda1"


# Pre-defined CPU configurations. Each tuple must be ordered as : (cpu_class,
# l1_icache_class, l1_dcache_class, l2_Cache_class). Any of
# the cache class may be 'None' if the particular cache is not present.
cpu_types = {
    "atomic": (AtomicSimpleCPU, None, None, None),
    "minor": (MinorCPU, devices.L1I, devices.L1D, devices.L2),
    "hpi": (HPI.HPI, HPI.HPI_ICache, HPI.HPI_DCache, HPI.HPI_L2),
    "o3": (
        O3_ARM_v7a.O3_ARM_v7a_3,
        O3_ARM_v7a.O3_ARM_v7a_ICache,
        O3_ARM_v7a.O3_ARM_v7a_DCache,
        O3_ARM_v7a.O3_ARM_v7aL2,
    ),
}


def create_cow_image(name):
    """Helper function to create a Copy-on-Write disk image"""
    image = CowDiskImage()
    image.child.image_file = SysPaths.disk(name)

    return image


def create(args):
    """Create and configure the system object."""

    if args.script and not os.path.isfile(args.script):
        print("Error: Bootscript %s does not exist" % args.script)
        sys.exit(1)

    cpu_class = cpu_types[args.cpu][0]
    mem_mode = cpu_class.memory_mode()
    # Only simulate caches when using a timing CPU (e.g., the HPI model)
    want_caches = True if mem_mode == "timing" else False

    system = devices.SimpleSystem(
        want_caches,
        args.mem_size,
        cossim_enabled=args.cossim, #COSSIM
        nodeNum=args.nodeNum,       #COSSIM
        mem_mode=mem_mode,
        workload=ArmFsLinux(object_file=SysPaths.binary(args.kernel)),
        readfile=args.script,
    )

    MemConfig.config_mem(args, system)

    # Add the PCI devices we need for this system. The base system
    # doesn't have any PCI devices by default since they are assumed
    # to be added by the configuration scripts needing them.
    system.pci_devices = [
        # Create a VirtIO block device for the system's boot
        # disk. Attach the disk image using gem5's Copy-on-Write
        # functionality to avoid writing changes to the stored copy of
        # the disk image.
        PciVirtIO(vio=VirtIOBlock(image=create_cow_image(args.disk_image)))
    ]

    # Attach the PCI devices to the system. The helper method in the
    # system assigns a unique PCI bus ID to each of the devices and
    # connects them to the IO bus.
    for dev in system.pci_devices:
        system.attach_pci(dev)

    # Wire up the system's memory system
    system.connect()

    # Add CPU clusters to the system
    system.cpu_cluster = [
        devices.CpuCluster(
            system, args.num_cores, args.cpu_freq, "1.0V", *cpu_types[args.cpu]
        )
    ]

    # Create a cache hierarchy for the cluster. We are assuming that
    # clusters have core-private L1 caches and an L2 that's shared
    # within the cluster.
    system.addCaches(want_caches, last_cache_level=2)

    # Setup gem5's minimal Linux boot loader.
    system.realview.setupBootLoader(system, SysPaths.binary)

    if args.dtb:
        system.workload.dtb_filename = args.dtb
    else:
        # No DTB specified: autogenerate DTB
        system.workload.dtb_filename = os.path.join(
            m5.options.outdir, "system.dtb"
        )
        system.generateDtb(system.workload.dtb_filename)

    if args.initrd:
        system.workload.initrd_filename = args.initrd

    # Linux boot command flags
    kernel_cmd = [
        # Tell Linux to use the simulated serial port as a console
        "console=ttyAMA0",
        # Hard-code timi
        "lpj=19988480",
        # Disable address space randomisation to get a consistent
        # memory layout.
        "norandmaps",
        # Tell Linux where to find the root disk image.
        "root=%s" % args.root_device,
        # Mount the root disk read-write by default.
        "rw",
        # Tell Linux about the amount of physical memory present.
        "mem=%s" % args.mem_size,
    ]
    system.workload.command_line = " ".join(kernel_cmd)

    return system

def addEthernet(system, options): #COSSIM
    # create NIC
    dev = IGbE_e1000()
    system.attach_pci(dev)
    system.ethernet = dev
    
    system.etherlink = COSSIMEtherLink(nodeNum=options.nodeNum, TotalNodes=options.TotalNodes, sys_clk=options.sys_clock,SynchTime=options.SynchTime, RxPacketTime=options.RxPacketTime) #system_clock is used for synchronization
    
    system.etherlink.interface = Parent.system.ethernet.interface
    if options.etherdump:
        system.etherdump = EtherDump(file=options.etherdump)
        system.etherlink.dump = system.etherdump

def addStandAloneEthernet(system, options): #COSSIM
    # create NIC
    dev = IGbE_e1000()
    system.attach_pci(dev)
    system.ethernet = dev
    
    system.etherlink = EtherLink()
    
    system.etherlink.int0 = Parent.system.ethernet.interface
    if options.etherdump:
        system.etherdump = EtherDump(file=options.etherdump)
        system.etherlink.dump = system.etherdump


def run(args):
    # Remove existing files from previous simulation (COSSIM)
    NodeNum = args.nodeNum
    os.system("rm -rf $GEM5/McPat/mcpatNode" + str(NodeNum) + ".xml")
    os.system("rm -rf $GEM5/McPat/mcpatOutput" + str(NodeNum) + ".txt")
    os.system("rm -rf $GEM5/McPat/energy" + str(NodeNum) + ".txt")
    # END Remove existing files from previous simulation (COSSIM)
    cptdir = m5.options.outdir
    if args.checkpoint:
        print("Checkpoint directory: %s" % cptdir)

    while True:
        event = m5.simulate()
        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print("Dropping checkpoint at tick %d" % m5.curTick())
            cpt_dir = os.path.join(m5.options.outdir, "cpt.%d" % m5.curTick())
            m5.checkpoint(os.path.join(cpt_dir))
            print("Checkpoint done.")
        else:
            print(exit_msg, " @ ", m5.curTick())
            break

    # Execute the McPat Script (COSSIM)   
    if exit_msg == "m5_exit instruction encountered":
        McPATXml = args.McPATXml #Specify the McPAT xml ProcessorDescriptionFile
        if McPATXml == "empty":
            print ("Power results are not available because McPat xml file is not exist!\n")
        else:
            print ("Power results are calculated with xml file: " + McPATXml + "\n")
            os.system("$GEM5/runMcPat.sh " + str(NodeNum) + " " + str(McPATXml) + " &")
    # END Execute the McPat Script (COSSIM)

    sys.exit(event.getCode())


def main():
    parser = argparse.ArgumentParser(epilog=__doc__)

    parser.add_argument(
        "--dtb", type=str, default=None, help="DTB file to load"
    )
    parser.add_argument(
        "--kernel", type=str, default=default_kernel, help="Linux kernel"
    )
    parser.add_argument(
        "--initrd",
        type=str,
        default=None,
        help="initrd/initramfs file to load",
    )
    parser.add_argument(
        "--disk-image",
        type=str,
        default=default_disk,
        help="Disk to instantiate",
    )
    parser.add_argument(
        "--root-device",
        type=str,
        default=default_root_device,
        help="OS device name for root partition (default: {})".format(
            default_root_device
        ),
    )
    parser.add_argument(
        "--script", type=str, default="", help="Linux bootscript"
    )
    parser.add_argument(
        "--cpu",
        type=str,
        choices=list(cpu_types.keys()),
        default="atomic",
        help="CPU model to use",
    )
    parser.add_argument("--cpu-freq", type=str, default="4GHz")
    parser.add_argument(
        "--num-cores", type=int, default=1, help="Number of CPU cores"
    )
    parser.add_argument(
        "--mem-type",
        default="DDR3_1600_8x8",
        choices=ObjectList.mem_list.get_names(),
        help="type of memory to use",
    )
    parser.add_argument(
        "--mem-channels", type=int, default=1, help="number of memory channels"
    )
    parser.add_argument(
        "--mem-ranks",
        type=int,
        default=None,
        help="number of memory ranks per channel",
    )
    parser.add_argument(
        "--mem-size",
        action="store",
        type=str,
        default="2GB",
        help="Specify the physical memory size",
    )
    parser.add_argument("--checkpoint", action="store_true")
    parser.add_argument("--restore", type=str, default=None)

    #COSSIM Options
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
    
    parser.add_argument("--mcpat-xml", action="store", type=str, default="empty", dest="McPATXml",
                      help="Specify the McPAT xml ProcessorDescriptionFile")

    args = parser.parse_args()

    root = Root(full_system=True)
    root.system = create(args)
    
    if args.cossim:                    #COSSIM
        addEthernet(root.system, args) #COSSIM
    else:
        addStandAloneEthernet(root.system, args) #COSSIM

    if args.restore is not None:
        m5.instantiate(args.restore)
    else:
        m5.instantiate()

    run(args)


if __name__ == "__m5_main__":
    main()
