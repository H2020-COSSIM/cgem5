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
A run script for a very simple Syscall-Execution running simple binaries.
The system has no cache heirarchy and is as "bare-bones" as you can get in
gem5 while still being functinal.
"""

from gem5.resources.resource import Resource
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.simulate.simulator import Simulator

import argparse

parser = argparse.ArgumentParser(
    description="A gem5 script for running simple binaries in SE mode."
)

parser.add_argument(
    "resource",
    type=str,
    help="The gem5 resource binary to run.",
)

parser.add_argument(
    "cpu",
    type=str,
    choices=("kvm", "timing", "atomic", "o3"),
    help="The CPU type used.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()

def input_to_cputype(input: str) -> CPUTypes:
    if input == "kvm":
        return CPUTypes.KVM
    elif input == "timing":
        return CPUTypes.TIMING
    elif input == "atomic":
        return CPUTypes.ATOMIC
    elif input == "o3":
        return CPUTypes.O3
    else:
        raise NotADirectoryError("Unknown CPU type '{}'.".format(input))

# Setup the system.
cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600()
processor = SimpleProcessor(cpu_type=input_to_cputype(args.cpu), num_cores=1)

motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set the workload
binary = Resource(args.resource,
        resource_directory=args.resource_directory)
motherboard.set_se_binary_workload(binary)

# Run the simulation
simulator = Simulator(board=motherboard, full_system=False)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(),
        simulator.get_last_exit_event_cause(),
    )
)
