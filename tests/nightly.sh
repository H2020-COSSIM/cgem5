#!/bin/bash

# Copyright (c) 2021 The Regents of the University of California
# All Rights Reserved.
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

set -e
set -x

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
gem5_root="${dir}/.."

# The first argument is the number of threads to be used for compilation. If no
# argument is given we default to one.
compile_threads=1
if [[ $# -gt 0 ]]; then
    compile_threads=$1
fi

# The second argument is the number of threads used to run gem5 (one gem5
# instance per thread). If no argument is given we default to one.
run_threads=1
if [[ $# -gt 1 ]]; then
    run_threads=$2
fi

# The third argument is the GPU ISA to run. If no argument is given we default
# to GCN3_X86.
gpu_isa=GCN3_X86
if [[ $# -gt 2 ]]; then
    gpu_isa=$3
fi

if [[ "$gpu_isa" != "GCN3_X86" ]] && [[ "$gpu_isa" != "VEGA_X86" ]]; then
    echo "Invalid GPU ISA: $gpu_isa"
    exit 1
fi

build_target () {
    isa=$1

    # Try to build. If not, delete the build directory and try again.
    # SCons is not perfect, and occasionally does not catch a necessary
    # compilation: https://gem5.atlassian.net/browse/GEM5-753
    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --rm \
        gcr.io/gem5-test/ubuntu-20.04_all-dependencies:v21-2 \
            bash -c "scons build/${isa}/gem5.opt -j${compile_threads} \
                || (rm -rf build && scons build/${isa}/gem5.opt -j${compile_threads})"
}

unit_test () {
    build=$1

    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --rm \
        gcr.io/gem5-test/ubuntu-20.04_all-dependencies:v21-2 \
            scons build/NULL/unittests.${build} -j${compile_threads}
}

# Ensure we have the latest docker images.
docker pull gcr.io/gem5-test/ubuntu-20.04_all-dependencies:v21-2

# Try to build the ISA targets.
build_target NULL
build_target RISCV
build_target X86
build_target ARM
build_target SPARC
build_target MIPS
build_target POWER

# Run the unit tests.
unit_test opt
unit_test debug

# Run the gem5 long tests.
docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}"/tests --rm \
    gcr.io/gem5-test/ubuntu-20.04_all-dependencies:v21-2 \
        ./main.py run --length long -j${compile_threads} -t${run_threads} -vv

# For the GPU tests we compile and run the GPU ISA inside a gcn-gpu container.
docker pull gcr.io/gem5-test/gcn-gpu:v21-2
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" gcr.io/gem5-test/gcn-gpu:v21-2  bash -c \
    "scons build/${gpu_isa}/gem5.opt -j${compile_threads} \
        || (rm -rf build && scons build/${gpu_isa}/gem5.opt -j${compile_threads})"

# get square
wget -qN http://dist.gem5.org/dist/v21-2/test-progs/square/square

mkdir -p tests/testing-results

# Square is the simplest, fastest, more heavily tested GPU application
# Thus, we always want to run this in the nightly regressions to make sure
# basic GPU functionality is working.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" gcr.io/gem5-test/gcn-gpu:v21-2  build/${gpu_isa}/gem5.opt \
    configs/example/apu_se.py --reg-alloc-policy=dynamic -n3 -c square

# get HeteroSync
wget -qN http://dist.gem5.org/dist/v21-2/test-progs/heterosync/gcn3/allSyncPrims-1kernel

# run HeteroSync sleepMutex -- 16 WGs (4 per CU in default config), each doing
# 10 Ld/St per thread and 4 iterations of the critical section is a reasonable
# moderate contention case for the default 4 CU GPU config and help ensure GPU
# atomics are tested.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" gcr.io/gem5-test/gcn-gpu:v21-2 build/${gpu_isa}/gem5.opt \
    configs/example/apu_se.py --reg-alloc-policy=dynamic -n3 -c \
    allSyncPrims-1kernel --options="sleepMutex 10 16 4"

# run HeteroSync LFBarr -- similar setup to sleepMutex above -- 16 WGs
# accessing unique data and then joining a lock-free barrier, 10 Ld/St per
# thread, 4 iterations of critical section.  Again this is representative of a
# moderate contention case for the default 4 CU GPU config and help ensure GPU
# atomics are tested.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" gcr.io/gem5-test/gcn-gpu:v21-2  build/${gpu_isa}/gem5.opt \
    configs/example/apu_se.py --reg-alloc-policy=dynamic -n3 -c \
    allSyncPrims-1kernel --options="lfTreeBarrUniq 10 16 4"

# Run an SST test.
build_and_run_SST () {
    isa=$1
    variant=$2

    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --rm gcr.io/gem5-test/sst-env:v21-2 \
            bash -c "\
scons build/${isa}/libgem5_${variant}.so -j${compile_threads} --without-tcmalloc; \
cd ext/sst; \
make clean; make -j ${compile_threads}; \
sst --add-lib-path=./ sst/example.py; \
"
}
build_and_run_SST RISCV opt
