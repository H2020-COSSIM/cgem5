# cgem5 - COSSIM-modified gem5 version. 

This repository includes the modified gem5 simulator that integrates with the rest of the COSSIM framework. The cgem5 supports interconnection with IEEE HLA interfaces and modifies the network interface so that it can communicate with other cgem5 nodes through a network simulator. It should be noted that cgem5 can be used independently of COSSIM as a standalone package incorporating all the changes that have been integrated to the official GEM5 November 2022 release (v22.0.0.1).

## Differences between cgem5 and official gem5 v22.0.0.1

In the following subsections the limitations of the current, publicly available, version of GEM5 are described in tandem with the modifications and extensions that have been implemented to alleviate those restrictions.

### Extending the Network Model of GEM5

In GEM5’s publicly available repositories, the only network interface card implemented, tested and verified is the Intel 8254x based gigabit Ethernet adapter. It is provided as a PCI GEM5 network device using the e1000 Linux driver.

However, the latest version of GEM5 supports this real-network device only on ARM-based architectures. In the scope of COSSIM, GEM5 has been recently modified so as to support the Intel 8254x network card for the x86 ISA.

In addition to the network interface cards, GEM5 supports networking through a simple Etherlink device. Etherlink is a virtual dummy link which emulates a cable over which Ethernet packets are sent and received without any delay (no switching or routing functionality is implemented).

In the scope of COSSIM, these limitations are unacceptably restrictive. Therefore Etherlink could not be used in its current form and thus it had been modified, while NICs supporting more protocols have been developed. Since NIC device models cannot easily be developed without specific information from their manufacturer -the Intel NIC model used in GEM5 has been contributed by Intel itself- and in order to support different physical networks, the COSSIM novel approach is to tap the Ethernet packets from Etherlink and send them to the Networking Simulator modifying at the same time the packets to match the specific network protocol required by the simulated application (i.e. Ethernet, WiFi, 3G, etc).

In order to achieve the aforementioned objectives the CERTI HLA interface has been employed. Specifically, the COSSIMlib has been integrated to the main core of the GEM5 system through Etherlink. COSSIMlib is a wrapper to an RTI Ambassador Class which is responsible for the exchange of the messages over the network with the HLA Server via TCP and UDP sockets. COSSIMlib exchanges Ethernet Packets captured from the Etherlink Device and sends (and accordingly receives) them to (from) the HLA Server. The HLA Server forwards these messages to a proper interface of the adopted Network Simulator that implements all the network related functionality.

### Supporting Parallel/Distributed Simulation

The simplistic network model of GEM5 has another serious limitation. It only supports the simulation of two identical networked systems (for example two identically configured ARM processors with exactly the same peripherals and memory configuration). Furthermore, the simulation of both systems is executed within the same thread, thus a serious performance penalty is triggered while no synchronization primitives between the two systems are provided.

By using HLA-complaint cGEM5 interfaces combined with a network simulator, the different cGEM5 instances can be efficiently connected. Each cGEM5 instance models a single node and different GEM5 instances are connected through a simulated network (more precisely through HLA links and a network simulator). As a result, all the following limitations of a conventional GEM5 simulation can be overcome:

- there are no limitations for identically configured systems.
- there are no limitations on the number of cGEM5 instances that are interconnected together; the overall system can be scaled to support as many processing nodes as required.
- parallelism can be extracted at the process level, since multiple cGEM5 instances can be executed in parallel. Furthermore, as CERTI HLA functions over IP, the different cGEM5 instances do not even need to be on the same physical machine and the overall COSSIM simulator can thus be executed efficiently in a highly distributed manner.

These changes have been proposed in the paper: 

<a id="1">[1]</a> 
N. Tampouratzis, I. Papaefstathiou, A. Nikitakis, A. Brokalakis,
S. Andrianakis, A. Dollas, M. Marcon, and E. Plebani, “A novel,
highly integrated simulator for parallel and distributed systems,”
ACM Trans. Archit. Code Optim., vol. 17, no. 1, Mar. 2020.
Available: https://dl.acm.org/doi/10.1145/3378934

## Additional Parameters

The following Table describes the additional GEM5 parameters that have to be defined in order to configure the communication between cgem5 and the network simulator as well as among different cgem5 instances. 
First of all, `--cossim` parameter declares the mode of cgem5; if it is placed, cgem5 is working in COSSIM mode, else it is working in standalone mode.
In addition, `--SynchTime` is the Global Synchronization Time which is a trade-off between simulation speed and simulation accuracy, while the `--RxPacketTime` is mostly defined by the latency of the network interface and it doesn’t constrain the simulation speed. 
Furthermore, `--nodeNum` and `--TotalNodes` are the number ID of cgem5 system and the total number of cgem5 systems in the network respectively. 
Finally, `--mcpat-xml` specifies the McPAT xml Processor Description File.
In all of the above parameters the simulated time is converted automatically to CPU ticks based on CPU frequency because each cgem5 system can simulate different types of CPUs with different clock cycles.

Parameter Name | Usage example | Description
------------ | ------------- | -------------
cossim | --cossim | Declares the cgem5 mode
SynchTime | --SynchTime=10ms | Simulated time which all cgem5 systems are synchronized periodically
RxPacketTime | --RxPacketTime=2ms | The minimum simulated time which the cgem5 system can receive Packet
nodeNum | --nodeNum=0 | The number ID of this cgem5 System
TotalNodes | --TotalNodes=2 | The total number of cgem5 Systems
mcpat-xml | --mcpat-xml=ARM_AtomicSimpleCPU_template.xml | Specifies the McPAT xml Processor Description File


## Compiling and executing cgem5

Please read the [README](README) file.

## Using cgem5 in the context of the COSSIM simulation framework

Please refer to [COSSIM _framework](https://github.com/H2020-COSSIM/COSSIM_framework) repository for all required instructions.

## Licensing

Refer to the [LICENSE](LICENSE) and [COPYING](COPYING) files included. Individual license may be present in different files in the source codes.

#### Authors

* Nikolaos Tampouratzis (tampouratzis@exapsys.eu)

Please contact for any questions.

## Acknowledgments

Code developed for the H2020-COSSIM project.

