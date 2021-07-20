# jsd
Just 'SOEM' Drivers - C drivers for Simple Open Ethercat Master(SOEM) devices

# Prerequisites
JSD has only be tested only on Ubuntu 16.04, 18.04, 20.04 and Raspberry Pi. The use of PThreads invalidates JSD on some platforms that SOEM can support. The JSD maintainers do not have non-posix platforms in-scope for JSD, but feel free to open an issue and let your voice be heard.

Required:

* SOEM 

- CMake 3.11 or later
- Posix threads

Optional:

```bash
$ sudo apt install libreadline-dev # only used for test programs
```

# Building

To build jsd from source:
```bash
$ git clone git@github.com:nasa-jpl/jsd.git
$ cd jsd
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Tests

The following commands will execute the unit tests:

```bash
$ cd build
$ cmake -DBUILD_JSD_TESTS=ON ..
$ make
$ make test
$ make memcheck  # note valgrind is required to perform memory checking
```

# Troubleshooting

See the `TROUBLESHOOTING.md` file for a soon-to-be growing list of tips and tricks to help you through common issues. Feel free to open an issue with you problem description.

# Supported Devices

Many devices have been used over the years within JPL, some more useful than others or fallen 
out of favor over time. Three tiers of devices are defined to focus JSD development, prioritizing the 
most useful devices. The following table represents the latest state of the JSD library:

| Device                    | Tier | Minimum JSD Version Required |
| ------------------------- | ---- | ---------------------------- |
| EL2124                    | 1    | 1.0.0                        |
| EL3208                    | 1    | 1.0.0                        |
| EL3602                    | 1    | 1.0.0                        |
| Elmo Gold Drives          | 1    | 1.0.0                        |
| EL3356                    | 2    | 1.1.0                        |
| JED (JPL EtherCat Device) | 2    | 1.2.0                        |
| ATI Force-Torque Sensor   | 2    | 1.4.0                        |
| EL1008                    | 2    | TBD                          |
| EL3202-0010               | 2    | 1.5.0                        |
| EL3255                    | 2    | TBD                          |
| EL3318                    | 2    | 1.5.0                        |
| EL3202                    | 2    | TBD                          |
| EL3104                    | 2    | 1.5.0                        |
| EL2624                    | 3    | By Request Only              |
| EL2809                    | 3    | By Request Only              |
| EL3008                    | 3    | By Request Only              |
| EL3058                    | 3    | By Request Only              |
| EL5101                    | 3    | By Request Only              |
| EL6001                    | 3    | By Request Only              |

# Using JSD in your Project 
For the software package to utilize JSD, an alternate way is to fetch JSD while building using FetchContent.
For the case, include the following to your CMakeLists.txt

```cmake
include(FetchContent)
FetchContent_Declare(jsd
    GIT_REPOSITORY git@github.com:nasa-jpl/jsd.git
    GIT_TAG v1.4.0
    )
FetchContent_MakeAvailable(jsd)
```
It is always recommend you specify your jsd dependency to a tagged release (`GIT_TAG v1.4.0`) so updates to master cannot break your build (NOT `GIT_TAG master`). 

### Semantic Versioning

JSD uses Semantic versioning to help applications reason about the software as updates are continuously rolled out. Tailored to JSD, the Semver rules are as follows:

* Major Versions will denote changes to the API, which may break some user applications.
* Minor Versions will denote new features or driver additions that do not break user applications.
* Patch Versions will denote bug fixes or minor improvements and will not break user applications.

Violations of these rules will be considered errors and should be patched immediately. Please open an issue if you find a violation.

# JSD Utilities

## jsd_slaveinfo

A slave introspection tool is provided by JSD. This tool is offered in a version of SOEM but we are leveraging a mode of SOEM that avoids global definitions so this tool was migrated to JSD. 

The most useful invocation is shown here.

```
$ sudo ./build/jsd_slaveinfo <NIC_Device_Name> -map
```

The -map option lists the PDO mapping active on the device. When ELMO Gold 
Drives are powered on they have the default PDO mapping. The following example 
simply has a single ELMO device on the EtherCAT bus.
``` 
$ sudo ./build/jsd_slaveinfo eth9 -map

jsd (Simple Open EtherCAT Master)
Slaveinfo
Starting slaveinfo
ec_init on eth9 succeeded.
ec_config_init 0
1 slaves found and configured.

...

Slave:1
 Name:? M:0000009a I:00030924
 Output size: 64bits
 Input size: 0bits
 State: 4
 Delay: 0[ns]
 Has DC: 1
 DCParentport:0
 Activeports:1.0.0.0
 Configured address: 1001
 Man: 0000009a ID: 00030924 Rev: 00010400
 SM0 A:1800 L: 140 F:00010026 Type:1
 SM1 A:1900 L: 140 F:00010022 Type:2
 SM2 A:1100 L:   8 F:00010064 Type:3
 SM3 A:1180 L:  32 F:00010020 Type:0
 FMMU0 Ls:00000000 Ll:   8 Lsb:0 Leb:7 Ps:1100 Psb:0 Ty:02 Act:01
 FMMUfunc 0:1 1:2 2:0 3:0
 MBX length wr: 140 rd: 140 MBX protocols : 0e
 CoE details: 3f FoE details: 01 EoE details: 01 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
PDO mapping according to CoE :
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x607A:0x00 0x20 INTEGER32    Target position
  [0x0004.0] 0x60FE:0x01 0x20 UNSIGNED32   Physical outputs
  [0x0008.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0xFF1C6338.0] 0x6064:0x00 0x20 INTEGER32    Position actual value
  [0xFF1C633C.0] 0x60FD:0x00 0x20 UNSIGNED32   Digital inputs
  [0xFF1C6340.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
End slaveinfo, close socket
[DEBUG] (/home/dev/src/jsd/src/jsd.c:190) Closing SOEM socket connection...
End program
 
```

## jsd_egd_tlc_tty

The Elmo Gold Drive Two-Letter Command TTY is a simple terminal application that performs SDO read and writes
through the Elmo Command Reference parameters. This is meant to mimic the function of the 
Elmo Application Studio (EAS) TTY to help reduce dependence on Windows computers running EAS. 

The utility uses the FSF Readline library to parse user inputs. This library can be installed with the following:

```bash
# Tested on Ubuntu 16.04
$ sudo apt install libreadline-dev
```
If this system library cannot be found, the build will ignore this program and continue making the jsd library. 

```bash
$ sudo ./bin/jsd_egd_tlc_tty 
[ ERROR ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:112) Expecting exactly 2 arguments
[ INFO  ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:113) Usage: jsd_egd_tlc_tty <ifname> <egd_slave_index> 
[ INFO  ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:114) Example: $ jsd_egd_tlc_tty eth0 2

$ sudo ./bin/jsd_egd_tlc_tty eth0 6

...

[ DEBUG ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:37) user input is: 
[ WARN  ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:96) Could not parse user input, n_parsed: -1
[ INFO  ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:97) Please input commands of the form: float CL[2] = 1.5
[ INFO  ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:98) Or query current parameters using the form: float CL[2]
[ INFO  ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:99) type q or quit (or ctrl+c) to end program


>> float PL[2] = 2.22

[ DEBUG ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:37) user input is: float PL[2] = 2.22
[ DEBUG ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:71)          parsed val_type: float
[ DEBUG ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:72)          parsed tlc: PL
[ DEBUG ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:73)          parsed subindex: 2
[ DEBUG ](/home/dev/src/jsd/tools/jsd_egd_tlc_tty.c:74)          parsed val_str: 2.22
[ DEBUG ](/home/dev/src/jsd/src/jsd_egd.c:443) Converted TLC: PL to index 0x3191
>> [ INFO  ](/home/dev/src/jsd/src/jsd_sdo.c:115) Slave[6] Write 0x3191:2 (F32) = 2.220000

```

# Device Test Programs

Single device test programs are provided with JSD, used for isolated driver development. Test programs are built by default but they can be excluded from the build using the BUILD_JSD_TESTS CMake option. 

```bash
# in the build directory
$ cmake .. -DBUILD_JSD_TESTS=OFF
$ make
...
```

Single device tests are created for each of the supported devices mainly for driver development on dedicated 'playground' electronics. Each program is created to stress test different parts of the driver so it is **not recommended** you run these tests on arbitrary hardware but only on test articles. Typically, pure input sensing devices tests **should** be safe to run but output devices (like EL2124 and EGDs) **will** output voltages and command actuators in a potentially unsafe way. 

Either way, these programs serve as a minimum working example and allow driver developers to test features and fix bugs.

For example, The EL3602 test can be queried and called according to the following commands:

```bash
$ ./jsd_el3602_test -h

[ERROR] (/home/dev/src/jsd/test/device/jsd_el3602_test.c:63) Expecting exactly 3 arguments
[INFO] (/home/dev/src/jsd/test/device/jsd_el3602_test.c:64) Usage: jsd_el3602_test <ifname> <el3602_slave_index> <loop_freq_hz>
[INFO] (/home/dev/src/jsd/test/device/jsd_el3602_test.c:65) Example: $ jsd_el3602_test eth0 2 1000

$ sudo ./jsd_el3602_test eth9 5 2000
```

# Documentation

The API documentation can be built using the source code locally using doxygen. 

The .doxygen.in file is converted to .doxygen during build to ensure semantic versioning number of the documentation is accurate. The output documentation is created in the directory 'doxygen_html' and can be opened by any web browser from the root index.html webpage.

```bash
# Install dependencies for Ubuntu 14.04, 16.04, and 18.04 
$ sudo apt install doxygen graphviz

# use the build system to generate the code for you!
$ cd build
$ make doc
```

# Contributing

* Do all development on a new branch prefixed with <your-username>-* branched from  "master" e.g. ("USER-my-widget")
  * All enums, structs, and functions should be prefixed with `jsd_` as necessary
  * Abstract the details of your driver to help users of JSD
    * Expose as little as possible in device_pub.h public API
    * Hide as much as you can in the private device.h header
  * Document your work 
    * DO write DOXYGEN preambles to functions, structs, and enums
    * DO Explaining the enums and fixed values used by your driver
    * DO Include references to manufacturer provided documents
    * DON'T write excessively long inline comments in functions (target single line if possible)
      * Consider refactoring if you find yourself wanting to write paragraphs
    * Strive for brevity after all, "comments aren't compiled"
  * run clang-format before you submit a pull request
* Once you have completed your work, Create a pull request to the "master" branch.
* The "master" branch will be occasionally be given a tagged release as required
  * semantic versioning will updated to "master" and the git commit will be tagged
  
    


# Code Format

before applying a commit, please run clang-format on all files using the default Google style. This can be achieved by running `make format` in the build directory that uses a CMake command to invoke the shell script from the project root directory. This is the prefered over the shell script since it ensures the working directory is properly set.
```
$ make format
```

It is highly recommended you integrate clang-format into your text editor of choice. [Clang Format Integration](https://clang.llvm.org/docs/ClangFormat.html)
