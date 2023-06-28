# JSD Changelog

Semantic Versioning will be observed on JSD. The basic rules apply:

### MAJOR version updates 
- WILL break application builds
- May have modifications to prexisting public interface functions (*_pub.h functions)
- May have modification to preexisting public types (*_types.h structs and enums)

### MINOR version updates
- WILL NOT break application builds
- May introduce new functions to public interface
- May introduce additional enums or struct fields in public types
- May add new EtherCat Slaves

### PATCH version updates
- WILL NOT break application builds
- May include bug fixes or minor revisions

## v01.03.00

---

* EGD: Increased extrapolation cycles from 1 to 5 to improve robustness to latency jitter
* EGD: Added smooth_factor config parameter
* EGD: Changed drive quickstop handling response from 6 to 2



## v01.02.04
- Improved EGD EMCY handling, stdout now shows reason for drive disabling faults

## v01.02.03
- Fixed a EL3602 voltage calculation that resulted in voltages being 1/2 of actual

## v01.02.02
- Add a 1 sec reset derate protection feature that inhibits excessively fast resets which is suspected to have failed a relay on brake driver circuit

## v01.02.01
- Calling mailbox receive on all slaves to fetch EMCY codes was hindering performance greatly and unnecessary

## v01.02.00
- Added JPL Ethercat Device (JED) Module support
- Added feature to reattempt the Safeop to Op transition during initialization to improve likelihood of successful initialization

## v01.01.00
- Added EL3356 Module support with onboard tare function and user scaling

## v01.00.02
- refactored controlword interface to avoid using dereferenced pointers in packed structs

## v01.00.01
- Cleaned up a number of compliler warnings
- fixed potential egd set digital output bug

## v01.00.00
#### Major Changes
- First major release consisting of EGD, EL2124, EL3602, and EL3208 device drivers
#### Minor Changes
- N/A 
#### Patch Changes
- N/A
