# PWMer
It PWMs your fans!

:warning: Do not use this project with your tachometer fan interfaces because
the low side is switched instead of the standard high-side switching! :warning:

## License
GPLv3.0 included

## Development Plan
The `zero` branch is used to develop the interfaces that are present on the
PWMer and test the base implementation of the algorithms.

`v0.1`: Basic trimpot operation

Future releases will likely include the following interfaces
* modbus
* thermistor

Future releases will likely include the following software features:
* fan curves based on temperature
* configurable LED?
