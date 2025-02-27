# Finder 6M for Finder Opta library

This library allows to easily read counters from Finder 6M devices connected
via ModBus to a Finder Opta, providing a number of built-in functions that
simplify the process of reading measurements.

The library also includes functions to directly read registers via ModBus and
perform conversions. Additionally, using this library it is possible to write
to Holding Registers and to configure some of the settings of the Finder 6M.

## Usage

The code below shows a basic example of how to use this library:

```cpp
#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_ADDRESS = 1;

void setup()
{
    Serial.begin(38400);
    f6m.init();
}

void loop()
{
    Finder6MMeasure energy = f6m.getEnergy(MODBUS_6M_ADDRESS);
    Serial.println("Energy in kWh: " + String(energy.toFloat()));
}
```

For more details take a look at [the example sketches](./examples/) provided
with the library.

### Notes

When using this library keep in mind that:

* The default baudrate is `38400`.
* The default configuration is `8-N-1`.
* When writing to registers you should always write one register at a time.
* ModBus addressing on the Finder 6M starts from `0`, so for example ModBus
address `40006` must be accessed from the library as Holding Register number
`5`.

## Breaking version update

As signaled by the major version jump, version `2.0` of this library is a major
update. This version introduces the new type `Finder6MMeasure`, now returned by
most of the functions of the library that previously returned `int32_t`,
`float` or `double` values.

The new class `Finder6MMeasure` allows you to:

* Get the read value as `float`, using the function `toFloat()`.
* Get the read value as `int32_t`, using the function `toInt()`.
* Get the result of the read as `bool`, using the function `isReadError()`.

Additionally, some functions that previously returned `uint16_t` now return a
`bool` and take a `uint16_t *` parameter.

This means that when you update from version `1.x` to version `2.x`, you will
likely need to update your sketches to reflect these changes. Below is the full
list of the changed functions:

* `getMachineId`, `getFirmwareVersion`, `getStatus`, `getFlagMeasurement`:
  * Changed return type from `uint16_t` to `bool`. Returns `true` on success,
    `false` otherwise.
  * Added parameter `value` of type `uint16_t *`. Pointer to the variable where
    the read value will be stored.
* `getTARatio`, `getTVRatio`:
  * Changed return type from `float` to `Finder6MMeasure`.
* `getVoltageRMS100` , `getVoltageMax100` , `getVoltageMin100` ,
  `getCurrentRMS100` , `getCurrentMax100` , `getCurrentMin100` ,
  `getActivePower100` , `getReactivePower100` , `getApparentPower100` ,
  `getPowerFactor100` , `getFrequency100` , `getTHD100` , `getEnergy100` ,
  `getEnergyPositive100` , `getEnergyNegative100` ,
  `getInstantaneousVoltagePeak100` , `getInstantaneousCurrentPeak100`:
  * Changed return type from `int32_t` to `Finder6MMeasure`.
* `getVoltageRMS` , `getVoltageMax` , `getVoltageMin` ,
  `getCurrentRMS` , `getCurrentMax` , `getCurrentMin` ,
  `getActivePower` , `getReactivePower` , `getApparentPower` ,
  `getPowerFactor` , `getFrequency` , `getTHD` , `getEnergy` ,
  `getEnergyPositive` , `getEnergyNegative` ,
  `getInstantaneousVoltagePeak` , `getInstantaneousCurrentPeak`:
  * Changed return type from `double` to `Finder6MMeasure`.
* `modbus6MRead16`:
  * Changed return type from `uint32_t` to `bool`. Returns `true` on success,
    `false` otherwise.
  * Added parameter `value` of type `uint16_t *`. Pointer to the variable where
    the read value will be stored.
* `modbus6MRead32`:
  * Changed return type from `uint32_t` to `bool`. Returns `true` on success,
    `false` otherwise.
  * Added parameter `value` of type `uint32_t *`. Pointer to the variable where
    the read value will be stored.

All of [the example sketches](./examples/) use the latest version of the
library, offering guidance on how to update your code.

## Resources

* [Getting started with
  Opta](https://opta.findernet.com/en/tutorial/getting-started).
* [Finder 6M.Tx User
  Guide](https://cdn.findernet.com/app/uploads/6M.Tx-User-Guide.pdf).
* [Finder 6M.Tx Modbus
  RS485](https://cdn.findernet.com/app/uploads/Modbus_RS485_6MTx.pdf).
* [ModBus over Serial
  Line](https://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf).

## License

This library is released under the GNU Lesser General Public License license.
For more details read [the full license](./LICENSE.txt).

## Contact

For communication reach out to <iot@dndg.it>.
