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

### Versions

As signaled by the major version jump, version `2.0` of this library is a major
update. This version adds the new type `Finder6MMeasure`, returned by most of
the functions of the library.

When updating from version `1.x` to version `2.x`, it will likely be necessary
to update the code of the sketches using this library. In particular, most
functions now return either:

* `bool`: to notify the result of the read to the calling code. The read value
  is stored in a variable, whose pointer was passed as parameter to the called
  function.
* `Finder6MMeasure`: this new class gives access to:
  * The read value as `float`, using the function `toFloat()`.
  * The read value as `int32_t`, using the function `toInt()`.
  * The result of the read as `bool`, using the function `isError()`.

All [the example sketches](./examples/) use the latest version of the library,
offering guidance on how to use the new `Finder6MMeasure` class.

The full list of changed functions is available in the changelog of the version
of the library `2.0`, so that you can easily verify if your sketch needs
changes and how to update it. <!-- TODO: add link -->

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
