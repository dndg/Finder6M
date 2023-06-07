# Finder 6M for Finder Opta library

This library allows to easily read counters from Finder 6M devices connected via ModBus to a Finder Opta, providing a number of
built-in functions that simplify the process of reading measurements.

The library also includes functions to directly read registers via ModBus and perform conversions. Additionally, using this library
it is possible to write to Holding Registers and to configure some of the settings of the Finder 6M.

## Usage

The code below shows a basic example of how to use this library:

```cpp
#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_ADDRESS = 3;

void setup()
{
    Serial.begin(38400);
    if (!f7m.init())
    {
        while (1)
        {
        }
    }
}

void loop()
{
    int32_t energy100 = f6m.getEnergy100(MODBUS_6M_ADDRESS);
    Serial.println("Energy in kWh/100 " + String(energy100));
}
```

For more details take a look at [the example sketches](./examples/) provided with the library.

### Notes

When using this library keep in mind that:

* The default baudrate is `38400`.
* The default configuration is `8-N-1`.
* When writing to registers you should always write one register at a time.
* ModBus addressing on the Finder 6M starts from 0, so for example ModBus address 40006 must be accessed from the library as Holding
Register number 5.

## Resources

* [Getting started with Opta](https://opta.findernet.com/en/tutorial/getting-started).
* [Finder 6M.Tx User Guide](https://cdn.findernet.com/app/uploads/6M.Tx-User-Guide.pdf).
* [Finder 6M.Tx Modbus RS485](https://cdn.findernet.com/app/uploads/Modbus_RS485_6MTx.pdf).
* [ModBus over Serial Line](https://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf).

## License

This library is released under the GNU Lesser General Public License license. For more details read [the full license](./LICENSE.txt).

## Contact

For communication reach out to <iot@dndg.it>.
