#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_DEFAULT_ADDRESS = 1;
constexpr uint8_t MODBUS_6M_ADDRESS = 3;

void setup()
{
    Serial.begin(38400);

    if (!f6m.init())
    {
        while (1)
            ;
    }

    delay(2000);
    /**
     * Change Modbus address from default to 3.
     * Set baudrate to 38400 and no parity bit as per default value of init function.
     */
    if (f6m.setModbusAddress(MODBUS_6M_ADDRESS, MODBUS_6M_DEFAULT_ADDRESS) &&
        f6m.setBaudrate(MODBUS_6M_DEFAULT_ADDRESS, 38400) &&
        f6m.setNoParity(MODBUS_6M_DEFAULT_ADDRESS))
    {
        Serial.println("Saving settings...");
        // Save all the above settings
        if (f6m.saveSettings(MODBUS_6M_DEFAULT_ADDRESS))
        {
            Serial.println("Waiting 10s while you:");
            Serial.println("1. Power OFF the 6M.");
            Serial.println("2. Set both DIP switches DOWN.");
            Serial.println("3. Power back ON the 6M.");
            delay(10000);
        }
        else
        {
            Serial.println("Error! Could not save settings.");
            while (1)
                ;
        }
    }
    else
    {
        Serial.println("Error! Could not change settings.");
        while (1)
            ;
    }
}

void loop()
{
    Serial.print("On address " + String(MODBUS_6M_ADDRESS) + " Finder 6M.");
    switch (f6m.getMachineId(MODBUS_6M_ADDRESS))
    {
    case 7:
        Serial.print("TA");
        break;
    case 18:
        Serial.print("TF");
        break;
    case 48:
        Serial.print("TB");
        break;
    }
    Serial.println(" with firmware version " + String(f6m.getFirmwareVersion(MODBUS_6M_ADDRESS)));

    while (1)
        ;
};
