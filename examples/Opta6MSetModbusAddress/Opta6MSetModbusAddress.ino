#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_DEFAULT_ADDRESS = 1;
constexpr uint8_t MODBUS_6M_ADDRESS = 3;

void setup()
{
    Serial.begin(9600);
    delay(2000);

    f6m.init();

    if (f6m.setModbusAddress(MODBUS_6M_ADDRESS, MODBUS_6M_DEFAULT_ADDRESS) && // Change Modbus address from default to 3.
        f6m.setBaudrate(MODBUS_6M_DEFAULT_ADDRESS, 38400) &&                  // Set baudrate to 38400
        f6m.setNoParity(MODBUS_6M_DEFAULT_ADDRESS))                           // Set no parity
    {
        Serial.print("Saving settings...");
        if (f6m.saveSettings(MODBUS_6M_DEFAULT_ADDRESS))
        {
            Serial.println("done! Set both DIP switches DOWN then click the user button.");
            while (digitalRead(BTN_USER) == HIGH)
                ;

            f6m.reset(MODBUS_6M_DEFAULT_ADDRESS);
            delay(1000); // Wait 1s after the reset

            Serial.print("On address " + String(MODBUS_6M_ADDRESS));

            uint16_t id;
            bool isOk = f6m.getMachineId(MODBUS_6M_ADDRESS, &id);
            if (isOk)
            {
                switch (id)
                {
                case FINDER_6M_MODEL_TA:
                    Serial.print(" Finder6M.TA");
                    break;
                case FINDER_6M_MODEL_TB:
                    Serial.print(" Finder6M.TF");
                    break;
                case FINDER_6M_MODEL_TF:
                    Serial.print(" Finder6M.TB");
                    break;
                }

                uint16_t fw;
                isOk = f6m.getFirmwareVersion(MODBUS_6M_ADDRESS, &fw);
                Serial.println(" with firmware version " + (isOk ? String(fw) : "...read error!"));
            }
            else
            {
                Serial.println("...read error!");
            }
        }
        else
        {
            Serial.println("error! Could not save settings.");
        }
    }
    else
    {
        Serial.println("Error! Could not change settings.");
    }
}

void loop()
{
}
