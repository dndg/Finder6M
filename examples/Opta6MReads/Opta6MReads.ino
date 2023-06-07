#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_ADDRESS = 3;

void setup()
{
    Serial.begin(38400);

    if (!f6m.init())
    {
        while (1)
        {
        }
    }
    f6m.measureAlternateCurrent(MODBUS_6M_ADDRESS);
    f6m.disableEnergyStoring(MODBUS_6M_ADDRESS);
    if (!f6m.saveSettings(MODBUS_6M_ADDRESS))
    {
        while (1)
            ;
    }
}

void loop()
{
    Serial.println("** Reading 6M at address " + String(MODBUS_6M_ADDRESS));

    // Voltage and Current transformer ratio
    float tvRatio = f6m.getTVRatio(MODBUS_6M_ADDRESS);
    printFloat("Voltage transformer ratio", tvRatio);
    float taRatio = f6m.getTARatio(MODBUS_6M_ADDRESS);
    printFloat("Current transformer ratio", taRatio);

    // Energy measurements in hundredths
    int32_t energy100 = f6m.getEnergy100(MODBUS_6M_ADDRESS);
    printInt("Energy in kWh/100", energy100);
    int32_t posEnergy100 = f6m.getEnergyPositive100(MODBUS_6M_ADDRESS);
    printInt("Positive energy in kWh/100", posEnergy100);
    int32_t negEnergy100 = f6m.getEnergyNegative100(MODBUS_6M_ADDRESS);
    printInt("Negative energy in kWh/100", negEnergy100);

    // Other measurements in hundredths
    int32_t voltageRMS100 = f6m.getVoltageRMS100(MODBUS_6M_ADDRESS);
    printInt("Voltage RMS in V/100", voltageRMS100);
    int32_t currentRMS100 = f6m.getCurrentRMS100(MODBUS_6M_ADDRESS);
    printInt("Current RMS in mA/100", currentRMS100);
    int32_t activePower100 = f6m.getActivePower100(MODBUS_6M_ADDRESS);
    printInt("Active Power in W/100", activePower100);
    int32_t reactivePower100 = f6m.getReactivePower100(MODBUS_6M_ADDRESS);
    printInt("Reactive Power in var/100", reactivePower100);
    int32_t apparentPower100 = f6m.getApparentPower100(MODBUS_6M_ADDRESS);
    printInt("Apparent Power in VA/100", apparentPower100);
    int32_t powerFactor100 = f6m.getPowerFactor100(MODBUS_6M_ADDRESS);
    printInt("Power factor in hundredths", powerFactor100);
    int32_t frequency100 = f6m.getFrequency100(MODBUS_6M_ADDRESS);
    printInt("Frequency in Hz/100", frequency100);
    int32_t thd100 = f6m.getTHD100(MODBUS_6M_ADDRESS);
    printInt("THD in hundredths", thd100);

    // Status
    uint16_t status = f6m.getStatus(MODBUS_6M_ADDRESS);
    printStatus(status);
}

void printFloat(String label, float n)
{
    Serial.println("   " + label + " = " + (f6m.toUint32(n) != INVALID_DATA ? String(n) : String("read error!")));
};

void printInt(String label, int32_t n)
{
    Serial.println("   " + label + " = " + (n != INVALID_DATA ? String(n) : String("read error!")));
};

void printStatus(uint16_t status)
{
    Serial.println("   Status: ");
    Serial.println("      flash settings error : " + String(status & FINDER_6M_STATUS_BITMASK_FLASH_SETTINGS_ERROR));
    Serial.println("      flash calibration error : " + String((status & FINDER_6M_STATUS_BITMASK_FLASH_CALIBRATION_ERROR) >> 1));
    Serial.println("      voltage over range : " + String(f6m.isVoltageOverRange(status)));
    Serial.println("      voltage under range : " + String(f6m.isVoltageUnderRange(status)));
    Serial.println("      zero crossing detecting : " + String((status & FINDER_6M_STATUS_BITMASK_ZERO_CROSSING_DETECTING) >> 6));
    Serial.println("      energy storing error : " + String((status & FINDER_6M_STATUS_BITMASK_ENERGY_STORING_ERROR) >> 10));
    Serial.println("      energy initialization error : " + String((status & FINDER_6M_STATUS_BITMASK_ENERGY_INIT_ERROR) >> 11));
    Serial.println("      current over range : " + String(f6m.isCurrentOverRange(status)));
    Serial.println("      current under range : " + String(f6m.isCurrentUnderRange(status)));
};
