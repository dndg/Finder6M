#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_ADDRESS = 1;

void setup()
{
    Serial.begin(38400);

    if (!f6m.init())
    {
        while (1)
            ;
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

    // Energy measurements
    double energy = f6m.getEnergy(MODBUS_6M_ADDRESS);
    printMeasurement("Energy in kWh", energy);
    double posEnergy = f6m.getEnergyPositive(MODBUS_6M_ADDRESS);
    printMeasurement("Positive energy in kWh", posEnergy);
    double negEnergy = f6m.getEnergyNegative(MODBUS_6M_ADDRESS);
    printMeasurement("Negative energy in kWh", negEnergy);

    // Other measurements
    double voltageRMS = f6m.getVoltageRMS(MODBUS_6M_ADDRESS);
    printMeasurement("Voltage RMS in V", voltageRMS);
    double currentRMS = f6m.getCurrentRMS(MODBUS_6M_ADDRESS);
    printMeasurement("Current RMS in mA", currentRMS);
    double activePower = f6m.getActivePower(MODBUS_6M_ADDRESS);
    printMeasurement("Active Power in W", activePower);
    double reactivePower = f6m.getReactivePower(MODBUS_6M_ADDRESS);
    printMeasurement("Reactive Power in var", reactivePower);
    double apparentPower = f6m.getApparentPower(MODBUS_6M_ADDRESS);
    printMeasurement("Apparent Power in VA", apparentPower);
    double powerFactor = f6m.getPowerFactor(MODBUS_6M_ADDRESS);
    printMeasurement("Power factor", powerFactor);
    double frequency = f6m.getFrequency(MODBUS_6M_ADDRESS);
    printMeasurement("Frequency in Hz", frequency);
    double thd = f6m.getTHD(MODBUS_6M_ADDRESS);
    printMeasurement("THD", thd);

    // Status
    uint16_t status = f6m.getStatus(MODBUS_6M_ADDRESS);
    printStatus(status);
}

void printFloat(String label, float n)
{
    Serial.println("   " + label + " = " + (f6m.toUint32(n) != INVALID_DATA ? String(n) : String("read error!")));
};

void printMeasurement(String label, double n)
{
    Serial.println("   " + label + " = " + (n != (INVALID_DATA / 100.0) ? String(n) : String("read error!")));
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
