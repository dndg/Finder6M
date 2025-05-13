#include <Finder6M.h>

Finder6M f6m;
constexpr uint8_t MODBUS_6M_ADDRESS = 1;

void setup()
{
    Serial.begin(9600);
    delay(1000);

    f6m.init();

    f6m.measureAlternateCurrent(MODBUS_6M_ADDRESS);
    f6m.saveSettings(MODBUS_6M_ADDRESS);
}

void loop()
{
    Serial.println("** Reading 6M at address " + String(MODBUS_6M_ADDRESS));

    // Voltage and Current transformer ratio
    Finder6MMeasure tvRatio = f6m.getTVRatio(MODBUS_6M_ADDRESS);
    printMeasure("Voltage transformer ratio", tvRatio);
    Finder6MMeasure taRatio = f6m.getTARatio(MODBUS_6M_ADDRESS);
    printMeasure("Current transformer ratio", taRatio);

    // Energy measurements
    Finder6MMeasure energy = f6m.getEnergy(MODBUS_6M_ADDRESS);
    printMeasure("Energy in kWh", energy);
    Finder6MMeasure energy100 = f6m.getEnergy100(MODBUS_6M_ADDRESS);
    printMeasure("Energy in kWh hundredths", energy100);
    Finder6MMeasure posEnergy = f6m.getEnergyPositive(MODBUS_6M_ADDRESS);
    printMeasure("Positive energy in kWh", posEnergy);
    Finder6MMeasure posEnergy100 = f6m.getEnergyPositive100(MODBUS_6M_ADDRESS);
    printMeasure("Positive energy in kWh hundredths", posEnergy100);
    Finder6MMeasure negEnergy = f6m.getEnergyNegative(MODBUS_6M_ADDRESS);
    printMeasure("Negative energy in kWh", negEnergy);
    Finder6MMeasure negEnergy100 = f6m.getEnergyNegative100(MODBUS_6M_ADDRESS);
    printMeasure("Negative energy in kWh hundredths", negEnergy100);

    // Other measurements
    Finder6MMeasure voltageRMS = f6m.getVoltageRMS(MODBUS_6M_ADDRESS);
    printMeasure("Voltage RMS in V", voltageRMS);
    Finder6MMeasure voltageRMS100 = f6m.getVoltageRMS100(MODBUS_6M_ADDRESS);
    printMeasure("Voltage RMS in V hundredths", voltageRMS100);
    Finder6MMeasure currentRMS = f6m.getCurrentRMS(MODBUS_6M_ADDRESS);
    printMeasure("Current RMS in mA", currentRMS);
    Finder6MMeasure currentRMS100 = f6m.getCurrentRMS100(MODBUS_6M_ADDRESS);
    printMeasure("Current RMS in mA hundredths", currentRMS100);
    Finder6MMeasure activePower = f6m.getActivePower(MODBUS_6M_ADDRESS);
    printMeasure("Active Power in W", activePower);
    Finder6MMeasure activePower100 = f6m.getActivePower100(MODBUS_6M_ADDRESS);
    printMeasure("Active Power in W hundredths", activePower100);
    Finder6MMeasure reactivePower = f6m.getReactivePower(MODBUS_6M_ADDRESS);
    printMeasure("Reactive Power in var", reactivePower);
    Finder6MMeasure reactivePower100 = f6m.getReactivePower100(MODBUS_6M_ADDRESS);
    printMeasure("Reactive Power in var hundredths", reactivePower100);
    Finder6MMeasure apparentPower = f6m.getApparentPower(MODBUS_6M_ADDRESS);
    printMeasure("Apparent Power in VA", apparentPower);
    Finder6MMeasure apparentPower100 = f6m.getApparentPower100(MODBUS_6M_ADDRESS);
    printMeasure("Apparent Power in VA hundredths", apparentPower100);
    Finder6MMeasure powerFactor = f6m.getPowerFactor(MODBUS_6M_ADDRESS);
    printMeasure("Power factor", powerFactor);
    Finder6MMeasure powerFactor100 = f6m.getPowerFactor100(MODBUS_6M_ADDRESS);
    printMeasure("Power factor in hundredths", powerFactor100);
    Finder6MMeasure frequency = f6m.getFrequency(MODBUS_6M_ADDRESS);
    printMeasure("Frequency in Hz", frequency);
    Finder6MMeasure frequency100 = f6m.getFrequency100(MODBUS_6M_ADDRESS);
    printMeasure("Frequency in Hz hundredths", frequency100);
    Finder6MMeasure thd = f6m.getTHD(MODBUS_6M_ADDRESS);
    printMeasure("THD", thd);
    Finder6MMeasure thd100 = f6m.getTHD100(MODBUS_6M_ADDRESS);
    printMeasure("THD in hundredths", thd100);

    // Status
    uint16_t status;
    bool res = f6m.getStatus(MODBUS_6M_ADDRESS, &status);
    printStatus(status);

    delay(5000);
}

void printMeasure(String label, Finder6MMeasure m)
{
    Serial.print("   " + label + " = ");
    if (m.isReadError())
    {
        Serial.println("read error!");
        return;
    }
    Serial.println("(float: " + String(m.toFloat()) + ", int: " + String(m.toInt()) + ")");
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

void stop()
{
    while (1)
        ;
}
