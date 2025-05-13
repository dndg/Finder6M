#include <Finder6M.h>

#define ADDRESS 1
Finder6M f6m;

void setup()
{
    Serial.begin(9600);
    delay(1000);

    f6m.init();
}

void loop()
{
    Serial.println("** Reading 6M at address " + String(ADDRESS));

    // Voltage
    Finder6MMeasure maxVoltage = f6m.getVoltageMax100(ADDRESS);
    printMeasure("Max Voltage in hundredths", maxVoltage);
    Finder6MMeasure minVoltage = f6m.getVoltageMin100(ADDRESS);
    printMeasure("Min Voltage in hundredths", minVoltage);
    Finder6MMeasure maxVoltage100 = f6m.getVoltageMax(ADDRESS);
    printMeasure("Max Voltage", maxVoltage100);
    Finder6MMeasure minVoltage100 = f6m.getVoltageMin(ADDRESS);
    printMeasure("Min Voltage", minVoltage100);

    // Current
    Finder6MMeasure maxCurrent = f6m.getCurrentMax100(ADDRESS);
    printMeasure("Max Current in hundredths", maxCurrent);
    Finder6MMeasure minCurrent = f6m.getCurrentMin100(ADDRESS);
    printMeasure("Min Current in hundredths", minCurrent);
    Finder6MMeasure maxCurrent100 = f6m.getCurrentMax(ADDRESS);
    printMeasure("Max Current", maxCurrent100);
    Finder6MMeasure minCurrent100 = f6m.getCurrentMin(ADDRESS);
    printMeasure("Min Current", minCurrent100);

    // Active Power
    Finder6MMeasure maxActPower = f6m.getActivePowerMax100(ADDRESS);
    printMeasure("Max Active Power in hundredths", maxActPower);
    Finder6MMeasure minActPower = f6m.getActivePowerMin100(ADDRESS);
    printMeasure("Min Active Power in hundredths", minActPower);
    Finder6MMeasure maxActPower100 = f6m.getActivePowerMax(ADDRESS);
    printMeasure("Max Active Power", maxActPower100);
    Finder6MMeasure minActPower100 = f6m.getActivePowerMin(ADDRESS);
    printMeasure("Min Active Power", minActPower100);

    // Reactive Power
    Finder6MMeasure maxReacPower = f6m.getReactivePowerMax100(ADDRESS);
    printMeasure("Max Reactive Power in hundredths", maxReacPower);
    Finder6MMeasure minReacPower = f6m.getReactivePowerMin100(ADDRESS);
    printMeasure("Min Reactive Power in hundredths", minReacPower);
    Finder6MMeasure maxReacPower100 = f6m.getReactivePowerMax(ADDRESS);
    printMeasure("Max Reactive Power", maxReacPower100);
    Finder6MMeasure minReacPower100 = f6m.getReactivePowerMin(ADDRESS);
    printMeasure("Min Reactive Power", minReacPower100);

    // Apparent Power
    Finder6MMeasure maxAppPower = f6m.getApparentPowerMax100(ADDRESS);
    printMeasure("Max Apparent Power in hundredths", maxAppPower);
    Finder6MMeasure minAppPower = f6m.getApparentPowerMin100(ADDRESS);
    printMeasure("Min Apparent Power in hundredths", minAppPower);
    Finder6MMeasure maxAppPower100 = f6m.getApparentPowerMax(ADDRESS);
    printMeasure("Max Apparent Power", maxAppPower100);
    Finder6MMeasure minAppPower100 = f6m.getApparentPowerMin(ADDRESS);
    printMeasure("Min Apparent Power", minAppPower100);

    // Power Factor
    Finder6MMeasure maxPF = f6m.getPowerFactorMax100(ADDRESS);
    printMeasure("Max Power Factor in hundredths", maxPF);
    Finder6MMeasure minPF = f6m.getPowerFactorMin100(ADDRESS);
    printMeasure("Min Power Factor in hundredths", minPF);
    Finder6MMeasure maxPF100 = f6m.getPowerFactorMax(ADDRESS);
    printMeasure("Max Power Factor", maxPF100);
    Finder6MMeasure minPF100 = f6m.getPowerFactorMin(ADDRESS);
    printMeasure("Min Power Factor", minPF100);

    // Frequency
    Finder6MMeasure maxF = f6m.getFrequencyMax100(ADDRESS);
    printMeasure("Max Frequency in Hz", maxF);
    Finder6MMeasure minF = f6m.getFrequencyMin100(ADDRESS);
    printMeasure("Min Frequency in Hz", minF);
    Finder6MMeasure maxF100 = f6m.getFrequencyMax(ADDRESS);
    printMeasure("Max Frequency in Hz hundredths", maxF100);
    Finder6MMeasure minF100 = f6m.getFrequencyMin(ADDRESS);
    printMeasure("Min Frequency in Hz hundredths", minF100);

    // THD
    Finder6MMeasure maxTHD = f6m.getTHDMax100(ADDRESS);
    printMeasure("Max THD in hundredths", maxTHD);
    Finder6MMeasure minTHD = f6m.getTHDMin100(ADDRESS);
    printMeasure("Min THD in hundredths", minTHD);
    Finder6MMeasure maxTHD100 = f6m.getTHDMax(ADDRESS);
    printMeasure("Max THD", maxTHD100);
    Finder6MMeasure minTHD100 = f6m.getTHDMin(ADDRESS);
    printMeasure("Min THD", minTHD100);

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
