/**
 * This file is part of Finder 6M for Finder Opta.
 *
 * Finder 6M for Finder Opta is free software: you can redistribute it and/or modify it under the terms
 * of the GNU Lesser General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * Finder 6M for Finder Opta is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with Foobar.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _FINDER_6_M_H_INCLUDED
#define _FINDER_6_M_H_INCLUDED

#include <ArduinoRS485.h>
#include "Finder6MMeasure.h"

constexpr int FINDER_6M_REG_MACHINE_ID = 0;                       // Machine ID
constexpr int FINDER_6M_REG_FIRMWARE_VERSION = 1;                 // Firmware version
constexpr int FINDER_6M_REG_MODBUS_ADDRESS = 2;                   // Modbus Address
constexpr int FINDER_6M_REG_BAUDRATE = 4;                         // Baudrate
constexpr int FINDER_6M_REG_PARITY = 5;                           // Parity
constexpr int FINDER_6M_REG_DC_FILTER = 6;                        // DC Filter
constexpr int FINDER_6M_REG_FLAG_MEASUREMENT = 7;                 // Flag Measurement
constexpr int FINDER_6M_REG_TV_RATIO = 8;                         // Voltage transformer ratio
constexpr int FINDER_6M_REG_TA_RATIO = 10;                        // Current transformer ratio
constexpr int FINDER_6M_REG_CURRENT_AND_POWER_CUT_OFF = 12;       // Current and Power cut off (Current in LSB, Power in MSB)
constexpr int FINDER_6M_REG_STATUS = 191;                         // Status
constexpr int FINDER_6M_REG_VOLTAGE_RMS_100 = 192;                // Voltage RMS in hundredths
constexpr int FINDER_6M_REG_CURRENT_RMS_100 = 194;                // Current RMS in hundredths
constexpr int FINDER_6M_REG_ACTIVE_POWER_100 = 196;               // Active Power (P) in hundredths
constexpr int FINDER_6M_REG_REACTIVE_POWER_100 = 198;             // Reactive Power (Q) in hundredths
constexpr int FINDER_6M_REG_APPARENT_POWER_100 = 200;             // Apparent Power (S) in hundredths
constexpr int FINDER_6M_REG_POWER_FACTOR_100 = 202;               // Power Factor in hundredths
constexpr int FINDER_6M_REG_FREQUENCY_100 = 204;                  // Frequency in hundredths
constexpr int FINDER_6M_REG_THD_100 = 206;                        // Total Harmonic Distortion (THD) in hundredths
constexpr int FINDER_6M_REG_ENERGY_100 = 208;                     // Total Energy in hundredths
constexpr int FINDER_6M_REG_ENERGY_POSITIVE_100 = 210;            // Positive Energy in hundredths
constexpr int FINDER_6M_REG_ENERGY_NEGATIVE_100 = 212;            // Negative Energy in hundredths
constexpr int FINDER_6M_REG_INSTANTANEOUS_VOLTAGE_PEAK_100 = 214; // Instantaneous Voltage Peak in hundredths
constexpr int FINDER_6M_REG_INSTANTANEOUS_CURRENT_PEAK_100 = 216; // Instantaneous Current Peak in hundredths
constexpr int FINDER_6M_REG_VOLTAGE_MAX_100 = 218;                // Max Voltage RMS in hundredths
constexpr int FINDER_6M_REG_VOLTAGE_MIN_100 = 220;                // Min Voltage RMS in hundredths
constexpr int FINDER_6M_REG_CURRENT_MAX_100 = 222;                // Max Current RMS in hundredths
constexpr int FINDER_6M_REG_CURRENT_MIN_100 = 224;                // Min Current RMS in hundredths
constexpr int FINDER_6M_REG_ACTIVE_POWER_MAX_100 = 226;           // Max Active Power (P) in hundredths
constexpr int FINDER_6M_REG_ACTIVE_POWER_MIN_100 = 228;           // Min Active Power (P) in hundredths
constexpr int FINDER_6M_REG_REACTIVE_POWER_MAX_100 = 230;         // Max Reactive Power (Q) in hundredths
constexpr int FINDER_6M_REG_REACTIVE_POWER_MIN_100 = 232;         // Min Reactive Power (Q) in hundredths
constexpr int FINDER_6M_REG_APPARENT_POWER_MAX_100 = 234;         // Max Apparent Power (S) in hundredths
constexpr int FINDER_6M_REG_APPARENT_POWER_MIN_100 = 236;         // Min Apparent Power (S) in hundredths
constexpr int FINDER_6M_REG_POWER_FACTOR_MAX_100 = 238;           // Max Power Factor in hundredths
constexpr int FINDER_6M_REG_POWER_FACTOR_MIN_100 = 240;           // Min Power Factor in hundredths
constexpr int FINDER_6M_REG_FREQUENCY_MAX_100 = 242;              // Max Frequency in hundredths
constexpr int FINDER_6M_REG_FREQUENCY_MIN_100 = 244;              // Min Frequency in hundredths
constexpr int FINDER_6M_REG_THD_MAX_100 = 246;                    // Max Total Harmonic Distortion (THD) in hundredths
constexpr int FINDER_6M_REG_THD_MIN_100 = 248;                    // Min Total Harmonic Distortion (THD) in hundredths
constexpr int FINDER_6M_REG_COMMAND = 251;                        // Command register
constexpr int FINDER_6M_REG_COMMAND_AUX = 252;                    // Auxiliary register for load commands

constexpr int FINDER_6M_BAUDRATE_CODE_1200 = 0;   // Indicates Baudarate of 1200
constexpr int FINDER_6M_BAUDRATE_CODE_2400 = 1;   // Indicates Baudarate of 2400
constexpr int FINDER_6M_BAUDRATE_CODE_4800 = 2;   // Indicates Baudarate of 4800
constexpr int FINDER_6M_BAUDRATE_CODE_9600 = 3;   // Indicates Baudarate of 9600
constexpr int FINDER_6M_BAUDRATE_CODE_19200 = 4;  // Indicates Baudarate of 19200
constexpr int FINDER_6M_BAUDRATE_CODE_38400 = 5;  // Indicates Baudarate of 38400
constexpr int FINDER_6M_BAUDRATE_CODE_57600 = 6;  // Indicates Baudarate of 57600
constexpr int FINDER_6M_BAUDRATE_CODE_115200 = 7; // Indicates Baudarate of 115200

constexpr int FINDER_6M_PARITY_CODE_NO = 0;   // Indicates no Parity
constexpr int FINDER_6M_PARITY_CODE_ODD = 1;  // Indicates odd Parity
constexpr int FINDER_6M_PARITY_CODE_EVEN = 2; // Indicates even Parity

constexpr int FINDER_6M_COMMAND_SAVE = 0xC1C0;                 // Save settings
constexpr int FINDER_6M_COMMAND_RESET = 0xC1A0;                // Reset device
constexpr int FINDER_6M_COMMAND_LOAD_ENERGY = 0xBABA;          // Load energy stored in the auxiliary register
constexpr int FINDER_6M_COMMAND_LOAD_POSITIVE_ENERGY = 0xBABB; // Load positive energy stored in the auxiliary register
constexpr int FINDER_6M_COMMAND_LOAD_NEGATIVE_ENERGY = 0xBABC; // Load negative energy stored in the auxiliary register

constexpr int FINDER_6M_STATUS_BITMASK_FLASH_SETTINGS_ERROR = 0x0001;    // Bitmask for flash settings error bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_FLASH_CALIBRATION_ERROR = 0x0002; // Bitmask for flash calibration error bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_VOLTAGE_OVER_RANGE = 0x0004;      // Bitmask for Voltage over range bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_VOLTAGE_UNDER_RANGE = 0x0008;     // Bitmask for Voltage under range bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_ZERO_CROSSING_DETECTING = 0x0040; // Bitmask for zero crossing detecting bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_ENERGY_STORING_ERROR = 0x0400;    // Bitmask for energy storing error bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_ENERGY_INIT_ERROR = 0x0800;       // Bitmask for energy initialization error bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_CURRENT_OVER_RANGE = 0x2000;      // Bitmask for Current over range bit in Status
constexpr int FINDER_6M_STATUS_BITMASK_CURRENT_UNDER_RANGE = 0x4000;     // Bitmask for Current under range bit in Status

#define INVALID_DATA 0xFFFFFFFF

enum Finder6MModel : uint16_t
{
    FINDER_6M_MODEL_TA = 7,
    FINDER_6M_MODEL_TB = 18,
    FINDER_6M_MODEL_TF = 48,
};

class Finder6M
{
public:
    Finder6M() {}
    /**
     * Set preDelay and postDelay and start the Modbus RTU client
     * with the parameters for the Finder 6M.
     *
     * @param baudrate Defaults to 38400, if not specified.
     * @param serialParameters Defaults to 8N1, if not specified.
     * @param timeoutMs Timeout in ms, if not specified the function
     * will internally use the best value according to Modbus specs.
     *
     * @return true in case of success, false otherwise.
     */
    bool init(uint32_t baudrate = 38400, uint32_t serialParameters = SERIAL_8N1, uint32_t timeoutMs = 0);
    /**
     * Set the Modbus RTU client timeout.
     *
     * @param timeoutMs Timeout in ms.
     */
    void setTimeout(uint32_t timeoutMs);
    /**
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the output value.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getMachineId(uint8_t address, uint16_t *value, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the output value.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getFirmwareVersion(uint8_t address, uint16_t *value, uint8_t attempts = 3);
    /**
     * @param newAddress Modbus id that will be assigned to the target device.
     * @param oldAddress Current id of the target device. If none is specified
     * defaults to 1.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setModbusAddress(uint8_t newAddress, uint8_t oldAddress = 1, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param baudrate Baudrate that will be set.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setBaudrate(uint8_t address, uint32_t baudrate, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setNoParity(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setOddParity(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setEvenParity(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the Voltage
     * transformer ratio.
     */
    Finder6MMeasure getTVRatio(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param value Value of the Voltage transformer ratio.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setTVRatio(uint8_t address, float value, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the Current
     * transformer ratio.
     */
    Finder6MMeasure getTARatio(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param value Value of the Current transformer ratio.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setTARatio(uint8_t address, float value, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Voltage RMS measurement in V hundredths.
     */
    Finder6MMeasure getVoltageRMS100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max Voltage RMS in V hundredths.
     */
    Finder6MMeasure getVoltageMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Voltage RMS in V hundredths.
     */
    Finder6MMeasure getVoltageMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Current RMS measurement in mA hundredths.
     */
    Finder6MMeasure getCurrentRMS100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Max Current RMS in mA hundredths.
     */
    Finder6MMeasure getCurrentMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Current RMS in mA hundredths.
     */
    Finder6MMeasure getCurrentMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Active Power measurement in W hundredths.
     */
    Finder6MMeasure getActivePower100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max Active Power in W hundredths.
     */
    Finder6MMeasure getActivePowerMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Active Power in W hundredths.
     */
    Finder6MMeasure getActivePowerMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Reactive Power measurement in var hundredths.
     */
    Finder6MMeasure getReactivePower100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max Reactive Power in var hundredths.
     */
    Finder6MMeasure getReactivePowerMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Reactive Power in var hundredths.
     */
    Finder6MMeasure getReactivePowerMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Apparent Power measurement in VA hundredths.
     */
    Finder6MMeasure getApparentPower100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max Apparent Power in VA hundredths.
     */
    Finder6MMeasure getApparentPowerMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Apparent Power in VA hundredths.
     */
    Finder6MMeasure getApparentPowerMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * power factor measurement in hundredths.
     */
    Finder6MMeasure getPowerFactor100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max Power Factor in hundredths.
     */
    Finder6MMeasure getPowerFactorMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Power Factor in hundredths.
     */
    Finder6MMeasure getPowerFactorMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * frequency measurement in Hz hundredths.
     */
    Finder6MMeasure getFrequency100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max Frequency in Hz hundredths.
     */
    Finder6MMeasure getFrequencyMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min Frequency in Hz hundredths.
     */
    Finder6MMeasure getFrequencyMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * total harmonic distortion (THD) measurement in hundredths.
     */
    Finder6MMeasure getTHD100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     *
     * @return A Finder 6MMeasure containing the
     * Max THD in hundredths.
     */
    Finder6MMeasure getTHDMax100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * Min THD in hundredths.
     */
    Finder6MMeasure getTHDMin100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * energy measurement in KWh hundredths.
     */
    Finder6MMeasure getEnergy100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * positive energy measurement in KWh hundredths.
     */
    Finder6MMeasure getEnergyPositive100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * negative energy measurement in KWh hundredths.
     */
    Finder6MMeasure getEnergyNegative100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * instantaneous voltage peak in V hundredths.
     */
    Finder6MMeasure getInstantaneousVoltagePeak100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder 6MMeasure containing the
     * instantaneous current peak in mA hundredths.
     */
    Finder6MMeasure getInstantaneousCurrentPeak100(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Voltage RMS measurement (V).
     */
    Finder6MMeasure getVoltageRMS(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max Voltage RMS (V).
     */
    Finder6MMeasure getVoltageMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min Voltage RMS (V).
     */
    Finder6MMeasure getVoltageMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Current RMS measurement (mA).
     */
    Finder6MMeasure getCurrentRMS(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max Current RMS (mA).
     */
    Finder6MMeasure getCurrentMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min Current RMS (mA).
     */
    Finder6MMeasure getCurrentMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Active Power measurement (W).
     */
    Finder6MMeasure getActivePower(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max Active Power (W).
     */
    Finder6MMeasure getActivePowerMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min Active Power (W).
     */
    Finder6MMeasure getActivePowerMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Reactive Power measurement (var).
     */
    Finder6MMeasure getReactivePower(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max Reactive Power (var).
     */
    Finder6MMeasure getReactivePowerMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min Reactive Power (var).
     */
    Finder6MMeasure getReactivePowerMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Apparent Power measurement (VA).
     */
    Finder6MMeasure getApparentPower(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max Apparent Power (VA).
     */
    Finder6MMeasure getApparentPowerMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min Apparent Power (VA).
     */
    Finder6MMeasure getApparentPowerMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * power factor measurement.
     */
    Finder6MMeasure getPowerFactor(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max Power Factor.
     */
    Finder6MMeasure getPowerFactorMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min Power Factor.
     */
    Finder6MMeasure getPowerFactorMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * frequency measurement (Hz).
     */
    Finder6MMeasure getFrequency(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Max frequency (Hz).
     */
    Finder6MMeasure getFrequencyMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * Min frequency (Hz).
     */
    Finder6MMeasure getFrequencyMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * total harmonic distortion (THD) measurement.
     */
    Finder6MMeasure getTHD(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the Max THD.
     */
    Finder6MMeasure getTHDMax(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the Min THD.
     */
    Finder6MMeasure getTHDMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * energy measurement (KWh).
     */
    Finder6MMeasure getEnergy(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * positive energy measurement (KWh).
     */
    Finder6MMeasure getEnergyPositive(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * negative energy measurement (KWh).
     */
    Finder6MMeasure getEnergyNegative(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * instantaneous voltage peak (V).
     */
    Finder6MMeasure getInstantaneousVoltagePeak(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return A Finder6MMeasure containing the
     * instantaneous current peak (mA).
     */
    Finder6MMeasure getInstantaneousCurrentPeak(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Voltage Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearVoltageMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Voltage Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearVoltageMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set instantaneous voltage peak to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearInstantaneousVoltagePeak(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Current Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearCurrentMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Current Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearCurrentMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set instantaneous current peak to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearInstantaneousCurrentPeak(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Active Power Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearActivePowerMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Active Power Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearActivePowerMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Reactive Power Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearReactivePowerMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Reactive Power Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearReactivePowerMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Apparent Power Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearApparentPowerMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Apparent Power Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearApparentPowerMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Power Factor Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearPowerFactorMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Power Factor Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearPowerFactorMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Frequency Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearFrequencyMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set Frequency Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearFrequencyMin(uint8_t address, uint8_t attempts = 3);
    /**
     * Set THD Max to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearTHDMax(uint8_t address, uint8_t attempts = 3);
    /**
     * Set THD Min to 0.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool clearTHDMin(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the output value.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getStatus(uint8_t address, uint16_t *value, uint8_t attempts = 3);
    /**
     * @param status Obtained with getStatus.
     *
     * @return true if voltage is over range, false otherwise.
     */
    bool isVoltageOverRange(uint16_t status);
    /**
     * @param status Obtained with getStatus.
     *
     * @return true if voltage is under range, false otherwise.
     */
    bool isVoltageUnderRange(uint16_t status);
    /**
     * @param status Obtained with getStatus.
     *
     * @return true if current is over range, false otherwise.
     */
    bool isCurrentOverRange(uint16_t status);
    /**
     * @param status Obtained with getStatus.
     *
     * @return true if current is under range, false otherwise.
     */
    bool isCurrentUnderRange(uint16_t status);
    /**
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the output value.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getFlagMeasurement(uint8_t address, uint16_t *value, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool measureDirectCurrent(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool measureAlternateCurrent(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool enableEnergyStoring(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool disableEnergyStoring(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool detectFrequencyOnVoltageChannel(uint8_t address, uint8_t attempts = 3);
    /**
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool detectFrequencyOnCurrentChannel(uint8_t address, uint8_t attempts = 3);
    /**
     * Set the current cutoff, the minimum threshold under which
     * current is treated as zero. The function adjusts the value
     * based on the device model.
     *
     * @param address Modbus id of the target device.
     * @param value Current value in mA.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setCurrentCutoff(uint8_t address, uint16_t value, uint8_t attempts = 3);
    /**
     * Set the power cutoff, the minimum threshold under which
     * power is treated as zero. The function adjusts the value
     * based on the device model.
     *
     * @param address Modbus id of the target device.
     * @param value Power value in W.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool setPowerCutoff(uint8_t address, uint16_t value, uint8_t attempts = 3);
    /**
     * Get the current cutoff, the minimum threshold under which
     * current is treated as zero. The function adjusts the value
     * based on the device model.
     *
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the
     * output value in mA.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getCurrentCutoff(uint8_t address, uint16_t *value, uint8_t attempts = 3);
    /**
     * Get the power cutoff, the minimum threshold under which
     * power is treated as zero. The function adjusts the value
     * based on the device model.
     *
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the
     * output value in W.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getPowerCutoff(uint8_t address, uint16_t *value, uint8_t attempts = 3);
    /**
     * Get current and power cutoff bytes.
     *
     * @param address Modbus id of the target device.
     * @param value Pointer to the variable that will store the output bytes.
     * @param scaleFactor Pointer to the variable that will store the scale
     * factor to convert the bytes into mA or W.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool getCurrentAndPowerCutoff(uint8_t address, uint16_t *value, uint16_t *scaleFactor, uint8_t attempts = 3);
    /**
     * Save the current settings on the flash memory of the device.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool saveSettings(uint8_t address, uint8_t attempts = 3);
    /**
     * Reboot the device.
     *
     * @param address Modbus id of the target device.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool reset(uint8_t address, uint8_t attempts = 3);
    /**
     * Read a 16-bits register.
     *
     * @param addr Modbus id of the target device.
     * @param reg Start address of the register.
     * @param value Pointer to the variable that will store the output value.
     * @param attempts Number of attempts before returning error.
     *
     * @return True in case of success, false in case of error.
     */
    bool modbus6MRead16(uint8_t address, uint16_t reg, uint16_t *value, uint8_t attempts = 3);
    /**
     * Read two consecutive 16-bits registers and compose them
     * into a single 32-bits value, by shifting the first value
     * left by 16 bits.
     *
     * @param addr Modbus id of the target device.
     * @param reg Start address of the register.
     * @param value Pointer to the variable that will store the output value.
     * @param swapped true for MSW first, false for LSW first (default).
     * @param attempts Number of attempts before returning error.
     *
     * @return True in case of success, false in case of error.
     */
    bool modbus6MRead32(uint8_t address, uint16_t reg, uint32_t *value, bool swapped = false, uint8_t attempts = 3);
    /**
     * Write 8-bits or 16-bits values to a given register.
     *
     * @param address Modbus id of the target device.
     * @param reg Start address of the destination register.
     * @param toWrite Content that will be written to the destination register.
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool modbus6MWrite16(uint8_t address, uint16_t reg, uint16_t toWrite, uint8_t attempts = 3);
    /**
     * Write 32-bits values to two consecutive 16-bits registers.
     *
     * @param address Modbus id of the target device.
     * @param reg Start address of the destination register.
     * @param toWrite Content that will be written to the destination registers.
     * @param swapped true for MSW first, false for LSW first (default).
     * @param attempts Number of attempts before returning error.
     *
     * @return true in case of success, false otherwise.
     */
    bool modbus6MWrite32(uint8_t address, uint16_t reg, uint32_t toWrite, bool swapped = false, uint8_t attempts = 3);
    /**
     * Safely convert float to 32-bits integer.
     */
    uint32_t toUint32(float data);

private:
    /**
     * @param value As unsigned 32-bits integer.
     * @param type A type of Finder6MMeasure.
     *
     * @return Performs validation and returns a Finder6MMeasure.
     *
     * @warning The Measure contains an error code that indicates
     * the result of the validation.
     */
    Finder6MMeasure generateMeasure(uint32_t value, uint8_t type, bool isError);
};

#endif
