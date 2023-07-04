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

constexpr int FINDER_6M_REG_MACHINE_ID = 0;            // Machine ID
constexpr int FINDER_6M_REG_FIRMWARE_VERSION = 1;      // Firmware version
constexpr int FINDER_6M_REG_MODBUS_ADDRESS = 2;        // Modbus Address
constexpr int FINDER_6M_REG_BAUDRATE = 4;              // Baudrate
constexpr int FINDER_6M_REG_PARITY = 5;                // Parity
constexpr int FINDER_6M_REG_FLAG_MEASUREMENT = 7;      // Flag Measurement
constexpr int FINDER_6M_REG_TV_RATIO = 8;              // Voltage transformer ratio
constexpr int FINDER_6M_REG_TA_RATIO = 10;             // Current transformer ratio
constexpr int FINDER_6M_REG_STATUS = 191;              // Status
constexpr int FINDER_6M_REG_VOLTAGE_RMS_100 = 192;     // Voltage RMS in hundredths
constexpr int FINDER_6M_REG_CURRENT_RMS_100 = 194;     // Current RMS in hundredths
constexpr int FINDER_6M_REG_ACTIVE_POWER_100 = 196;    // Active Power (P) in hundredths
constexpr int FINDER_6M_REG_REACTIVE_POWER_100 = 198;  // Reactive Power (Q) in hundredths
constexpr int FINDER_6M_REG_APPARENT_POWER_100 = 200;  // Apparent Power (S) in hundredths
constexpr int FINDER_6M_REG_POWER_FACTOR_100 = 202;    // Power factor in hundredths
constexpr int FINDER_6M_REG_FREQUENCY_100 = 204;       // Frequency in hundredths
constexpr int FINDER_6M_REG_THD_100 = 206;             // Total harmonic distortion (THD) in hundredths
constexpr int FINDER_6M_REG_ENERGY_100 = 208;          // Total Energy in hundredths
constexpr int FINDER_6M_REG_ENERGY_POSITIVE_100 = 210; // Positive Energy in hundredths
constexpr int FINDER_6M_REG_ENERGY_NEGATIVE_100 = 212; // Negative Energy in hundredths
constexpr int FINDER_6M_REG_VOLTAGE_MAX_100 = 218;     // Max Voltage RMS in hundredths
constexpr int FINDER_6M_REG_VOLTAGE_MIN_100 = 220;     // Min Voltage RMS in hundredths
constexpr int FINDER_6M_REG_CURRENT_MAX_100 = 222;     // Max Current RMS in hundredths
constexpr int FINDER_6M_REG_CURRENT_MIN_100 = 224;     // Min Current RMS in hundredths
constexpr int FINDER_6M_REG_COMMAND = 251;             // Command register

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

constexpr int FINDER_6M_COMMAND_SAVE = 0xC1C0;  // Save settings
constexpr int FINDER_6M_COMMAND_RESET = 0xC1A0; // Reset settings

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
     *
     * @return true in case of success, false otherwise.
     */
    bool init(uint32_t baudrate = 38400, uint32_t serialParameters = SERIAL_8N1);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The machine ID of the device.
     */
    uint16_t getMachineId(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The firmware version of the device.
     */
    uint16_t getFirmwareVersion(uint8_t address);
    /**
     * @param newAddress Modbus id that will be assigned to the target device.
     * @param oldAddress Current id of the target device. If none is specified
     * defaults to 1.
     *
     * @return true in case of success, false otherwise.
     */
    bool setModbusAddress(uint8_t newAddress, uint8_t oldAddress = 1);
    /**
     * @param address Modbus id of the target device.
     * @param baudrate Baudrate that will be set.
     *
     * @return true in case of success, false otherwise.
     */
    bool setBaudrate(uint8_t address, uint32_t baudrate);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool setNoParity(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool setOddParity(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool setEvenParity(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Voltage transformer ratio.
     */
    float getTVRatio(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     * @param value Value of the Voltage transformer ratio.
     *
     * @return true in case of success, false otherwise.
     */
    bool setTVRatio(uint8_t address, float value);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Current transformer ratio.
     */
    float getTARatio(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     * @param value Value of the Current transformer ratio.
     *
     * @return true in case of success, false otherwise.
     */
    bool setTARatio(uint8_t address, float value);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Voltage RMS measurement in hundredths (V/100).
     */
    int32_t getVoltageRMS100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Max Voltage RMS in hundredths (V/100).
     */
    int32_t getVoltageMax100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Min Voltage RMS in hundredths (V/100).
     */
    int32_t getVoltageMin100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Current RMS measurement in hundredths (mA/100).
     */
    int32_t getCurrentRMS100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Max Current RMS in hundredths (mA/100).
     */
    int32_t getCurrentMax100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Min Current RMS in hundredths (mA/100).
     */
    int32_t getCurrentMin100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Active Power measurement in hundredths (W/100).
     */
    int32_t getActivePower100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Reactive Power measurement in hundredths (var/100).
     */
    int32_t getReactivePower100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The Apparent Power measurement in hundredths (VA/100).
     */
    int32_t getApparentPower100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The power factor measurement in hundredths.
     */
    int32_t getPowerFactor100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The frequency measurement in hundredths (Hz/100).
     */
    int32_t getFrequency100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The total harmonic distortion (THD) measurement in hundredths.
     */
    int32_t getTHD100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The energy measurement in hundredths (KWh/100).
     */
    int32_t getEnergy100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The positive energy measurement in hundredths (KWh/100).
     */
    int32_t getEnergyPositive100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The megative energy measurement in hundredths (KWh/100).
     */
    int32_t getEnergyNegative100(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return The status of the device.
     */
    uint16_t getStatus(uint8_t address);
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
     *
     * @return The content of the Flag Measurement register of the device.
     */
    uint16_t getFlagMeasurement(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool measureDirectCurrent(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool measureAlternateCurrent(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool enableEnergyStoring(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool disableEnergyStoring(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool detectFrequencyOnVoltageChannel(uint8_t address);
    /**
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool detectFrequencyOnCurrentChannel(uint8_t address);
    /**
     * Save the current settings on the flash memory of the device.
     *
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool saveSettings(uint8_t address);
    /**
     * Reset the device completely.
     *
     * @param address Modbus id of the target device.
     *
     * @return true in case of success, false otherwise.
     */
    bool resetSettings(uint8_t address);
    /**
     * Read a 16-bits register.
     *
     * @param addr Modbus id of the target device.
     * @param reg Start address of the register.
     *
     * @return The read value or INVALID_DATA.
     */
    uint32_t modbus6MRead16(uint8_t address, uint16_t reg);
    /**
     * Read two consecutive 16-bits registers and compose them
     * into a single 32-bits value, by shifting the first value
     * left by 16 bits.
     *
     * @param addr Modbus id of the target device.
     * @param reg Start address of the register.
     * @param swapped true for MSW first, false for LSW first (default).
     *
     * @return The composed value or INVALID_DATA.
     */
    uint32_t modbus6MRead32(uint8_t address, uint16_t reg, bool swapped = false);
    /**
     * Write 8-bits or 16-bits values to a given register.
     *
     * @param address Modbus id of the target device.
     * @param reg Start address of the destination register.
     * @param toWrite Content that will be written to the destination register.
     *
     * @return true in case of success, false otherwise.
     */
    bool modbus6MWrite16(uint8_t address, uint16_t reg, uint16_t toWrite);
    /**
     * Write 32-bits values to two consecutive 16-bits registers.
     *
     * @param address Modbus id of the target device.
     * @param reg Start address of the destination register.
     * @param toWrite Content that will be written to the destination registers.
     * @param swapped true for MSW first, false for LSW first (default).
     *
     * @return true in case of success, false otherwise.
     */
    bool modbus6MWrite32(uint8_t address, uint16_t reg, uint32_t toWrite, bool swapped = false);
    /**
     * Safely convert 32-bits integer to float.
     */
    float toFloat(uint32_t data);
    /**
     * Safely convert float to 32-bits integer.
     */
    uint32_t toUint32(float data);
};

#endif
