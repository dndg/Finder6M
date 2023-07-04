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

#include <ArduinoModbus.h>
#include <math.h>

#include "Finder6M.h"

bool Finder6M::init(uint32_t baudrate, uint32_t serialParameters)
{
    uint32_t preDelay, postDelay, timeout;
    float bitDuration = 1.f / baudrate;

    if (baudrate <= 19200)
    {
        preDelay = postDelay = bitDuration * 9.6f * 3.5f * 1e6;
        timeout = 200;
    }
    else
    {
        preDelay = postDelay = 1750;
        timeout = 1000;
    }

    RS485.setDelays(preDelay, postDelay);
    ModbusRTUClient.setTimeout(timeout);
    return ModbusRTUClient.begin(baudrate, serialParameters) == 1;
};

uint16_t Finder6M::getMachineId(uint8_t address)
{
    return modbus6MRead16(address, FINDER_6M_REG_MACHINE_ID);
};

uint16_t Finder6M::getFirmwareVersion(uint8_t address)
{
    return modbus6MRead16(address, FINDER_6M_REG_FIRMWARE_VERSION);
};

bool Finder6M::setModbusAddress(uint8_t newAddress, uint8_t oldAddress)
{
    return modbus6MWrite16(oldAddress, FINDER_6M_REG_MODBUS_ADDRESS, newAddress);
};

bool Finder6M::setBaudrate(uint8_t address, uint32_t baudrate)
{
    switch (baudrate)
    {
    case 1200:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_1200);
    case 2400:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_2400);
    case 4800:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_4800);
    case 9600:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_9600);
    case 19200:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_19200);
    case 38400:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_38400);
    case 57600:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_57600);
    case 115200:
        return modbus6MWrite16(address, FINDER_6M_REG_BAUDRATE, FINDER_6M_BAUDRATE_CODE_115200);
    default:
        return false;
    }
};

bool Finder6M::setNoParity(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_PARITY, FINDER_6M_PARITY_CODE_NO);
};

bool Finder6M::setOddParity(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_PARITY, FINDER_6M_PARITY_CODE_ODD);
};

bool Finder6M::setEvenParity(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_PARITY, FINDER_6M_PARITY_CODE_EVEN);
};

float Finder6M::getTVRatio(uint8_t address)
{
    return toFloat(modbus6MRead32(address, FINDER_6M_REG_TV_RATIO));
};

bool Finder6M::setTVRatio(uint8_t address, float value)
{
    return modbus6MWrite32(address, FINDER_6M_REG_TV_RATIO, toUint32(value));
};

float Finder6M::getTARatio(uint8_t address)
{
    return toFloat(modbus6MRead32(address, FINDER_6M_REG_TA_RATIO));
};

bool Finder6M::setTARatio(uint8_t address, float value)
{
    return modbus6MWrite32(address, FINDER_6M_REG_TA_RATIO, toUint32(value));
};

int32_t Finder6M::getVoltageRMS100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_VOLTAGE_RMS_100);
};

int32_t Finder6M::getVoltageMax100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_VOLTAGE_MAX_100);
};

int32_t Finder6M::getVoltageMin100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_VOLTAGE_MIN_100);
};

int32_t Finder6M::getCurrentRMS100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_CURRENT_RMS_100);
};

int32_t Finder6M::getCurrentMax100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_CURRENT_MAX_100);
};

int32_t Finder6M::getCurrentMin100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_CURRENT_MIN_100);
};

int32_t Finder6M::getActivePower100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_ACTIVE_POWER_100);
};

int32_t Finder6M::getReactivePower100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_REACTIVE_POWER_100);
};

int32_t Finder6M::getApparentPower100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_APPARENT_POWER_100);
};

int32_t Finder6M::getPowerFactor100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_POWER_FACTOR_100);
};

int32_t Finder6M::getFrequency100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_FREQUENCY_100);
};

int32_t Finder6M::getTHD100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_THD_100);
};

int32_t Finder6M::getEnergy100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_ENERGY_100);
};

int32_t Finder6M::getEnergyPositive100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_ENERGY_POSITIVE_100);
};

int32_t Finder6M::getEnergyNegative100(uint8_t address)
{
    return modbus6MRead32(address, FINDER_6M_REG_ENERGY_NEGATIVE_100);
};

uint16_t Finder6M::getStatus(uint8_t address)
{
    return modbus6MRead16(address, FINDER_6M_REG_STATUS);
};

bool Finder6M::isVoltageOverRange(uint16_t status)
{
    return (status & FINDER_6M_STATUS_BITMASK_VOLTAGE_OVER_RANGE) >> 2;
};

bool Finder6M::isVoltageUnderRange(uint16_t status)
{
    return (status & FINDER_6M_STATUS_BITMASK_VOLTAGE_UNDER_RANGE) >> 3;
};

bool Finder6M::isCurrentOverRange(uint16_t status)
{
    return (status & FINDER_6M_STATUS_BITMASK_CURRENT_OVER_RANGE) >> 13;
};

bool Finder6M::isCurrentUnderRange(uint16_t status)
{
    return (status & FINDER_6M_STATUS_BITMASK_CURRENT_UNDER_RANGE) >> 14;
};

uint16_t Finder6M::getFlagMeasurement(uint8_t address)
{
    return modbus6MRead16(address, FINDER_6M_REG_FLAG_MEASUREMENT);
};

bool Finder6M::measureDirectCurrent(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (getFlagMeasurement(address) | 0x0001));
};

bool Finder6M::measureAlternateCurrent(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (getFlagMeasurement(address) & 0xFFFE));
};

bool Finder6M::enableEnergyStoring(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (getFlagMeasurement(address) | 0x0002));
};

bool Finder6M::disableEnergyStoring(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (getFlagMeasurement(address) & 0xFFFD));
};

bool Finder6M::detectFrequencyOnVoltageChannel(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (getFlagMeasurement(address) | 0x0004));
};

bool Finder6M::detectFrequencyOnCurrentChannel(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (getFlagMeasurement(address) & 0xFFFB));
};

bool Finder6M::saveSettings(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_COMMAND, FINDER_6M_COMMAND_SAVE);
};

bool Finder6M::resetSettings(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_COMMAND, FINDER_6M_COMMAND_RESET);
};

uint32_t Finder6M::modbus6MRead16(uint8_t address, uint16_t reg)
{
    uint32_t attempts = 3;
    while (attempts > 0)
    {
        ModbusRTUClient.requestFrom(address, HOLDING_REGISTERS, reg, 1);
        uint32_t data = ModbusRTUClient.read();
        if (data != INVALID_DATA)
        {
            return data;
        }
        else
        {
            attempts -= 1;
            delay(10);
        }
    }
    return INVALID_DATA;
};

uint32_t Finder6M::modbus6MRead32(uint8_t address, uint16_t reg, bool swapped)
{
    uint8_t attempts = 3;
    while (attempts > 0)
    {
        ModbusRTUClient.requestFrom(address, HOLDING_REGISTERS, reg, 2);
        uint32_t data1 = ModbusRTUClient.read();
        uint32_t data2 = ModbusRTUClient.read();
        if (data1 != INVALID_DATA && data2 != INVALID_DATA)
        {
            if (swapped)
            {
                return data1 << 16 | data2;
            }
            else
            {
                return data2 << 16 | data1;
            }
        }
        else
        {
            attempts -= 1;
            delay(10);
        }
    }
    return INVALID_DATA;
};

bool Finder6M::modbus6MWrite16(uint8_t address, uint16_t reg, uint16_t toWrite)
{
    uint8_t attempts = 3;
    while (attempts > 0)
    {
        if (ModbusRTUClient.holdingRegisterWrite(address, reg, toWrite) == 1)
        {
            return true;
        }
        else
        {
            attempts -= 1;
            delay(10);
        }
    }
    return false;
};

bool Finder6M::modbus6MWrite32(uint8_t address, uint16_t reg, uint32_t toWrite, bool swapped)
{
    bool okWrite1, okWrite2;
    uint8_t attempts = 3;
    while (attempts > 0)
    {
        if (swapped)
        {
            okWrite1 = ModbusRTUClient.holdingRegisterWrite(address, reg, ((toWrite & 0xFFFF0000) >> 16)) == 1;
            okWrite2 = ModbusRTUClient.holdingRegisterWrite(address, reg + 1, (toWrite & 0x0000FFFF)) == 1;
        }
        else
        {
            okWrite1 = ModbusRTUClient.holdingRegisterWrite(address, reg, (toWrite & 0x0000FFFF)) == 1;
            okWrite2 = ModbusRTUClient.holdingRegisterWrite(address, reg + 1, ((toWrite & 0xFFFF0000) >> 16)) == 1;
        }
        if (okWrite1 && okWrite2)
        {
            return true;
        }
        else
        {
            attempts -= 1;
            delay(10);
        }
    }
    return false;
};

float Finder6M::toFloat(uint32_t data)
{
    assert(sizeof(uint32_t) == sizeof(float));
    float fData = reinterpret_cast<float &>(data);
    return fData;
};

uint32_t Finder6M::toUint32(float data)
{
    assert(sizeof(uint32_t) == sizeof(float));
    uint32_t uiData = reinterpret_cast<uint32_t &>(data);
    return uiData;
};
