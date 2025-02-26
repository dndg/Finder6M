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

bool Finder6M::init(uint32_t baudrate, uint32_t serialParameters, uint32_t timeoutMs)
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

    if (timeoutMs > 0)
    {
        timeout = timeoutMs;
    }

    RS485.setDelays(preDelay, postDelay);
    ModbusRTUClient.setTimeout(timeout);
    return ModbusRTUClient.begin(baudrate, serialParameters) == 1;
};

bool Finder6M::getMachineId(uint8_t address, uint16_t *value)
{
    return modbus6MRead16(address, FINDER_6M_REG_MACHINE_ID, value);
};

bool Finder6M::getFirmwareVersion(uint8_t address, uint16_t *value)
{
    return modbus6MRead16(address, FINDER_6M_REG_FIRMWARE_VERSION, value);
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

Finder6MMeasure Finder6M::getTVRatio(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_TV_RATIO, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, !isOk);
};

bool Finder6M::setTVRatio(uint8_t address, float value)
{
    return modbus6MWrite32(address, FINDER_6M_REG_TV_RATIO, toUint32(value));
};

Finder6MMeasure Finder6M::getTARatio(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_TA_RATIO, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, !isOk);
};

bool Finder6M::setTARatio(uint8_t address, float value)
{
    return modbus6MWrite32(address, FINDER_6M_REG_TA_RATIO, toUint32(value));
};

Finder6MMeasure Finder6M::getVoltageRMS100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_VOLTAGE_RMS_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getVoltageMax100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_VOLTAGE_MAX_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getVoltageMin100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_VOLTAGE_MIN_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getCurrentRMS100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_CURRENT_RMS_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getCurrentMax100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_CURRENT_MAX_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getCurrentMin100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_CURRENT_MIN_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getActivePower100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_ACTIVE_POWER_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getReactivePower100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_REACTIVE_POWER_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getApparentPower100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_APPARENT_POWER_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getPowerFactor100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_POWER_FACTOR_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getFrequency100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_FREQUENCY_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getTHD100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_THD_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getEnergy100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_ENERGY_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getEnergyPositive100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_ENERGY_POSITIVE_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getEnergyNegative100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_ENERGY_NEGATIVE_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
};

Finder6MMeasure Finder6M::getInstantaneousVoltagePeak100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_INSTANTANEOUS_VOLTAGE_PEAK_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
}

Finder6MMeasure Finder6M::getInstantaneousCurrentPeak100(uint8_t address)
{
    uint32_t value;
    bool isOk = modbus6MRead32(address, FINDER_6M_REG_INSTANTANEOUS_CURRENT_PEAK_100, &value);
    return generateMeasure(value, F6M_MEASURE_TYPE_INT, !isOk);
}

Finder6MMeasure Finder6M::getVoltageRMS(uint8_t address)
{
    Finder6MMeasure m = getVoltageRMS100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getVoltageMax(uint8_t address)
{
    Finder6MMeasure m = getVoltageMax100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getVoltageMin(uint8_t address)
{
    Finder6MMeasure m = getVoltageMin100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getCurrentRMS(uint8_t address)
{
    Finder6MMeasure m = getCurrentRMS100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getCurrentMax(uint8_t address)
{
    Finder6MMeasure m = getCurrentMax100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getCurrentMin(uint8_t address)
{
    Finder6MMeasure m = getCurrentMin100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getActivePower(uint8_t address)
{
    Finder6MMeasure m = getActivePower100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getReactivePower(uint8_t address)
{
    Finder6MMeasure m = getReactivePower100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getApparentPower(uint8_t address)
{
    Finder6MMeasure m = getApparentPower100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getPowerFactor(uint8_t address)
{
    Finder6MMeasure m = getPowerFactor100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getFrequency(uint8_t address)
{
    Finder6MMeasure m = getFrequency100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getTHD(uint8_t address)
{
    Finder6MMeasure m = getTHD100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getEnergy(uint8_t address)
{
    Finder6MMeasure m = getEnergy100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getEnergyPositive(uint8_t address)
{
    Finder6MMeasure m = getEnergyPositive100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getEnergyNegative(uint8_t address)
{
    Finder6MMeasure m = getEnergyNegative100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getInstantaneousVoltagePeak(uint8_t address)
{
    Finder6MMeasure m = getInstantaneousVoltagePeak100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

Finder6MMeasure Finder6M::getInstantaneousCurrentPeak(uint8_t address)
{
    Finder6MMeasure m = getInstantaneousCurrentPeak100(address);
    uint32_t value = toUint32(m.toInt() / 100.00f);
    return generateMeasure(value, F6M_MEASURE_TYPE_FLOAT, m.isReadError());
}

bool Finder6M::getStatus(uint8_t address, uint16_t *value)
{
    return modbus6MRead16(address, FINDER_6M_REG_STATUS, value);
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

bool Finder6M::getFlagMeasurement(uint8_t address, uint16_t *value)
{
    return modbus6MRead16(address, FINDER_6M_REG_FLAG_MEASUREMENT, value);
};

bool Finder6M::measureDirectCurrent(uint8_t address)
{
    uint16_t value;
    if (!getFlagMeasurement(address, &value))
    {
        return false;
    }
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (value | 0x0001));
};

bool Finder6M::measureAlternateCurrent(uint8_t address)
{
    uint16_t value;
    if (!getFlagMeasurement(address, &value))
    {
        return false;
    }
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (value & 0xFFFE));
};

bool Finder6M::enableEnergyStoring(uint8_t address)
{
    uint16_t value;
    if (!getFlagMeasurement(address, &value))
    {
        return false;
    }
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (value | 0x0002));
};

bool Finder6M::disableEnergyStoring(uint8_t address)
{
    uint16_t value;
    if (!getFlagMeasurement(address, &value))
    {
        return false;
    }
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (value & 0xFFFD));
};

bool Finder6M::detectFrequencyOnVoltageChannel(uint8_t address)
{
    uint16_t value;
    if (!getFlagMeasurement(address, &value))
    {
        return false;
    }
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (value | 0x0004));
};

bool Finder6M::detectFrequencyOnCurrentChannel(uint8_t address)
{
    uint16_t value;
    if (!getFlagMeasurement(address, &value))
    {
        return false;
    }
    return modbus6MWrite16(address, FINDER_6M_REG_FLAG_MEASUREMENT, (value & 0xFFFB));
};

bool Finder6M::saveSettings(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_COMMAND, FINDER_6M_COMMAND_SAVE);
};

bool Finder6M::reset(uint8_t address)
{
    return modbus6MWrite16(address, FINDER_6M_REG_COMMAND, FINDER_6M_COMMAND_RESET);
};

bool Finder6M::modbus6MRead16(uint8_t address, uint16_t reg, uint16_t *value)
{
    uint32_t attempts = 3;
    while (attempts > 0)
    {
        ModbusRTUClient.requestFrom(address, HOLDING_REGISTERS, reg, 1);
        uint32_t data = ModbusRTUClient.read();
        if (data != INVALID_DATA)
        {
            *value = data;
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

bool Finder6M::modbus6MRead32(uint8_t address, uint16_t reg, uint32_t *value, bool swapped)
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
                *value = data1 << 16 | data2;
                return true;
            }
            else
            {
                *value = data2 << 16 | data1;
                return true;
            }
        }
        else
        {
            attempts -= 1;
            delay(10);
        }
    }
    return false;
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

uint32_t Finder6M::toUint32(float data)
{
    assert(sizeof(uint32_t) == sizeof(float));
    uint32_t uiData = reinterpret_cast<uint32_t &>(data);
    return uiData;
};

Finder6MMeasure Finder6M::generateMeasure(uint32_t value, uint8_t type, bool isError)
{
    if (isError)
    {
        return Finder6MMeasure(0, type, F6M_MEASURE_ERROR_INVALID_READ);
    }
    return Finder6MMeasure(value, type, F6M_MEASURE_ERROR_NONE);
};
