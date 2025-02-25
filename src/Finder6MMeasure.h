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

#ifndef _FINDER_6M_MEASURE_H_INCLUDED
#define _FINDER_6M_MEASURE_H_INCLUDED

#include <stdint.h>

#define F6M_MEASURE_TYPE_INT 0x01
#define F6M_MEASURE_TYPE_FLOAT 0x02

#define F6M_MEASURE_ERROR_NONE 0x00
#define F6M_MEASURE_ERROR_INVALID_READ 0x01

class Finder6MMeasure
{
public:
    Finder6MMeasure(uint32_t value, uint8_t type, uint8_t error)
    {
        _value = value;
        _type = type;
        _error = error;
    };

    /**
     * Get the measured value.
     *
     * @return The measured value as int32_t.
     */
    int32_t toInt();
    /**
     * Get the measured value.
     *
     * @return The measured value as float.
     */
    float toFloat();
    /**
     * Get the type of measure.
     *
     * @return The type in which value is stored. Possible values:
     * 1 = If the bytes represent a int32_t.
     * 2 = If the bytes represent a float.
     */
    uint8_t getType();
    /**
     * Check if there was an error during the measure.
     *
     * @return True if there was an error while reading this measure.
     */
    bool isReadError();
    /**
     * Get the error code of a measure.
     *
     * @return The error code. Possible values:
     * 0 = No error.
     * 1 = Generic read error.
     */
    uint8_t getErrorCode();

private:
    uint32_t _value;
    uint8_t _type;
    uint8_t _error;
};

#endif
