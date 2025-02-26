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
#include "Finder6MMeasure.h"

#include <cassert>

int32_t Finder6MMeasure::toInt()
{
    if (_type == F6M_MEASURE_TYPE_INT)
    {
        return _value;
    }

    if (_type == F6M_MEASURE_TYPE_FLOAT)
    {
        return static_cast<int32_t>(toFloat());
    }

    return 0;
};

float Finder6MMeasure::toFloat()
{
    if (_type == F6M_MEASURE_TYPE_INT)
    {
        int32_t v = reinterpret_cast<int32_t &>(_value);
        return static_cast<float>(v);
    }

    if (_type == F6M_MEASURE_TYPE_FLOAT)
    {
        assert(sizeof(uint32_t) == sizeof(float));
        return reinterpret_cast<float &>(_value);
    }

    return 0;
};

uint8_t Finder6MMeasure::getType()
{
    return _type;
};

bool Finder6MMeasure::isReadError()
{
    return (_error & F6M_MEASURE_ERROR_INVALID_READ) > 0;
};

uint8_t Finder6MMeasure::Finder6MMeasure::getErrorCode()
{
    return _error;
};
