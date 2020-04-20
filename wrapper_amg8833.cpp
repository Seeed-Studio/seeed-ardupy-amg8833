/**
 * The MIT License (MIT)
 *
 * Author: Hongtai.Liu (lht856@foxmail.com)
 *
 * Copyright (C) 2020  Seeed Technology Co.,Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "Arduino.h"
#define private public
#include "Seeed_AMG8833/Seeed_AMG8833_driver.h"
extern "C"
{
#include "py/objtype.h"
#include "shared-bindings/util.h"
}

#define Thermal (*(AMG8833*)self->module)
void *operator new(size_t, void *);

extern "C"
{
    void common_hal_amg8833_thermal_construct(abstract_module_t *self)
    {
        self->module = new (m_new_obj(AMG8833)) AMG8833();
        Thermal.init();
    }
    void common_hal_amg8833_thermal_deinit(abstract_module_t *self)
    {
        Thermal.~AMG8833();
    }
    int common_hal_amg8833_thermal_set_upper_limit(abstract_module_t *self, uint8_t *value)
    {
        return Thermal.set_upper_limit(value);
    }
    int common_hal_amg8833_thermal_reset_flags(abstract_module_t *self, uint8_t value)
    {
        return Thermal.reset_flags(value);
    }
    int common_hal_amg8833_thermal_set_lower_limit(abstract_module_t *self, uint8_t *value)
    {
        return Thermal.set_lower_limit(value);
    }
    int common_hal_amg8833_thermal_set_sensor_mode(abstract_module_t *self, uint8_t mode)
    {
        return Thermal.set_sensor_mode(mode);
    }
    int common_hal_amg8833_thermal_set_hysteresis(abstract_module_t *self, uint8_t *value)
    {
        return Thermal.set_hysteresis(value);
    }
    int common_hal_amg8833_thermal_clear_status(abstract_module_t *self, uint8_t value)
    {
        return Thermal.clear_status(value);
    }
    int common_hal_amg8833_thermal_set_interrupt_mode(abstract_module_t *self, uint8_t mode)
    {
        return Thermal.clear_status(mode);
    }
    int common_hal_amg8833_thermal_frame_rate(abstract_module_t *self, uint8_t rate)
    {
        return Thermal.set_frame_rate(rate);
    }
    int common_hal_amg8833_thermal_read_pixel_temperature(abstract_module_t *self, float *pixel_date)
    {
        return Thermal.read_pixel_temperature(pixel_date);
    }
    int common_hal_amg8833_thermal_get_interrupt_status(abstract_module_t *self)
    {
        return Thermal.get_interrupt_status();
    }
    int common_hal_amg8833_thermal_read_pixels_interrupt_status(abstract_module_t *self, uint8_t *status)
    {
        return Thermal.read_pixels_interrupt_status(status);
    }

    int common_hal_amg8833_thermal_read_pixel_temperature_reg_value(abstract_module_t *self, uint16_t *value)
    {
        return Thermal.read_pixel_temperature_reg_value(value);
    }
}