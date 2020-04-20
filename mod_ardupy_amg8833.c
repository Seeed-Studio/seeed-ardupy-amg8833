/**
 * The MIT License (MIT)
 *
 * Author: Baozhu Zuo (zuobaozhu@gmail.com)
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

#include "py/mphal.h"
#include "py/nlr.h"
#include "py/objtype.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "shared-bindings/util.h"

void common_hal_amg8833_thermal_construct(abstract_module_t *self);
void common_hal_amg8833_thermal_deinit(abstract_module_t *self);
int common_hal_amg8833_thermal_set_upper_limit(abstract_module_t *self, uint8_t *value);
int common_hal_amg8833_thermal_reset_flags(abstract_module_t *self, uint8_t value);
int common_hal_amg8833_thermal_set_lower_limit(abstract_module_t *self, uint8_t *value);
int common_hal_amg8833_thermal_set_sensor_mode(abstract_module_t *self, uint8_t mode);
int common_hal_amg8833_thermal_set_hysteresis(abstract_module_t *self, uint8_t *value);
int common_hal_amg8833_thermal_clear_status(abstract_module_t *self, uint8_t value);
int common_hal_amg8833_thermal_set_interrupt_mode(abstract_module_t *self, uint8_t mode);
int common_hal_amg8833_thermal_frame_rate(abstract_module_t *self, uint8_t rate);
int common_hal_amg8833_thermal_read_pixel_temperature(abstract_module_t *self, float *pixel_date);
int common_hal_amg8833_thermal_get_interrupt_status(abstract_module_t *self);
int common_hal_amg8833_thermal_read_pixels_interrupt_status(abstract_module_t *self, uint8_t *status);
int common_hal_amg8833_thermal_read_pixel_temperature_reg_value(abstract_module_t *self, uint16_t *value);

extern const mp_obj_type_t grove_amg8833_thermal_type;

m_generic_make(amg8833_thermal)
{
    abstract_module_t *self = new_abstruct_module(type);
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    common_hal_amg8833_thermal_construct(self);
    return self;
}

#define array_index(scale, row, col) ((row) * (scale * 8) + (col))

mp_obj_t amg8833_thermal_read_pixel_temperature(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];

    if (!mp_obj_is_type(args[1], &mp_type_list))
    {
        mp_raise_TypeError("expected list");
        return mp_const_none;
    }

    mp_obj_list_t *data = MP_OBJ_TO_PTR(args[1]);
    uint32_t scale = 1;
    if (n_args == 3)
    {
        scale = mp_obj_get_int(pos_args[2]);
    }

    if (scale < 1)
    {
        mp_raise_TypeError("scale must more than zero.");
        return mp_const_none;
    }

    if ((scale * scale * 64) != data.len)
    {
        mp_raise_TypeError("64 data (8 * 8) are returned by default. Your current scale is %d, so you need to enter an array of %d size", scale, (scale * scale * 64));
        return mp_const_none;
    }
    float *buff = (float *)m_malloc(data->len * sizeof(float));

    common_hal_amg8833_thermal_read_pixel_temperature(self, buff);

    if (scale == 1)
    {
        for (int i = 0; i < data->len; i++)
        {
            data->items[i] = mp_obj_new_float(buff[i]);
        }
        m_free(buff);
    }

    // now that we have an 8 x 8 sensor array
    // interpolate to get a bigger screen
    // interpolate the 8 rows (interpolate the 70 column points between the 8 sensor pixels first)
    for (int row = 0; row < 8; row++)
    {
        for (int col = 0; col < scale * 8; col++)
        {
            // get the first array point, then the next
            // also need to bump by 8 for the subsequent rows
            int aLow = col / 10 + (row * 8);
            int aHigh = (col / 10) + 1 + (row * 8);
            // get the amount to interpolate for each of the 10 columns
            // here were doing simple linear interpolation mainly to keep performace high and
            // display is 5-6-5 color palet so fancy interpolation will get lost in low color depth
            float intPoint = ((buff[aHigh] - buff[aLow]) / 10.0);
            // determine how much to bump each column (basically 0-9)
            int incr = col % 10;
            // find the interpolated value
            float val = (intPoint * incr) + buff[aLow];
            // store in the 70 x 70 array
            // since display is pointing away, reverse row to transpose row data
            int index = array_index(scale, (7 - row) * 10， col);
            data[index] = val;
        }
    }

    // now that we have raw data with 70 columns
    // interpolate each of the 70 columns
    // forget Arduino..no where near fast enough..Teensy at > 72 mhz is the starting point

    for (int col = 0; col < scale * 8; col++)
    {
        for (int row = 0; row < scale * 8; row++)
        {
            // get the first array point, then the next
            // also need to bump by 8 for the subsequent cols
            int aLow = (row / 10) * 10;
            int aHigh = aLow + 10;
            // get the amount to interpolate for each of the 10 columns
            // here were doing simple linear interpolation mainly to keep performace high and
            // display is 5-6-5 color palet so fancy interpolation will get lost in low color depth
            float intPoint = ((HDTemp[aHigh][col] - HDTemp[aLow][col]) / 10.0);
            // determine how much to bump each column (basically 0-9)
            int incr = row % 10;
            // find the interpolated value
            int val = (intPoint * incr) + HDTemp[aLow][col];
            int index = array_index(scale, row， col);
            // store in the 70 x 70 array
            data[index] = val;
        }
    }

    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_read_pixel_temperature_obj, 2, 3, amg8833_thermal_read_pixel_temperature);

mp_obj_t amg8833_thermal_Scale(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    uint32_t from = mp_obj_get_int(pos_args[1]);
    uint32_t to = mp_obj_get_int(pos_args[2]);

    if (!mp_obj_is_type(args[3], &mp_type_list))
    {
        return mp_const_none;
        mp_raise_TypeError("expected list");
    }
    if (!mp_obj_is_type(args[4], &mp_type_list))
    {
        return mp_const_none;
        mp_raise_TypeError("expected list");
    }
    mp_obj_list_t *s_data = MP_OBJ_TO_PTR(args[3]);
    mp_obj_list_t *d_data = MP_OBJ_TO_PTR(args[3]);

    float *buff = (float *)m_malloc(data->len * sizeof(float));

    return mp_const_none;
}

const mp_rom_map_elem_t amg8833_thermal_locals_dict_table[] = {
    // instance methods
    {MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&amg8833_thermal_deinit_obj)},
    {MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj)},
    {MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&amg8833_thermal_obj___exit___obj)},
    {MP_ROM_QSTR(MP_QSTR_read_pixel_temperature), MP_ROM_PTR(&amg8833_thermal_read_pixel_temperature_obj)},
};

MP_DEFINE_CONST_DICT(amg8833_thermal_locals_dict, amg8833_thermal_locals_dict_table);

const mp_obj_type_t grove_amg8833_thermal_type = {
    {&mp_type_type},
    .name = MP_QSTR_grove_amg8833_thermal,
    .make_new = amg8833_thermal_make_new,
    .locals_dict = (mp_obj_t)&amg8833_thermal_locals_dict
    //.attr = &tm1637_tube_obj_attr,
};