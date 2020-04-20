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

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

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

#define MinTemp 25
#define MaxTemp 35

static float a;
static float b;
static float c;
static float d;
static uint8_t red;
static uint8_t green;
static uint8_t blue;

static void Getabcd()
{
    a = MinTemp + (MaxTemp - MinTemp) * 0.2121;
    b = MinTemp + (MaxTemp - MinTemp) * 0.3182;
    c = MinTemp + (MaxTemp - MinTemp) * 0.4242;
    d = MinTemp + (MaxTemp - MinTemp) * 0.8182;
}

static uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static uint16_t GetColor(float val)
{

    red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

    if ((val > MinTemp) & (val < a))
    {
        green = constrain(255.0 / (a - MinTemp) * val - (255.0 * MinTemp) / (a - MinTemp), 0, 255);
    }
    else if ((val >= a) & (val <= c))
    {
        green = 255;
    }
    else if (val > c)
    {
        green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
    }
    else if ((val > d) | (val < a))
    {
        green = 0;
    }

    if (val <= b)
    {
        blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
    }
    else if ((val > b) & (val <= d))
    {
        blue = 0;
    }
    else if (val > d)
    {
        blue = constrain(240.0 / (MaxTemp - d) * val - (d * 240.0) / (MaxTemp - d), 0, 240);
    }

    // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
    return color565(red, green, blue);
}

void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color, uint8_t *data)
{
     if ((x >= 200) || (y >= 200)) {
        return;
    }

    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }

    if ((x + w) > 200) {
        w = 200  - x;
    }
    if ((y + h) > 200) {
        h = 200 - y;
    }

    if ((w < 1) || (h < 1)) {
        return;
    }
    int32_t yp = 200 * y + x;
    color = (color & 0xE000) >> 8 | (color & 0x0700) >> 6 | (color & 0x0018) >> 3;
    while (h--)
    {
        //memset(data + yp, (uint8_t)color, w);
        yp += 200;
    }
}

m_generic_make(amg8833_thermal)
{
    abstract_module_t *self = new_abstruct_module(type);
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    common_hal_amg8833_thermal_construct(self);
    return self;
}

mp_obj_t amg8833_thermal_read_pixel_temperature(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];

    if (!mp_obj_is_type(args[1], &mp_type_list))
    {
        return mp_const_none;
        mp_raise_TypeError("expected list");
    }
    mp_obj_list_t *data = MP_OBJ_TO_PTR(args[1]);

    float *buff = (float *)m_malloc(data->len * sizeof(float));

    common_hal_amg8833_thermal_read_pixel_temperature(self, buff);

    for (int i = 0; i < data->len; i++)
    {
        data->items[i] = mp_obj_new_float(buff[i]);
    }
    m_free(buff);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_read_pixel_temperature_obj, 2, 2, amg8833_thermal_read_pixel_temperature);

mp_obj_t amg8833_thermal_form_image(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];

    Getabcd();

    float *pixels = (float *)m_malloc(64 * sizeof(float));

    common_hal_amg8833_thermal_read_pixel_temperature(self, pixels);

    char *data = (char *)m_malloc(200 * 200);
    float HDTemp[80][80];
   
    for (int row = 0; row < 8; row++)
    {
        for (int col = 0; col < 70; col++)
        {
            // get the first array point, then the next
            // also need to bump by 8 for the subsequent rows
            int aLow = col / 10 + (row * 8);
            int aHigh = (col / 10) + 1 + (row * 8);
            // get the amount to interpolate for each of the 10 columns
            // here were doing simple linear interpolation mainly to keep performace high and
            // display is 5-6-5 color palet so fancy interpolation will get lost in low color depth
            float intPoint = ((pixels[aHigh] - pixels[aLow]) / 10.0);
            // determine how much to bump each column (basically 0-9)
            int incr = col % 10;
            // find the interpolated value
            float val = (intPoint * incr) + pixels[aLow];
            // store in the 70 x 70 array
            // since display is pointing away, reverse row to transpose row data
            HDTemp[(7 - row) * 10][col] = val;
        }
    }

    // now that we have raw data with 70 columns
    // interpolate each of the 70 columns
    // forget Arduino..no where near fast enough..Teensy at > 72 mhz is the starting point

    for (int col = 0; col < 70; col++)
    {
        for (int row = 0; row < 70; row++)
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
            float val = (intPoint * incr) + HDTemp[aLow][col];
            // store in the 70 x 70 array
            HDTemp[row][col] = val;
            //printf("%f, ", HDTemp[row][col]);
        }
        //printf('\n');
    }

    // for (int row = 0; row < 70; row++)
    // {
    //     for (int col = 0; col < 70; col++)
    //     {
    //         printf("%f, ", HDTemp[row][col]);
    //     }
    //     printf('\n');
    // }
    int BoxWidth = 0;
    int BoxHeight = 0;
    // rip through 70 rows
    for (int row = 0; row < 70; row++)
    {

        // fast way to draw a non-flicker grid--just make every 10 pixels 2x2 as opposed to 3x3
        // drawing lines after the grid will just flicker too much

        if ((row % 10 == 9))
        {
            BoxWidth = 2;
        }
        else
        {
            BoxWidth = 3;
        }

        // then rip through each 70 cols
        for (int col = 0; col < 70; col++)
        {

            if ((col % 10 == 9))
            {
                BoxHeight = 2;
            }
            else
            {
                BoxHeight = 3;
            }
            fillRect((row * 3) + 15, (col * 3) + 15, BoxWidth, BoxHeight, GetColor(HDTemp[row][col]), (uint8_t *)data);
        }
    }

    mp_obj_t str = mp_obj_new_bytes(data, 200 * 200);
    m_free(pixels);
    m_free(data);
    return str;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_form_image_obj, 1, 1, amg8833_thermal_form_image);

const mp_rom_map_elem_t amg8833_thermal_locals_dict_table[] = {
    // instance methods
    {MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&amg8833_thermal_deinit_obj)},
    {MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj)},
    {MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&amg8833_thermal_obj___exit___obj)},
    {MP_ROM_QSTR(MP_QSTR_read_pixel_temperature), MP_ROM_PTR(&amg8833_thermal_read_pixel_temperature_obj)},
    {MP_ROM_QSTR(MP_QSTR_form_image), MP_ROM_PTR(&amg8833_thermal_form_image_obj)},
};

MP_DEFINE_CONST_DICT(amg8833_thermal_locals_dict, amg8833_thermal_locals_dict_table);

const mp_obj_type_t grove_amg8833_thermal_type = {
    {&mp_type_type},
    .name = MP_QSTR_grove_amg8833_thermal,
    .make_new = amg8833_thermal_make_new,
    .locals_dict = (mp_obj_t)&amg8833_thermal_locals_dict
    //.attr = &tm1637_tube_obj_attr,
};
