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
int common_hal_amg8833_thermal_read_pixel_temperature_scale(abstract_module_t *self, float *pixel_date, int width, int height);
bool common_hal_amg8833_thermal_is_connect(abstract_module_t *self);

extern const mp_obj_type_t grove_amg8833_thermal_type;

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols,
                       float *dest, uint8_t dest_rows, uint8_t dest_cols);

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y)
{
    if (x < 0)
    {
        x = 0;
    }
    if (y < 0)
    {
        y = 0;
    }
    if (x >= cols)
    {
        x = cols - 1;
    }
    if (y >= rows)
    {
        y = rows - 1;
    }
    return p[y * cols + x];
}

void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f)
{
    if ((x < 0) || (x >= cols))
    {
        return;
    }
    if ((y < 0) || (y >= rows))
    {
        return;
    }
    p[y * cols + x] = f;
}

// src is a grid src_rows * src_cols
// dest is a pre-allocated grid, dest_rows*dest_cols
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols,
                       float *dest, uint8_t dest_rows, uint8_t dest_cols)
{
    float mu_x = (src_cols - 1.0) / (dest_cols - 1.0);
    float mu_y = (src_rows - 1.0) / (dest_rows - 1.0);

    float adj_2d[16]; // matrix for storing adjacents

    for (uint8_t y_idx = 0; y_idx < dest_rows; y_idx++)
    {
        for (uint8_t x_idx = 0; x_idx < dest_cols; x_idx++)
        {
            float x = x_idx * mu_x;
            float y = y_idx * mu_y;
            //  Serial.print("("); Serial.print(y_idx); Serial.print(", "); Serial.print(x_idx); Serial.print(") = ");
            //  Serial.print("("); Serial.print(y); Serial.print(", "); Serial.print(x); Serial.print(") = ");
            get_adjacents_2d(src, adj_2d, src_rows, src_cols, x, y);

            //  Serial.print("[");
            //  for (uint8_t i=0; i<16; i++) {
            //    Serial.print(adj_2d[i]); Serial.print(", ");
            //  }
            //  Serial.println("]");

            float frac_x = x - (int)x; // we only need the ~delta~ between the points
            float frac_y = y - (int)y; // we only need the ~delta~ between the points
            float out = bicubicInterpolate(adj_2d, frac_x, frac_y);
            //  Serial.print("\tInterp: "); Serial.println(out);
            set_point(dest, dest_rows, dest_cols, x_idx, y_idx, out);
        }
    }
}

// p is a list of 4 points, 2 to the left, 2 to the right
float cubicInterpolate(float p[], float x)
{
    float r = p[1] + (0.5 * x * (p[2] - p[0] + x * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3] + x * (3.0 * (p[1] - p[2]) + p[3] - p[0]))));

    // Serial.print("interpolating: [");
    // Serial.print(p[0],2); Serial.print(", ");
    // Serial.print(p[1],2); Serial.print(", ");
    // Serial.print(p[2],2); Serial.print(", ");
    // Serial.print(p[3],2); Serial.print("] w/"); Serial.print(x); Serial.print(" = ");
    // Serial.println(r);

    return r;
}

// p is a 16-point 4x4 array of the 2 rows & columns left/right/above/below
float bicubicInterpolate(float p[], float x, float y)
{
    float arr[4] = {0, 0, 0, 0};
    arr[0] = cubicInterpolate(p + 0, x);
    arr[1] = cubicInterpolate(p + 4, x);
    arr[2] = cubicInterpolate(p + 8, x);
    arr[3] = cubicInterpolate(p + 12, x);
    return cubicInterpolate(arr, y);
}

// src is rows*cols and dest is a 4-point array passed in already allocated!
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y)
{
    // Serial.print("("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");
    // pick two items to the left
    dest[0] = get_point(src, rows, cols, x - 1, y);
    dest[1] = get_point(src, rows, cols, x, y);
    // pick two items to the right
    dest[2] = get_point(src, rows, cols, x + 1, y);
    dest[3] = get_point(src, rows, cols, x + 2, y);
}

// src is rows*cols and dest is a 16-point array passed in already allocated!
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y)
{
    // Serial.print("("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");
    float arr[4];
    for (int8_t delta_y = -1; delta_y < 3; delta_y++)
    {                                          // -1, 0, 1, 2
        float *row = dest + 4 * (delta_y + 1); // index into each chunk of 4
        for (int8_t delta_x = -1; delta_x < 3; delta_x++)
        { // -1, 0, 1, 2
            row[delta_x + 1] = get_point(src, rows, cols, x + delta_x, y + delta_y);
        }
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

    float *buff = (float *)m_malloc(64 * sizeof(float));
    mp_obj_t *ret_val = (mp_obj_t *)m_malloc(64 * sizeof(mp_obj_t));

    common_hal_amg8833_thermal_read_pixel_temperature(self, buff);

    for (int i = 0; i < 64; i++)
    {
        ret_val[i] = mp_obj_new_float(buff[i]);
    }
    mp_obj_t ret = mp_obj_new_tuple(64, ret_val);
    m_free(buff);
    m_free(ret_val);
    return ret;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_read_pixel_temperature_obj, 1, 1, amg8833_thermal_read_pixel_temperature);

mp_obj_t amg8833_thermal_read_pixel_temperature_scale(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    mp_obj_list_t *data = MP_OBJ_TO_PTR(args[1]);
    int width = mp_obj_get_int(args[2]);
    int height = mp_obj_get_int(args[3]);
    float *dest = (float *)m_malloc(width * height * sizeof(float));
    float *src = (float *)m_malloc(64 * sizeof(float));
    common_hal_amg8833_thermal_read_pixel_temperature(self, src);
    interpolate_image(src, 8, 8, dest, width, height);

    for (int i = 0; i < width * height; i++)
    {
        data->items[i] = mp_obj_new_float(dest[i]);
    }
    m_free(dest);
    m_free(src);

    return data;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_read_pixel_temperature_scale_obj, 4, 4, amg8833_thermal_read_pixel_temperature_scale);

mp_obj_t amg8833_thermal_set_upper_limit(size_t n_args, const mp_obj_t *args)
{
   abstract_module_t *self = (abstract_module_t *)args[0];
    float limit = mp_obj_get_float(args[1]);
    uint16 tmep = limit%1 / 0.25;
    tmep &= (limit -  limit%1) << 2;
    uint8_t value[2];
    value[0] = tmep&0x00FF;
    value[1] = tmep&0xFF00 << 8;
    common_hal_amg8833_thermal_set_upper_limit(self, value);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_set_upper_limit_obj, 2, 2, amg8833_thermal_set_upper_limit);

mp_obj_t amg8833_thermal_set_lower_limit(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    float limit = mp_obj_get_float(args[1]);
    uint16 tmep = limit%1 / 0.25;
    tmep &= (limit -  limit%1) << 2;
    uint8_t value[2];
    value[0] = tmep&0x00FF;
    value[1] = tmep&0xFF00 << 8;
    common_hal_amg8833_thermal_set_lower_limit(self, value);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_set_lower_limit_obj, 2, 2, amg8833_thermal_set_lower_limit);

mp_obj_t amg8833_thermal_set_hysteresis(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    uint16_t limit = mp_obj_get_int(args[1]);
    uint8_t value[2];
    value[0] = limit & 0xFF00 >> 8;
    value[1] = limit && 0x00FF;
    common_hal_amg8833_thermal_set_hysteresis(self, value);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_set_hysteresis_obj, 2, 2, amg8833_thermal_set_hysteresis);

mp_obj_t amg8833_thermal_set_sensor_mode(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    uint8_t mode = mp_obj_get_int(args[1]) & 0xFF;
    common_hal_amg8833_thermal_set_sensor_mode(self, mode);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_set_sensor_mode_obj, 2, 2, amg8833_thermal_set_sensor_mode);

mp_obj_t amg8833_thermal_set_interrupt_mode(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    uint8_t mode = mp_obj_get_int(args[1]) & 0xFF;
    common_hal_amg8833_thermal_set_interrupt_mode(self, mode);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_set_interrupt_mode_obj, 2, 2, amg8833_thermal_set_interrupt_mode);

mp_obj_t amg8833_thermal_clear_status(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    uint8_t value = mp_obj_get_int(args[1]) & 0xFF;
    amg8833_thermal_clear_status(self, value);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_clear_status_obj, 2, 2, amg8833_thermal_clear_status);

mp_obj_t amg8833_thermal_frame_rate(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    uint8_t value = mp_obj_get_int(args[1]) & 0xFF;
    common_hal_amg8833_thermal_frame_rate(self, value);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_frame_rate_obj, 2, 2, amg8833_thermal_frame_rate);

mp_obj_t amg8833_thermal_get_interrupt_status(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];
    if (common_hal_amg8833_thermal_get_interrupt_status(self) == 1)
        return mp_obj_new_bool(true);

    return mp_obj_new_bool(false);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_get_interrupt_status_obj, 1, 1, amg8833_thermal_get_interrupt_status);

mp_obj_t amg8833_thermal_read_pixels_interrupt_status(size_t n_args, const mp_obj_t *args)
{
    abstract_module_t *self = (abstract_module_t *)args[0];

    uint8_t *buff = (uint8_t *)m_malloc(8*sizeof(uint8_t));
    mp_obj_t *ret_val = (mp_obj_t *)m_malloc(8 * sizeof(mp_obj_t));

    common_hal_amg8833_thermal_read_pixels_interrupt_status(self, buff);

    for (int i = 0; i < 8; i++)
    {
        ret_val[i] = mp_obj_new_int(buff[i]);
    }
    mp_obj_t ret = mp_obj_new_tuple(8, ret_val);
    m_free(buff);
    m_free(ret_val);
    return ret;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(amg8833_thermal_read_pixels_interrupt_status_obj, 1, 1, amg8833_thermal_read_pixels_interrupt_status);

const mp_rom_map_elem_t amg8833_thermal_mode_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_Normal), MP_ROM_INT(0x00)},
    {MP_ROM_QSTR(MP_QSTR_Sleep), MP_ROM_INT(0x10)},
    {MP_ROM_QSTR(MP_QSTR_StandBy_60sec_intermittence), MP_ROM_INT(0x20)},
    {MP_ROM_QSTR(MP_QSTR_StandBy_10sec_intermittence), MP_ROM_INT(0x21)},
};

MP_DEFINE_CONST_DICT(amg8833_thermal_mode_locals_dict, amg8833_thermal_mode_locals_dict_table);

static const mp_obj_type_t amg8833_thermal_mode_type = {
    {&mp_type_type},
    .name = MP_QSTR_mode,
    .locals_dict = (mp_obj_t)&amg8833_thermal_mode_locals_dict,
};

void amg8833_thermal_obj_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{

    abstract_module_t *self = (abstract_module_t *)self_in;
    uint32_t value;

    if (dest[0] == MP_OBJ_NULL)
    {
        if (attr == MP_QSTR_is_connect)
        {
            bool ret = common_hal_amg8833_thermal_is_connect(self);
            dest[0] = mp_obj_new_bool(ret);
            return;
        }
    }
    generic_method_lookup(self_in, attr, dest);
}

const mp_rom_map_elem_t amg8833_thermal_locals_dict_table[] = {
    // instance methods
    {MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&amg8833_thermal_deinit_obj)},
    {MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj)},
    {MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&amg8833_thermal_obj___exit___obj)},
    {MP_ROM_QSTR(MP_QSTR_read_pixel_temperature), MP_ROM_PTR(&amg8833_thermal_read_pixel_temperature_obj)},
    {MP_ROM_QSTR(MP_QSTR_read_pixel_temperature_scale), MP_ROM_PTR(&amg8833_thermal_read_pixel_temperature_scale_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_upper_limit), MP_ROM_PTR(&amg8833_thermal_set_upper_limit_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_lower_limit), MP_ROM_PTR(&amg8833_thermal_set_lower_limit_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_hysteresis), MP_ROM_PTR(&amg8833_thermal_set_hysteresis_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_sensor_mode), MP_ROM_PTR(&amg8833_thermal_set_sensor_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_interrupt_mode), MP_ROM_PTR(&amg8833_thermal_set_interrupt_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_clear_status), MP_ROM_PTR(&amg8833_thermal_clear_status_obj)},
    {MP_ROM_QSTR(MP_QSTR_frame_rate), MP_ROM_PTR(&amg8833_thermal_frame_rate_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_interrupt_status), MP_ROM_PTR(&amg8833_thermal_get_interrupt_status_obj)},
    {MP_ROM_QSTR(MP_QSTR_mode), MP_ROM_PTR(&amg8833_thermal_mode_type)},
};

MP_DEFINE_CONST_DICT(amg8833_thermal_locals_dict, amg8833_thermal_locals_dict_table);

const mp_obj_type_t grove_amg8833_thermal_type = {
    {&mp_type_type},
    .name = MP_QSTR_grove_amg8833_thermal,
    .make_new = amg8833_thermal_make_new,
    .locals_dict = (mp_obj_t)&amg8833_thermal_locals_dict,
    .attr = &amg8833_thermal_obj_attr,
};