# Seeed ArduPy AMG8833 [![Build Status](https://api.travis-ci.com/Seeed-Studio/seeed-ardupy-amg8833.svg?branch=master)](https://travis-ci.com/github/Seeed-Studio/seeed-ardupy-amg8833)

## Introduction

The AGM8833 is a high Precision Infrared Array Sensor based on Advanced MEMS Technology.

You can get more information in here [AMG8833](https://github.com/Seeed-Studio/Seeed_AMG8833).

## How to binding with ArduPy
- Install [AIP](https://github.com/Seeed-Studio/ardupy-aip)
- Build firmware with Seeed ArduPy AMG8833
```shell
aip install Seeed-Studio/seeed-ardupy-amg8833
aip build
```
- Flash new firmware to you ArduPy board
```shell
aip flash # + Ardupy Bin PATH
```
For more examples of using AIP, please refer to [AIP](https://github.com/Seeed-Studio/ardupy-aip).
## Usage

```python
from arduino import grove_amg8833_thermal
from machine import LCD
from machine import Sprite
import time

tft = LCD()
th = grove_amg8833_thermal()
spr = Sprite(tft)

MinTemp = 25
MaxTemp = 35
HDTemp = [0.0]*1600
red = 0
green = 0
blue = 0

a = 0.0
b = 0.0
c = 0.0
d = 0.0

s = 0
x = 0
y = 0
ShowGrid = -1
pixels = [0.0]*64
colors_table = [0]*100
#HDTemp = [[0.0 for i in range(80)] for i in range(80)]

def toggleGrid():
    global ShowGrid
    ShowGrid = ShowGrid * -1
    tft.fillRect(15, 15, 210, 210, tft.color.BLACK)

def constrain(amt,low,high):
    value = low if (amt < low) else (high if (amt > high) else amt)
    return int(value)

#function to get the cutoff points in the temp vs RGB graph
def Getabcd():
    global a
    global b
    global c
    global d
    a = MinTemp + (MaxTemp - MinTemp) * 0.2121
    b = MinTemp + (MaxTemp - MinTemp) * 0.3182
    c = MinTemp + (MaxTemp - MinTemp) * 0.4242
    d = MinTemp + (MaxTemp - MinTemp) * 0.8182


def GetColors():

    global colors_table

    val = 25.0
    for i in range(100):
        colors_table[i] = calcColor(val)
        val += 0.1

    #print(colors_table)

def GetColor(val):
    if val <= 25.0:
        return 30
    if val >= 35.0:
        return 65518
    index = int((val - 25.0) * 10)
    return colors_table[index]
    
# my fast yet effective color interpolation routine
def calcColor(val):
    
    global red
    global green
    global blue

    red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255)

    if (val > MinTemp) and (val < a):
        green = constrain(255.0 / (a - MinTemp) * val - (255.0 * MinTemp) / (a - MinTemp), 0, 255)
    elif (val >= a) & (val <= c):
        green = 255
    elif (val > c):
        green = 0

    if val <= b:
        blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255)
    elif ((val > b) and (val <= d)):
        blue = 0;
    elif (val > d):
        blue = constrain(240.0 / (MaxTemp - d) * val - (d * 240.0) / (MaxTemp - d), 0, 240)
    
    return tft.color565(red, green, blue)

#function to draw a legend
def DrawLegend():
    #color legend with max and min text
  
    inc = (MaxTemp - MinTemp ) / 160.0
    j = 0
    ii = MinTemp
    while ii < MaxTemp:
        tft.drawFastHLine(260, 200 - j, 30, GetColor(ii))
        j = j + 1
        ii = ii + inc
    

    tft.setTextSize(2)
    tft.setTextColor(tft.color.WHITE, tft.color.BLACK)
    str_tmep = '{a}/{b}'.format(a=MaxTemp, b=95)
    tft.drawString(str_tmep, 245, 20)
    str_tmep = '{a}/{b}'.format(a=MinTemp, b=77)
    tft.drawString(str_tmep, 245, 210)

# function to display the results
def DisplayGradient():

    global BoxWidth
    global BoxHeigh
   
    for row in range(40):
        for col in range(40):
            spr.fillRect((row * 5) + 10, (col * 5) + 10, 5, 5, GetColor(HDTemp[row*40+col]))


def init():
    global pixels

    print(th.read_pixel_temperature())

    print("\n\r")
    tft.fillScreen(tft.color.BLACK)
    tft.setRotation(3)
    spr.createSprite(220, 220)
   
    Getabcd()
    GetColors()
    DrawLegend()

def run():
    
    global pixels
    global HDTemp
    
    tick1 = time.ticks_ms()
    while True:
        spr.fillSprite(spr.color.WHITE)
        th.read_pixel_temperature_scale(HDTemp, 40, 40)

        #return
        DisplayGradient()
        #Crosshair in the middle of the screen
        spr.drawCircle(115, 115, 5, tft.color.WHITE)
        spr.drawFastVLine(115, 105, 20, tft.color.WHITE)
        spr.drawFastHLine(105, 115, 20, tft.color.WHITE)
        spr.setRotation(0)
        spr.setTextColor(tft.color.WHITE)
        tft.setRotation(2)
        spr.pushSprite(0,0)
        tft.setRotation(3)
        tft.drawFloat(HDTemp[35*40+35], 2, 90, 20)



def main():
    init()
    run()


if __name__ == "__main__":
    main()
```

## API Reference

- **read_pixel_temperature(*void*)** : Read out 64 tuples of float.
```python
print(thermal.read_pixel_temperature())
```

- **read_pixel_temperature_scale(*buff\<list\>, width\<int\>, height\<int\>*)** : Read temperature list with length of height * width
```python
buff = [0.0] * 400
thermal.read_pixel_temperature(buff, 20, 20)
```

- **set_sensor_mode(*mode\<int\>*)** : Set the power mode.
```python
thermal.set_sensor_mode(thermal.mode.Normal)
thermal.set_sensor_mode(thermal.mode.Sleep)
thermal.set_sensor_mode(thermal.mode.tandBy_60sec_intermittence)
thermal.set_sensor_mode(thermal.mode.tandBy_10sec_intermittence)
```

- **set_sensor_mode(*mode\<int\>*)** : Set the power mode.
```python
thermal.set_sensor_mode(thermal.mode.Normal)
thermal.set_sensor_mode(thermal.mode.Sleep)
thermal.set_sensor_mode(thermal.mode.tandBy_60sec_intermittence)
thermal.set_sensor_mode(thermal.mode.tandBy_10sec_intermittence)
```
- **get_interrupt_status(*void*)** : Get the status of interrupt.
```python
print(thermal.get_interrupt_status())
```
- **clear_interrupt_status(*mode\<uint8_t\>*)** : Clear status.
```python
thermal.clear_status(0x01)
```
- **set_upper_limit(*value\<float\>*)** : Set upper limit.
```python
thermal.set_upper_limit(40.0)
```

- **set_lower_limit(*value\<float\>*)** : Set lower limit.
```python
thermal.set_lower_limit(23.0)
```

- **set_frame_rate(*value\<uint8_t\>*)** : Set frame rate.
```python
thermal.set_frame_rate(1)
```

- **set_hysteresis(*value\<uint16_t\>*)** : Set hysteresis.
```python
thermal.set_hysteresis(100)
```

----

This software is written by seeed studio<br>
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Contributing to this software is warmly welcomed. You can do this basically by<br>
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above<br>
for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>
