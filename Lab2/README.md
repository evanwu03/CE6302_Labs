## Summary

This lab demonstrates to read xyz acceleration values from the accelerometer embedded on the BOOSTXL-EDUMKII. This program is intended to be 

## Prerequisites 

* MSP432P410R Launchpad 
* arm-none-eabi-gcc compiler 
* openocd 
* gdb-multiarch 

## How To Run 


1. To compile the project using make: 
```
$ cd gcc 
$ make all
``` 

2.  Open an openocd session and run the following command 

``` 
$ openocd -f board/ti_msp432_launchpad.cfg 
```


3. Open an arm gdb session and connect to openocd server and load application
``` 
$ gdb-multiarch
$ (gdb) target remote :3333
$ (gdb) load lab2_accelerometer.out 
```

4. That's all! you can type `continue` or press physical reset button to see LCD screen display accelerometer data on the LCD screen.

