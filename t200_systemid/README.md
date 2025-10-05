## Summary

The code is intended to perform System Identification on a Blue Robotics T200 Thruster using a Basic ESC R3

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
$ (gdb) load t200_systemid.out 
```

4. That's all! you can type `continue` or press physical reset button to see LCD screen display accelerometer data on the LCD screen.

