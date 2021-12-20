# AMG8833Thermal
My take on AMG8833. I took the simplistic code using only a subset of the accessible capabilities and then combined it with the Arduino demo version, 
and also made it use the regular i2c libraries on Raspberry Pi and output to a screen buffer. 

Pertinent links also in code:
https://github.com/ayaromenok/RaspberryPiCameraForAMG8833
This is the simplistic version RaspberryPiCameraForAMG8833-master. I didn't want to use that extra GUI stuff though. 
For solving other issues along the way, like using the already-there i2c library and such, I went to these places. 

        https://github.com/nox771/i2c_t3/issues/16
        
        https://github.com/threebrooks/AdafruitStepperMotorHAT_CPP/issues/1
        
        https://www.raspberrypi.org/forums/viewtopic.php?t=189709
        

The images are launch and also the obligatory "waving the finger in front of it". 

Testing was done on a first generation Raspberry Pi Zero. 

Comnpile and link  g++ test.cpp AMG8833IR.cpp  -Wall -std=c++11   -fpermissive -o testamg -lm -lpng
