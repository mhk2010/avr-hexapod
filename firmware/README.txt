HEXAPOD
=======

    HEXAPOD is a robot. It s a mechanical construction that uses 6 arms to achieve a movement. This hexapod uses 2 servos for arm. It limits its movement flexibility remarkably, but is believed to achieve sufficient movements. One servo control the angle between tibia and femur, the other servo control the vertical angle between body and tibia. Tibia builds a fixed horizontal angle with body and thus limits the range of movements.

                     2
                    /\
                 3 /  \
                  /    \ 4
            _____/      \
             5   1       \
                          \

	 Where (1) is the shoulder. Shoulder contain first servo. The rotation allows to bring the entire arm forward and backwards. The first servo and its case form COXA (5). It is relatively short. The shoulder connects the COXA with FEMUR (3). The angle is fixed, no servo is used. FEMUR and COXA are set in 45 degrees angle. The joint between FEMUR and TIBIA (4) contain second servo. The direction allows to change the angle between FEMUR and TIBIA and thus control the "reach" of the arm.

	 User geometry allows to create basic movements like forward, backward, turn left and right. The one dimensional joint between COXA and FEMUR causes the fixed heights of the robot above the surface. Ideal geometry brings one more servo at the left end of the COXA and changes way of rotation of the first servo -> modify angle between COXA and FEMUR on the same plane as the joint between FEMUR and TIBIA.	 



STATUS     
======

  1.1.2012:
    start looking at the construction options.

  18.3.2012
    The work on the firmware started.


REQUIREMENTS
============

    1. Implement the data display, similar to the BatCentre way of displaying
       the flight data.

    2. Implement simple configuration facility that sets the 3 essential
       parameters of Watchdog device.

       - clear memory
       - toggle logging
       - toggle radio

    3. Use SPI based communication protocol to talk to Watchdog. That protocol
       uses the same pins that are already available for ISP programming head.
       The only pin must be added for SS to bing it to logical zero. That jumper
       (bridge) is also used in Watchdog to enter the specific mode - answers
       the commands from AVR-DBB unit only.

    4. use a simple keyboard to control the device. 


DESIGN NOTES
============

	 The base design uses heavily an interrupt to generate the control pulses for servos. The HW uses decade counter 4017 as servo shiled. Decade counter has 10 output and the input signal propagaes on the outputs as the chich gets clocked. 
