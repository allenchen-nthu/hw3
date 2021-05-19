# hw3
# set up the connection of wifi
# First open the screen by sudo dev/ttyACM0
# when it is successfully connected, you can see "wait for python" on the screen
# Next, open the python program
# after openning, you can see "start angle selection", in the meantime led1 would turn on
# use gesture slope to decrease the threshold angle by 5 degrees, and gesture ring to increase the threshold by 5 degrees
# while the angle is 50 degrees and we increase the threshold angle, it would return to 30 degrees, vice versa
# if we push the user button, the threshold angle would be sent, meanwhile entering the tilt mode, and led1 turn off, led2 turn on
# after the board gets the reference angle, the screen would show the X, Y, Z data and the transformed angle, also the uLCD would show the degrees in integer
# if we surpass the threshold angle, the screen would show "over tilt 1 time"
# if we surpass the threshold angle again, the screen would show "over tilt 2 times"
# if we surpass the threshold angle again, the screen would show "over tilting", then exit the tilt mode, meanwhile, led2 would turn off
# we can also control the program by typing e.g. "/gesture/run"
