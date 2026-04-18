Libraries used:
Boost.odeint
Lie++
Eigen

Convensions:
ned frame
SI units on system equations

Input:
heading estimation from magnetometer
gnss coordinates in ned frame
barometer data


things to do:
make main thread:
->  init eqf
    loop:
    -> receive data on serial
    -> parse into either mbag or gnss with EqFparser
    -> feed into EqFalgo
    -> plot with EqFplotter


filtered altitude becomes -nan in the IMU propagation step, before print time and before/independent of MagBaroUpdate.


TODO: verify that A and Lift are computed correctly