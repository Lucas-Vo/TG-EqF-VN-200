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


TODO:
- extend vn200 binary output to include GPSGROUP_VELNED
- extend EqFparserResult to include velned, so gnssdata is gnssposdata and gnssveldata
- extend gnss measurements to both have pos and vel measuremtns, modify jacobian accordingly
  - you will probably need Mat6x18, Mat18x6 etc...

- move setting this value into the constructor, and make it so that i set the magnetic field in main const Vec3 m = {0.373029, 0.024338, 0.927500};

- modify printMeasurements to print vectornavData.gnsspos, vectornavData.gnssvel, UncompAccel, m cross EqFparserResult.magData converted to angle axis with cos sin, note that m must be passed in as parameter, its print format should be the exact same as TGEqF and VN200, except that you dont have pos_uncert and vel_uncert
- it should
    constexpr const char* kEstimateCsvHeader =
        "timestamp,angle_axis_x,angle_axis_y,angle_axis_z,angle_axis_cos,angle_axis_sin,"
        "pos_N,pos_E,pos_D,vel_N,vel_E,vel_D,accel_x,accel_y,accel_z,pos_uncert,vel_uncert";
- write a logMeasurements that writes to a csv file
- modify all python files to plot measurements in addition to tgeqf and vn200, make its line lime green