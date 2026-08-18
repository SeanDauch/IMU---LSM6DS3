#ifndef LSM6DS3_h
    #define LSM6DS3_h

    void LSM6DS3_init();
    struct gyro_data get_gyro_data();
    struct accel_data get_accel_data();
#endif
