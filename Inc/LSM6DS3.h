#ifndef LSM6DS3_h
    #define LSM6DS3_h

    void LSM6DS3_init();

    struct data_3D{
    int16_t x;
    int16_t y;
    int16_t z;
    };
    struct data_3D get_gyro_data();
    struct data_3D get_accel_data();
#endif
