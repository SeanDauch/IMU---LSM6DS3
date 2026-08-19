#include <stdint.h>

#include <LSM6DS3.h>

int main(){

    LSM6DS3_init();

    while (1){
        uint8_t status = read_status_register();
        struct data_3D gyro_data = get_gyro_data();
        status = read_status_register();
        struct data_3D accel_data = get_accel_data();
    }
    
    return 1;
}
