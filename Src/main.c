#include <stdint.h>

#include <LSM6DS3.h>

int main(){

    LSM6DS3_init();

    while (1){
       struct data_3D gyro_data = get_gyro_data();
       struct data_3D accel_data = get_accel_data();
    }
    
    return 1;
}
