#include <stdint.h>
#include "LSM6DS3.h"

// all port A
#define SCK_pin 5
#define MISO_pin 6
#define MOSI_pin 7
#define CS_pin 8

#define RCC_Base 0x40023800
#define RCC_APB2ENR *((volatile uint32_t*)(RCC_Base + 0x44))
#define RCC_AHB1ENR *((volatile uint32_t*)(RCC_Base + 0x30))

#define GPIOA_Base 0x40020000
#define GPIOA_MODER *((volatile uint32_t*)(GPIOA_Base + 0x00))
#define GPIOA_OSPEEDR *((volatile uint32_t*)(GPIOA_Base + 0x08))
#define GPIOA_ODR *((volatile uint32_t*)(GPIOA_Base + 0x14))
#define GPIOA_AFRL *((volatile uint32_t*)(GPIOA_Base + 0x20))

#define SPI1_Base 0x40013000
#define SPI_CR1 *((volatile uint32_t*)(SPI1_Base + 0x00))
#define SPI_CR2 *((volatile uint32_t*)(SPI1_Base + 0x04))
#define SPI_SR *((volatile uint32_t*)(SPI1_Base + 0x08))
#define SPI_DR *((volatile uint32_t*)(SPI1_Base + 0x0C))


// ------------------ SPI things -----------------------------------------------

void _cs_enable(){
    GPIOA_ODR &= ~(1<<CS_pin);
}
void _cs_disable(){
    GPIOA_ODR |= (1<<CS_pin);
}

void _turn_off_SPI1(){

    // wait to recive last data (until RXNE=1)
    while((SPI_SR & (1<<0)) == 0){}

    // wait until TXE=1
    while((SPI_SR & (1<<1)) == 0){}

    // wait until BSY=0
    while((SPI_SR & (1<<7)) == 1){}

    // Disable SPI
    SPI_CR1 &= ~(1<<6);
}

// Enables SPI1 in GPIOA
void _SPI1_init(){

    // ----------- GPIO init ---------------
    GPIOA_MODER |= (2<<10)|(2<<12)|(2<<14)|(1<<(CS_pin*2));
    GPIOA_OSPEEDR |= (2<<10)|(2<<12)|(2<<14)|(1<<(CS_pin*2)); // high speed
    GPIOA_AFRL |= (5<<20)|(5<<24)|(5<<28); // 5 = spi1-4

    // ---------- Clocks -------------------
    RCC_AHB1ENR |= 1<<0; // enable GPIOA clock
    RCC_APB2ENR |= 1<<12; // enable SPI1 clock

    // ---------- SPI ----------------------
    _turn_off_SPI1();
    SPI_CR1 &= ~(0b111<<3); //change baud rate (currently 1/2)
    SPI_CR1 &= ~(1<<11); // 8bit frame format
    SPI_CR1 &= ~(1<<7); // MSb first
    SPI_CR2 &= ~(1<<4); // Motorola mode
    SPI_CR1 &= ~(1<<2); // Master config
    SPI_CR1 |= (1<<6); // SPI enable
}

void _SPI1_send(uint8_t data){

    // wait fror the TXE buffer to empty
    while((SPI_SR & (1<<1)) == 0){}

    SPI_DR = data;
}

uint8_t _SPI1_receive(){

    // send dummy signal
    _SPI1_send(0);

    uint16_t data = SPI_DR;
    return (uint8_t)data;
}

// ------------------------------- Sensor --------------------------------------

// gets data from sensor at specific address
uint8_t _get_data_from_addr(uint8_t address){
    _cs_enable();
    _SPI1_send(address);
    uint8_t data = _SPI1_receive();
    _cs_disable();

    return data;
}

// combines two 8 bit registers to make a signed 16 bit number
int16_t _get_16bit_data(uint8_t high_addr, uint8_t low_addr){
    uint8_t low = _get_data_from_addr(low_addr);
    uint8_t high = _get_data_from_addr(high_addr);

    int16_t data = (high<<8)|low;

    return data;
}

// turns on both gyro and accelerometer
void LSM6DS3_init(){

    _SPI1_init();

    // ------- enable XL --------------
    _cs_enable();
    _SPI1_send(0x10); // addr for CTRL1_XR
    _SPI1_send(0b01110000); //! 833 HZ (change?)
    _cs_disable();

    // ------- enable gyro ------------
    _cs_enable();
    _SPI1_send(0x11); // addr for CTRL2_G
    _SPI1_send(0b01110000); //! 833 HZ & 250dps (change?)
    _cs_disable();
}

struct gyro_data{
    int16_t x;
    int16_t y;
    int16_t z;
};
struct gyro_data get_gyro_data(){

    int16_t x_data = _get_16bit_data(0x23,0x22);
    int16_t y_data = _get_16bit_data(0x25,0x24);
    int16_t z_data = _get_16bit_data(0x27,0x26);

    struct gyro_data data = {x_data, y_data, z_data};

    return data;
}

struct accel_data{
    int16_t x;
    int16_t y;
    int16_t z;
};
struct accel_data get_accel_data(){

    int16_t x_data = _get_16bit_data(0x29,0x28);
    int16_t y_data = _get_16bit_data(0x2B,0x2A);
    int16_t z_data = _get_16bit_data(0x2D,0x2C);

    struct accel_data data = {x_data, y_data, z_data};

    return data;
}