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
    while(SPI_SR & (1<<7)){}

    // Disable SPI
    SPI_CR1 &= ~(1<<6);
}

// Enables SPI1 in GPIOA
void _SPI1_init(){

    // ---------- Clocks -------------------
    RCC_AHB1ENR |= 1<<0; // enable GPIOA clock
    RCC_APB2ENR |= 1<<12; // enable SPI1 clock

    // ----------- GPIO init ---------------
    GPIOA_MODER |= (2<<10)|(2<<12)|(2<<14)|(1<<(CS_pin*2));
    GPIOA_OSPEEDR |= (2<<10)|(2<<12)|(2<<14)|(1<<(CS_pin*2)); // high speed
    GPIOA_AFRL |= (5<<20)|(5<<24)|(5<<28); // 5 = spi1-4

    GPIOA_ODR |= (1<<CS_pin); // cs triggers on falling edge so set high in setup

    // ---------- SPI ----------------------
    //_turn_off_SPI1();
    SPI_CR1 &= ~(0b111<<3); //change baud rate (currently 1/2)
    SPI_CR1 |= (1<<9); // enable SSM (use gpio pins for cs)
    SPI_CR1 |= (1<<8); // set SSI high (crutial since active low)
    SPI_CR1 &= ~(1<<11); // 8bit frame format
    SPI_CR1 &= ~(1<<7); // MSb first
    SPI_CR2 &= ~(1<<4); // Motorola mode
    SPI_CR1 |= (1<<2); // Master config
    SPI_CR1 |= (1<<6); // SPI enable
}

void _SPI1_send(uint8_t data){

    // wait fror the TXE buffer to empty
    while((SPI_SR & (1<<1)) == 0){}

    // write data to register
    SPI_DR = data;

    // wait for recieve buffer to fill with trash data
    while(!(SPI_SR & (1<<0))){}

    // read empty data
    uint16_t temp = SPI_DR;
    (void)temp;  
}

uint8_t _SPI1_receive(){

    // wait fror the TXE buffer to empty
    while((SPI_SR & (1<<1)) == 0){}
    
    // send dummy signal
    SPI_DR = 0;

    // wait for Receive buffer to fill
    while(!(SPI_SR & (1<<0))){}

    uint16_t data = SPI_DR;
    return (uint8_t)data;
}

// ------------------------------- Sensor --------------------------------------

// sends write command to data at specific address
void _write_data_at_addr(uint8_t address, uint8_t data){

    // not neccesary but just for clairty (MSB = 0 means write)
    uint8_t adjusted_addr = address & ~(1<<7);

    _cs_enable();
    _SPI1_send(adjusted_addr);
    _SPI1_send(data);
    _cs_disable();
}

// reads data from sensor at specific address
uint8_t _get_data_from_addr(uint8_t address){

    uint8_t adjusted_addr = address | (1<<7); // first bit is R/W

    _cs_enable();
    _SPI1_send(adjusted_addr);
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

    // ------- configure filters ------
    
    // gyros are smooth but they drift
    // use HPF to catch changes but limit small consitent errors
    // addr for CTRL7_G //! HPF en with fc= 2hz
    _write_data_at_addr(0x16, 0b01100000);

    // XL are jittery especialy with movement 
    // use LPF to catch gravity but not movement
    // addr for CTRL8_XL //! LPF en with fc = ODR/100
    _write_data_at_addr(0x17, 0b10100100);


    // ------- enable XL --------------
    // addr for CTRL1_XR //! 833 HZ & +-2g (change?)
    _write_data_at_addr(0x10, 0b01110000);

    // ------- enable gyro ------------
    // addr for CTRL2_G //! 833 HZ & 250dps (change?)
    _write_data_at_addr(0x11, 0b01110000);
}

uint8_t read_status_register(){

    uint8_t data = _get_data_from_addr(0x1E);
    
    return data;
}

struct data_3D get_gyro_data(){

    int16_t x_data = _get_16bit_data(0x23,0x22);
    int16_t y_data = _get_16bit_data(0x25,0x24);
    int16_t z_data = _get_16bit_data(0x27,0x26);

    struct data_3D data = {x_data, y_data, z_data};

    return data;
}

struct data_3D get_accel_data(){

    int16_t x_data = _get_16bit_data(0x29,0x28);
    int16_t y_data = _get_16bit_data(0x2B,0x2A);
    int16_t z_data = _get_16bit_data(0x2D,0x2C);

    struct data_3D data = {x_data, y_data, z_data};

    return data;
}