#include "i2c-sensor.h"
#include <stdio.h>
#define MACHXO2_ADDR 0x69
#define OPTICAL_ADDR 0x42

#define OCTOPUS_I2C_BUS 0x00

//Those command here reads from one of the MACH XO2 buses
//octopus_bus can be 
// 0 --> NORTH BUS
// 1 --> SOUTH BUS
// 2 --> CLOCK SYNTHESIZER BUS
// 3 --> OPTICAL MODULE BUS 

#define NORTH_BUS   0x0
#define SOUTH_BUS   0x1
#define CLOCK_BUS   0x2
#define OPTICAL_BUS 0x3
#define OPTICS_BUS  0x4

int twos_complement(int val,u8 bits);
int i2c_read_octopus_bus(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 reg, u8 *result,u8 reg16b);
int i2c_read_word_octopus_bus(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 reg, int *result,u8 reg16b);
int i2c_write_octopus_bus(int i2c_fd_snsr,u8 octopus_bus,u8 slave_addr, u8 reg, u8 data,u8 reg16b); 


//Those two functions are talking to the optiocal module groups
#define OPTICAL_BUS_0     0x0
#define OPTICAL_BUS_1     0x1
#define OPTICAL_BUS_2     0x2



int i2c_read_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, u8 *result,u8 reg16b);
int i2c_read_word_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, int *result,u8 reg16b);
int i2c_write_optical_bus(int i2c_fd_snsr,u8 optical_bus,u8 slave_addr, u8 reg, u8 data,u8 reg16b); 




//Sensors some have V,some I some P some T.See the octopus.c file !

//those have only T
#define K7                        5
#define VUP                       6

//those have T 
#define RAIL_2V7_INTERMEDIATE     0
#define RAIL_0V9_MGTAVCC_VUP_N    1
#define RAIL_1V2_MGTAVTT_VUP_N    2 
#define RAIL_0V9_MGTAVCC_VUP_S   3
#define RAIL_1V2_MGTAVTT_VUP_S    4
#define RAIL_3V3_OPTICAL_G0   7
#define RAIL_3V3_OPTICAL_G1   8
#define RAIL_3V3_OPTICAL_G2   9
#define RAIL_3V3_OPTICAL_G3   10
#define RAIL_3V3_OPTICAL_G4   11
#define RAIL_0V85_VCCINT_VUP 29

//those have V and I +above rails
#define RAIL_1V0_VCCINT_K7 21
#define RAIL_1V8_VCCAUX_VUP 30



//those have V + above rails
#define RAIL_12V0 12
#define RAIL_3V3_STANDBY 13
#define RAIL_3V3_SI5395J 14
#define RAIL_1V8_SI5395J_XO2 15
#define RAIL_2V5_OSC_NE 16
#define RAIL_2V5_OSC_NW 17
#define RAIL_2V5_OSC_SE 18
#define RAIL_2V5_OSC_SW 19
#define RAIL_1V8_MGTVCCAUX_VUP_N 20
#define RAIL_2V5_OSC_K7 22
#define RAIL_1V2_MGTAVTT_K7 23
#define RAIL_1V0_MGTAVCC_K7 24
#define RAIL_0V675_DDR_VTT 25
#define RAIL_1V35_DDR 26
#define RAIL_1V8_VCCAUX_K7 27
#define RAIL_1V8_MGTVCCAUX_VUP_S 28
#define RAIL_OPTICAL_12V0 31
#define RAIL_OPTICAL_3V3_STANDBY 32






//temperature. sensor from above , num is the number of the regulator or FPGA if there are more
u8 power_up_octopus(int i2c_fd_snsr,u8 timeout);
void power_down_octopus(int i2c_fd_snsr);
u8 power_up_qsfpdd_module(int i2c_fd_snsr,u8 timeout);
void power_down_qsfpdd_module(int i2c_fd_snsr);
int optics_temperature(int i2c_fd_snsr,int mask);
void  configure_octopus(int i2c_fd_snsr);
void  configure_qsfpdd_module(int i2c_fd_snsr);
int optics_presence(int i2c_fd_snsr);
void set_led(int i2c_fd_snsr,u8 cage,u8 r,u8 g,u8 b);
int optics_powered;
