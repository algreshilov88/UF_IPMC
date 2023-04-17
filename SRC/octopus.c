#include "octopus.h"
#include <unistd.h>
#include "logger.h"



int twos_complement(int val,u8 bits) {
  if ((val & (1<< (bits-1)))!=0)
    return val-(1<<bits);
  return val;
}


int i2c_read_octopus_bus(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 reg, u8 *result,u8 reg16b) {
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,0))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,2,slave_addr<<1))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,3,reg&0xff))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,4,(reg>>8)&0xff))
    ;
  u8 d=0;
  if (reg16b)
    d=0xd;
  else
    d=0xf;
  d=d|(octopus_bus<<4);
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,d))
    ;
  u8 resu=0;
  while (resu==0) {
    while(i2c_read(i2c_fd_snsr,MACHXO2_ADDR,0,&resu))
      ;
  }
  if (resu==2)
    printf("I2C Read transaction failed bus=%d addr=%d reg=%d\n",octopus_bus,slave_addr,reg);
  while(i2c_read(i2c_fd_snsr,MACHXO2_ADDR,1,result))
    ;
  return resu;
}

int i2c_read_word_octopus_bus(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 reg, int *result,u8 reg16b) {
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,0);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,2,slave_addr<<1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,3,reg&0xff);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,4,(reg>>8)&0xff);
  u8 d=0;
  if (reg16b)
    d=0x9;
  else
    d=0xb;

  d=d|(octopus_bus<<4);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,d);
  u8 resu=0;
  while (resu==0) {
    res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,0,&resu);
  }
  if (resu==2)
    printf("I2C Read Word transaction failed bus=%d addr=%d reg=%d\n",octopus_bus,slave_addr,reg);


  u8 r0,r1;
  res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,1,&r0);
  res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,2,&r1);
  *result=(r1<<8)|r0;
  return (res!=0) ||(resu==0x02);
}



int i2c_write_octopus_bus(int i2c_fd_snsr,u8 octopus_bus,u8 slave_addr, u8 reg, u8 data,u8 reg16b) {
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,0))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,2,slave_addr<<1))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,3,reg&0xff))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,4,(reg>>8)&0xff))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,5,data&0xff))
    ;
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,6,(data>>8)&0xff))
    ;
  u8 d=0x7;
  if (reg16b)
    d=0x5;
  d=d|(octopus_bus<<4);
  while(i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,d))
    ;
  u8 result=0;
  while (result==0) {
    while(i2c_read(i2c_fd_snsr,MACHXO2_ADDR,0,&result))
      ;
  }
  if (result==2)
    printf("I2C Write transaction failed bus=%d addr=%d reg=%d\n",octopus_bus,slave_addr,reg);


  return result;
}

int i2c_read_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, u8 *result,u8 reg16b) {
  int res =  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,0,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,slave_addr<<1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,reg&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,(reg>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,6,optical_bus,0);

  u8 d=0;
  if (reg16b)
    d=0xd;
  else
    d=0xf;
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,d,0);
  u8 resu=0;
  while (resu==0) {
    res += i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,&resu,0);
  }
  if (resu==0x02)
    logger("ERROR","I2C ACK failed on optical bus read");
  res+=i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,result,0);
  return (res!=0) || ((resu)==0x02);
}

int i2c_read_word_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, int *result,u8 reg16b) {
  int res =  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,0,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,slave_addr<<1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,reg&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,(reg>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,6,optical_bus,0);

  u8 d=0;
  if (reg16b)
    d=0x9;
  else
    d=0xb;
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,d,0);

  u8 resu=0;
  while (resu==0) {
    res += i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,&resu,0);
  }
  if (resu==0x02)
    logger("ERROR","I2C ACK failed on read word of optical bus");

  u8 r0,r1;
  res+=i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,&r0,0);
  res+=i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,&r1,0);
  int a = r1<<8;

  *result=(a|r0);
  return (res!=0) || ((resu)==0x02);
}

int i2c_write_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, u8 data,u8 reg16b) {
  int res =  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,0,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,slave_addr<<1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,reg&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,(reg>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,4,data&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,5,(data>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,6,optical_bus,0);
  u8 d=0;
  if (reg16b)
    d=0x5;
  else
    d=0x7;
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,d,0);

  u8 resu=0;
  while (resu==0) {
    res += i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,&resu,0);
  }
  if (resu==0x02) {
    logger("ERROR","I2C ACK failed on optical bus write ");
    //    printf("optical write failure bus =%d addr=%d reg=%d\n",optical_bus,slave_addr,reg);
  }
  return (res!=0) || ((resu)==0x02);
}




void configure_octopus_voltage_sensor(int i2c_fd_snsr,u8 bus,u8 addr,const float v[8],const float  margin[8],const float  factor[8]) {
  u8 mask=0x0;
  u8 i;
  for (i=0;i<8;i=i+1) {
    if (v[i]>0.0)
      mask=mask|(1<<i);
  }
  mask=mask ^0xff;

  int res=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x00,0x80,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x3,mask,0);
  u8 r=0;
  res+=i2c_read_octopus_bus(i2c_fd_snsr,bus,addr, 0x3,&r,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x7,1,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x8,mask,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0xb,2,0);
  for (i=0;i<8;i=i+1) {
    if (v[i]>0.0) {
      u8 maxV = (256.0*v[i]*(1.0+margin[i])/(factor[i]*2.56));
      u8 minV = (256.0*v[i]*(1.0-margin[i])/(factor[i]*2.56));
      i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x2a+2*i,maxV,0);
      i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x2a+2*i+1,minV,0);
    }
  }
  res+=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x0,3,0);
}


void configure_optical_voltage_sensor(int i2c_fd_snsr,u8 bus,u8 addr,const float v[8],const float  margin[8],const float  factor[8]) {
  u8 mask=0x0;
  u8 i;
  for (i=0;i<8;i=i+1) {
    if (v[i]>0.0)
      mask=mask|(1<<i);
  }
  mask=mask ^0xff;

  int res=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x00,0x80,0);
  res+=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x3,mask,0);
  u8 r=0;
  res+=i2c_read_optical_bus(i2c_fd_snsr,bus,addr, 0x3,&r,0);
  res+=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x7,1,0);
  res+=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x8,mask,0);
  res+=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0xb,2,0);
  for (i=0;i<8;i=i+1) {
    if (v[i]>0.0) {
      u8 maxV = (256.0*v[i]*(1.0+margin[i])/(factor[i]*2.56));
      u8 minV = (256.0*v[i]*(1.0-margin[i])/(factor[i]*2.56));
      i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x2a+2*i,maxV,0);
      i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x2a+2*i+1,minV,0);
    }
  }
  res+=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x0,3,0);
}



void  configure_octopus_temperature_sensor(int i2c_fd_snsr,u8 sensor,u8 number,u8 local,u8 remote,float ideality) {
  u8 lsb=0;
  u8 msb=0;
  u8 addr;
  u8 bus;
    
  if (sensor==RAIL_2V7_INTERMEDIATE) {
    addr=0x4d;
    bus=NORTH_BUS;
  }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_N) {
    bus=NORTH_BUS;
    if (number==0)
      addr = 0x48;
    else
      addr = 0x49;
   }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_N) {
    bus=NORTH_BUS;
    if (number==0)
      addr = 0x4b;
    else
      addr = 0x4c;
   }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_S) {
    bus=SOUTH_BUS;

    if (number==0)
      addr = 0x48;
    else
      addr = 0x49;
   }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_S) {
    bus=SOUTH_BUS;

    if (number==0)
      addr = 0x4b;
    else
      addr = 0x4c;
 
  }
  else if (sensor==VUP) {
    bus=NORTH_BUS;
    addr=0x4a;
  }
  else if (sensor==K7) {
    bus=SOUTH_BUS;
    addr=0x4a;
  }
  int res =i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x21,10,0);
  res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x19,local,0);
  res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x20,remote,0);
  u8 adjust = twos_complement(1.008*2088/ideality-2088,8);
  res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x23,adjust,0);
}


void  configure_optical_temperature_sensor(int i2c_fd_snsr,u8 sensor,u8 number,u8 local,u8 remote,float ideality) {
  u8 lsb=0;
  u8 msb=0;
  u8 addr;
  u8 bus;

  if (sensor==RAIL_3V3_OPTICAL_G0) {
    bus=OPTICAL_BUS_1;
    addr=0x48;
  }
  else if (sensor==RAIL_3V3_OPTICAL_G1) {
    bus=OPTICAL_BUS_1;
    addr=0x49;
  }
  else if (sensor==RAIL_3V3_OPTICAL_G2) {
    bus=OPTICAL_BUS_1;
    addr=0x4a;
  }
  else if (sensor==RAIL_3V3_OPTICAL_G3) {
    bus=OPTICAL_BUS_0;
    addr=0x48;
  }
  else if (sensor==RAIL_3V3_OPTICAL_G4) {
    bus=OPTICAL_BUS_0;
    addr=0x49;
  }
  int res =i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x21,10,0);
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x19,local,0);
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x20,remote,0);
  u8 adjust = twos_complement(1.008*2088/ideality-2088,8);
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x23,adjust,0);
}





void  configure_ltm4700(int i2c_fd_snsr,u8 bus,u8 addr) {
  u8 ch;
  for (ch=0;ch<2;ch=ch+1) {
    int res= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x0,ch,0);
    res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0xd4,0xc6,0);
    res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x01,0x80,0);
  }
}


void  configure_octopus(int i2c_fd_snsr) {
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_2V7_INTERMEDIATE,0,90,105,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_N,0,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_N,1,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_N,0,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_N,1,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_S,0,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_S,1,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_S,0,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_S,1,90,95,1.008);
  configure_octopus_temperature_sensor(i2c_fd_snsr,K7,0,85,85,1.010);
  configure_octopus_temperature_sensor(i2c_fd_snsr,VUP,0,90,90,1.026);
  const float nominal_0[8] = {12.0,-1.00,3.3,-1.00,3.3,-1.00,-1.00,1.8};
  const float factor_0[8] = {7.72,-1.00,2.0,-1.00,2.0,-1.00,-1.00,1.0};
  const float margin_0[8] = {0.05,-1.00,0.1,-1.00,0.1,-1.00,-1.00,0.1};

  configure_octopus_voltage_sensor(i2c_fd_snsr,NORTH_BUS,0x1d,nominal_0,margin_0,factor_0);

  const float nominal_1[8] = {2.5,1.8,1.2,2.7,0.9,2.5,-1.0,-1.0};
  const float factor_1[8]  = {2.0,1.0,1.0,2.0,1.0,2.0,-1.0,-1.0};
  const float margin_1[8]  = {0.1,0.05,0.05,0.1,0.05,0.1,-1.0,-1.0};

  configure_octopus_voltage_sensor(i2c_fd_snsr,NORTH_BUS,0x1f,nominal_1,margin_1,factor_1);

  const float nominal_2[8] = {1.0,2.5,1.2,1.0,-1.0,1.35,1.8,2.5};
  const float factor_2[8]  = {1.0,2.0,1.0,1.0,-1.0,1.0,1.0,2.0};
  const float margin_2[8]  = {0.05,0.1,0.05,0.05,-1.0,0.05,0.05,0.1};

  configure_octopus_voltage_sensor(i2c_fd_snsr,SOUTH_BUS,0x1d,nominal_2,margin_2,factor_2);
  const float nominal_3[8] = {1.8,0.9,1.2,0.85,1.8,2.5,-1.0,-1.0};
  const float factor_3[8]  = {1.0,1.0,1.0,1.0,1.0,2.0,-1.0,-1.0};
  const float margin_3[8]  = {0.05,0.05,0.05,0.1,0.05,0.2,-1.0,-1.0};

  configure_octopus_voltage_sensor(i2c_fd_snsr,SOUTH_BUS,0x1f,nominal_3,margin_3,factor_3);
}




void  configure_qsfpdd_module(int i2c_fd_snsr) {
  configure_optical_temperature_sensor(i2c_fd_snsr,RAIL_3V3_OPTICAL_G0,0,70,90,1.008);
  configure_optical_temperature_sensor(i2c_fd_snsr,RAIL_3V3_OPTICAL_G1,0,70,90,1.008);
  configure_optical_temperature_sensor(i2c_fd_snsr,RAIL_3V3_OPTICAL_G2,0,70,90,1.008);
  configure_optical_temperature_sensor(i2c_fd_snsr,RAIL_3V3_OPTICAL_G3,0,70,90,1.008);
  configure_optical_temperature_sensor(i2c_fd_snsr,RAIL_3V3_OPTICAL_G4,0,70,90,1.008);

  const float nominal_0[8] = {-1.0,12.0,3.3,3.3,3.3,3.3,3.3,3.3};
  const float factor_0[8]  = {-1.0,11.0,11.0,11.0,11.0,11.0,11.0,11.0};
  const float margin_0[8]  = {-1.0,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
  configure_optical_voltage_sensor(i2c_fd_snsr,OPTICAL_BUS_0,0x1d,nominal_0,margin_0,factor_0);
}




void power_down_octopus(int i2c_fd_snsr) {
  //disable latches
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,10,1);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1f);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0xf);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x7);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x3);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x0);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0xf);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x7);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x3);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x1);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x0);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x3f);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1f);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0xf);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x7);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x3);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x0);
}

void power_down_octopus_fast(int i2c_fd_snsr) {
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x0);
  res+= i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x0);
  res+= i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x0);
}


void clear_interrupts(int i2c_fd_snsr) {
  int res=0;
  u8 result;
  res = i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,0x1d, 0x1,&result,0);
  res = i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,0x1f, 0x1,&result,0);
  res = i2c_read_octopus_bus(i2c_fd_snsr,SOUTH_BUS,0x1d, 0x1,&result,0);
  res = i2c_read_octopus_bus(i2c_fd_snsr,SOUTH_BUS,0x1f, 0x1,&result,0);

}

void clear_interrupts_optical(int i2c_fd_snsr) {
  int res=0;
  u8 result;
  res = i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_BUS_0,0x1d, 0x1,&result,0);
}


u8 trackVoltage(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 ch,float factor,float v,float margin,u8 timeout,u8 verbose) {
  int result=0;
  int res=0;
  res=i2c_read_word_octopus_bus(i2c_fd_snsr,octopus_bus,slave_addr,0x20+ch, &result,0);
  float converted = 2.56*result*factor/65536.0;
  if (converted>(v*(1.0+margin))) {
      power_down_octopus_fast(i2c_fd_snsr);
      return 0;
  }
  u8 success = converted>(v*(1.0-margin));
  
  u8 t=0;
  while ((!success) && (t<timeout)) { 
      usleep(10000);
      i2c_read_word_octopus_bus(i2c_fd_snsr,octopus_bus,slave_addr,0x20+ch, &result,0);
      converted = 2.56*result*factor/65536.0;
      //OV
      if (converted>v*(1.0+margin)) {
	power_down_octopus_fast(i2c_fd_snsr);
	return 0;
      }
      success = converted>v*(1.0-margin);
      t=t+1;
    }
  if(verbose) {
    printf("Voltage Nominal=%f Margin=%f Readout=%f success=%d\n",v,margin,converted,success);
  }
  if (!success) {
      power_down_octopus_fast(i2c_fd_snsr);
    }
  return success;
}



u8 trackOpticalVoltage(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 ch,float factor,float v,float margin,u8 timeout,u8 verbose) {
  int result=0;
  int res=0;
  res=i2c_read_word_optical_bus(i2c_fd_snsr,octopus_bus,slave_addr,0x20+ch, &result,0);
  float converted = 2.56*result*factor/65536.0;
  if (converted>(v*(1.0+margin))) {
    printf("Optical Module Overvoltage detected\n");
    power_down_qsfpdd_module(i2c_fd_snsr);
      return 0;
  }
  u8 success = converted>(v*(1.0-margin));
  
  u8 t=0;
  while ((!success) && (t<timeout)) { 
      usleep(10000);
      printf("Rereading\n");
      i2c_read_word_optical_bus(i2c_fd_snsr,octopus_bus,slave_addr,0x20+ch, &result,0);
      converted = 2.56*result*factor/65536.0;
      printf("Voltage Nominal=%f Margin=%f Readout=%f success=%d\n",v,margin,converted,success);
      //OV
      if (converted>v*(1.0+margin)) {
	printf("Optical Module Overvoltage detected\n");
	power_down_qsfpdd_module(i2c_fd_snsr);
	return 0;
      }
      success = converted>v*(1.0-margin);
      t=t+1;
    }
  if(verbose) {
    printf("Voltage Nominal=%f Margin=%f Readout=%f success=%d\n",v,margin,converted,success);
  }
  if (!success) {
      power_down_qsfpdd_module(i2c_fd_snsr);
    }
  return success;
}



u8 power_up_octopus(int i2c_fd_snsr,u8 timeout) {

  //disable internal safety system.
  //Since this is the first read retry till you get it
  //to make sure the bus is up

  while (i2c_write(i2c_fd_snsr,MACHXO2_ADDR,10,1)!=0)
    ;

  int res = 0;
  //configure everything
  u8 verbose=1;
  //power up sequence
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1d,0,7.72,12.0,0.05,timeout,verbose)) return 0;
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1d,2,2.0,3.3,0.1,timeout,verbose)) return 0;
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1);
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1f,3,2.0,2.7,0.1,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x7);
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1d,4,2.0,3.3,0.1,timeout,verbose)) return 0;
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1d,7,1.0,1.8,0.1,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0xf);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,1,2.0,2.5,0.1,timeout,verbose)) return 0;
 
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1f);
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1f,0,2.0,2.5,0.1,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x3f);
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1f,5,2.0,2.5,0.1,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x7f);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,7,2.0,2.5,0.1,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0xff);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1f,5,2.0,2.5,0.1,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x1);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,0,1.0,1.0,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x3);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,6,1.0,1.8,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x7);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,5,1.0,1.35,0.05,timeout,verbose)) return 0; //DDR

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0xf);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,3,1.0,1.0,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x1f);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1d,2,1.0,1.2,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1f,3,1.0,0.85,0.05,timeout,verbose)) return 0;
  configure_ltm4700(i2c_fd_snsr,SOUTH_BUS,0x4d); 
  configure_ltm4700(i2c_fd_snsr,SOUTH_BUS,0x4e); 
  configure_ltm4700(i2c_fd_snsr,SOUTH_BUS,0x4f); 
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1f,3,1.0,0.85,0.05,timeout,verbose)) return 0;//again
  

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x3);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1f,4,1.0,1.8,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x7);
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1f,4,1.0,0.9,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0xf);
  if(!trackVoltage(i2c_fd_snsr,NORTH_BUS,0x1f,2,1.0,1.2,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1f);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1f,1,1.0,0.9,0.05,timeout,verbose)) return 0;

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x3f);
  if(!trackVoltage(i2c_fd_snsr,SOUTH_BUS,0x1f,2,1.0,1.2,0.05,timeout,verbose)) return 0;

  clear_interrupts(i2c_fd_snsr);
  //wait for PGOOD
  u8 t=0;
  u8 alerts;
  res = i2c_read(i2c_fd_snsr,MACHXO2_ADDR,4,&alerts);
  while((t<=timeout) && (alerts!=0xff)) {
    res = i2c_read(i2c_fd_snsr,MACHXO2_ADDR,4,&alerts);
    t=t+1;
  }
  if (t>timeout) {
    power_down_octopus_fast(i2c_fd_snsr);    
    return 0;
  }
  //enable latches
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,10,0);
  //trigger a monitoring cycle
  u8 test=0;
  res += i2c_read(i2c_fd_snsr, MACHXO2_ADDR, 142, &test);
  return (alerts==0xff);
}

int optics_presence(int i2c_fd_snsr) {
  u8 msb=0;
  u8 lsb=0;
  i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,5, &msb,0);
  i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,4, &lsb,0);
  return (msb<<8)|lsb;
}
void set_led(int i2c_fd_snsr,u8 cage,u8 r,u8 g,u8 b) {
  u8 mode = ((b&3) << 4) | ((g&3) << 2) | (r&3);
  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,15+cage, mode,0);
}


u8 power_up_qsfpdd_module(int i2c_fd_snsr,u8 timeout) {
  u8 verbose=1;
  int res=0;
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 1,11.0,12.0,0.05,100,verbose))
    return 0;
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 7,11.0,3.3,0.05,100,verbose))
    return 0;
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 7, 0x1f,0);
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 2,11.0,3.3,0.05,100,verbose))
    return 0;
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 3,11.0,3.3,0.05,100,verbose))
    return 0;
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 4,11.0,3.3,0.05,100,verbose))
    return 0;
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 5,11.0,3.3,0.05,100,verbose))
    return 0;
  if(!trackOpticalVoltage(i2c_fd_snsr, OPTICAL_BUS_0,0x1d, 6,11.0,3.3,0.05,100,verbose))
    return 0;
  clear_interrupts_optical(i2c_fd_snsr);
  u8 alerts;
  res = i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,&alerts,0);
  u8 t=0;
  while((t<=timeout) && (alerts!=0xff)) {
    usleep(1000);
    res = i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,&alerts,0);
    t=t+1;
    printf("Optical Alert=%d\n",alerts);
  }
  if (t>timeout) {
    printf("Timeout reached for Optical Module boot, shutting down\n");
    power_down_qsfpdd_module(i2c_fd_snsr);    
    return 0;
  }
  //enable latches
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 8, 0x0,0);
  //Set LPMode for Optics
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 11, 0x0,0);
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 12, 0x0,0);
  //reset Optics
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 9, 0x0,0);
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 10, 0x0,0);
  usleep(30000);
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 9, 0xff,0);
  res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 10, 0xff,0);
  //disco mode
  u8 i=0;
  for (i=0;i<15;++i) {
    set_led(i2c_fd_snsr,i,3,3,3);
  }
  optics_powered=optics_presence(i2c_fd_snsr);

  return 1;
  
}

void power_down_qsfpdd_module(int i2c_fd_snsr) {
  int res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 7, 0x0,0);
}



void select_cage(int i2c_fd_snsr,u8 cage) {
  int i= 1<<cage;
  i=i^0x7fff;
  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,13,0xff& i,0);
  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,14, 0xff & (i>>8),0);
}


int optics_temperature(int i2c_fd_snsr,int mask) {
  u8 i=0;
  float t=20.0;
  for(i=0;i<15;++i) {
    if ((mask&(1<<i))==0) {
      //found cage 
      select_cage(i2c_fd_snsr,i);
      //check what version it is
      u8 id=0;
      u8 msb=0;
      u8 lsb=0;
      //use while because the optics take 2s to start sometimes!
      i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,0,&id,0);
      if (id==17||id==13||id==12) { //QSFP
	i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,22,&msb,0);
	i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,23,&lsb,0);
      }
      else if (id==24) { //QSFP-DD
	u8 version=0;
	i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,1,&version,0);
	if (version>=0x40) {
	  i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,14,&msb,0);	    
	  i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,15,&lsb,0);
	}
	else {
	  i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,0x1a,&msb,0);
	  i2c_read_octopus_bus(i2c_fd_snsr,OPTICS_BUS,0x50,0x1b,&lsb,0);
	}
      }
      int raw = (msb<<8)|lsb;
      int temp_d = twos_complement(raw,16);
      printf("MSB=%d LSB=%d RAW=%d TEMPD =%d \n",msb,lsb,raw,temp_d);
      float temp_f = temp_d/256.0;
      if(temp_f>t)
	t=temp_f;
    }

  }
  return t;
}
      
