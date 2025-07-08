#include <Wire.h>
#include <stdint.h>

#define TS_3032_C7_ADDRESS 0x51  // Dirección del dispositivo esclavo
#define SDA_PIN 21
#define SCL_PIN 22

//------------TS3032C7_Registers-----------//
#define REG_100th_Seconds        0x00  
#define REG_Seconds              0x01        
#define REG_Minutes              0x02        
#define REG_Hours                0x03          
#define REG_Weekday              0x04        
#define REG_Day                  0x05           
#define REG_Month                0x06          
#define REG_Year                 0x07
#define REG_Temp_MSBs            0x0F
#define REG_Temp_LSBs            0x0E
#define REG_Control_1            0x10
#define REG_EEADDR               0X3D
#define REG_EEDATA               0X3E
#define REG_EECMD                0X3F

//------------XTAL_CLKOUT_FREC_SEL_REGISTERS------------//
#define REG_CLKout_1             0xC2
#define REG_CLKout_2             0xC3
#define Default_32_768KHz        0x00
#define XTAL_CLKOUT_1024Hz       0x20
#define XTAL_CLKOUT_64Hz         0x40
#define XTAL_CLKOUT_1Hz          0x60

//----------EEPROM COMANDS FOR EECMD REG---------//
#define UPDATE_RAM_CONF_TO_EEPROM 0X11
#define REFRESH_EEPROM_CONF_TO_RAM 0X12
#define WRITE_TO_ONE_EEPROM_BYTE 0X21
#define READ_ONE_EEPROM_BYTE 0X22

//-----------CLOCK OUTPUT MODES-----------//
#define XTAL_MODE 0
#define HF_MODE 1

typedef struct{
  int Hour;
  int Min;
  int Seg;
}TIME;

typedef struct{
  int MSBs;
  float LSBs;
}TEMPERATURE;

void setup() {
  Serial.begin(115200);
  Wire.begin();  
  byte error;

  for (int i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("TS-3032-C7 ADDRESS: 0x");
      Serial.println(i, HEX);
    }
    delay(10);
  }


}

void loop() {
  
  


  while(1);
  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');
  //   uint16_t value = input.toInt();
  //   SET_HF_CLK_OUTPUT_FREC(value);
  //   Serial.println(value);
  // }
}

//CLKOUT Frequency
// Selection in HF mode
// = HFD × 8.192 kHz
void SET_HF_CLK_OUTPUT_FREC(uint16_t  HFD){
  HFD--;

  if(HFD >= 8191)
    HFD = 8191;
  if(HFD <= 0)
    HFD = 0;

  uint8_t HFD_0_to_7_BITS = (uint8_t)(HFD&0xFF);
  uint8_t HFD_8_to_12_BITS = (uint8_t)((HFD&0x1F00)>>8);
  HFD_8_to_12_BITS |= 0x80;
 
  WRITE_TO_EEPROM(REG_CLKout_1, HFD_0_to_7_BITS);
  REFRESH_RAM_REG_FROM_EEPROM_REG();
  
  WRITE_TO_EEPROM(REG_CLKout_2, HFD_8_to_12_BITS);
  REFRESH_RAM_REG_FROM_EEPROM_REG();
}

void SET_XTAL_CLK_OUTPUT_FREC(uint8_t XTAL_FREC){
  WRITE_TO_EEPROM(REG_CLKout_2, XTAL_FREC);
  REFRESH_RAM_REG_FROM_EEPROM_REG();
}

void REFRESH_RAM_REG_FROM_EEPROM_REG(){
  //----------Disable Auto refresh---------//
  int Reg_state = Read_from_Register(REG_Control_1);
  Reg_state |= 0x4;
  Write_to_Register(REG_Control_1, Reg_state);
  delay(10);

  //-----------REFRESH_EEPROM_CONF_TO_RAM------------//
  Write_to_Register(REG_EECMD, REFRESH_EEPROM_CONF_TO_RAM);
  delay(10);

  //----------Enable Auto refresh---------//
  Reg_state = Read_from_Register(REG_Control_1);
  Reg_state &= 0xFB;
  Write_to_Register(REG_Control_1, Reg_state);
  delay(10);
}

byte READ_EEPROM(byte REG_ADDRESS){
  //----------Disable Auto refresh---------//
  int Reg_state = Read_from_Register(REG_Control_1);
  Reg_state |= 0x4;
  Write_to_Register(REG_Control_1, Reg_state);
  delay(10);

  //-------address to read data---------//
  Write_to_Register(REG_EEADDR, REG_ADDRESS);
  delay(10);

  //------Verify if EEPROM is not bussy before reading-----//
  while(IS_EEPROM_BUSSY());

  //-----send 0x22 comand to EECMD register-------//
  Write_to_Register(REG_EECMD, READ_ONE_EEPROM_BYTE);

  //-----Read byte from EEDARA REG------//
  byte DATA = Read_from_Register(REG_EEDATA);

  //----------Enable Auto refresh---------//
  Reg_state = Read_from_Register(REG_Control_1);
  Reg_state &= 0xFB;
  Write_to_Register(REG_Control_1, Reg_state);
  delay(10);
  return DATA;

}

void WRITE_TO_EEPROM(byte REG_ADDRESS, byte DATA){
  //----------Disable Auto refresh---------//
  int Reg_state = Read_from_Register(REG_Control_1);
  Reg_state |= 0x4;
  Write_to_Register(REG_Control_1, Reg_state);
  delay(10);

  //-------address to write data---------//
  Write_to_Register(REG_EEADDR, REG_ADDRESS);
  delay(10);

  //-------data to write to EEPROM-------//
  Write_to_Register(REG_EEDATA, DATA);
  delay(10);

  //------Verify if EEPROM is not bussy before writing-----//
  while(IS_EEPROM_BUSSY());

  //-----send 0x22 comand to EECMD register-------//
  Write_to_Register(REG_EECMD, WRITE_TO_ONE_EEPROM_BYTE);
  delay(10);

  //----------Enable Auto refresh---------//
  Reg_state = Read_from_Register(REG_Control_1);
  Reg_state &= 0xFB;
  Write_to_Register(REG_Control_1, Reg_state);
  delay(10);

}

byte IS_EEPROM_BUSSY(){
  byte REG_STATUS = Read_from_Register(REG_Temp_LSBs);
  return (REG_STATUS >> 2) & 0x01;
}

void PRINT_TEMP(){
  TEMPERATURE Temp;
  Temp.MSBs = Read_temp_Integer_Part(Read_from_Register(REG_Temp_MSBs));
  Temp.LSBs = Read_temp_Decimal_Part(Read_from_Register(REG_Temp_LSBs));
  Serial.println((float)(Temp.MSBs + Temp.LSBs),4);
}

int Read_temp_Integer_Part(int binario){
    if(binario & 0x80){
        return (((~binario)&255)+1)*-1;
    }
    else{
        return binario;
    }
}

float Read_temp_Decimal_Part(int binario){
    int temp;
    if(binario & 0x80){
        binario = (binario & 0xF0)>>4;
        return (float)((((~binario)&7)+1)*-0.0625);
    }
    else{
        temp = (binario & 0xF0)>>4;
        return (float)temp*0.0625;
    }
}

void PRINT_TIME(){
  TIME time_ = READ_TIME();
  Serial.print(time_.Hour);
  Serial.print(":");
  Serial.print(time_.Min);
  Serial.print(":");
  Serial.println(time_.Seg);
}

TIME READ_TIME(){
  TIME tiempo_leido;
  tiempo_leido.Hour = BCD_to_decimal(Read_from_Register(REG_Hours));
  tiempo_leido.Min = BCD_to_decimal(Read_from_Register(REG_Minutes));
  tiempo_leido.Seg = BCD_to_decimal(Read_from_Register(REG_Seconds));

  return tiempo_leido;
}

void SET_TIME(byte Hour, byte min, byte seg){
  Write_to_Register(REG_Hours, decimal_to_BCD(Hour));
  Write_to_Register(REG_Minutes, decimal_to_BCD(min));
  Write_to_Register(REG_Seconds, decimal_to_BCD(seg));
}

int decimal_to_BCD(int decimal){
    int BCD[3];
  
    if(decimal<10)
      return decimal;
    
    else if(decimal>=10 && decimal<100){
        BCD[0] = (int)(decimal/10);
        BCD[1] = (decimal - BCD[0]*10);
        return (BCD[0] << 4 | BCD[1]);
    }
    else if(decimal>=100 && decimal <1000){
        BCD[0] = (int)decimal/100;
        BCD[1] = (int)(decimal - BCD[0]*100)/10;
        BCD[2] = (int)(decimal - BCD[0]*100 - BCD[1]*10);
        return (BCD[0] << 8 | BCD[1] << 4 | BCD[2]);
    }
}

int BCD_to_decimal(int BCD){
    int decimal[3];

    if(BCD < 10){
        return BCD;
    }
    else if(BCD >= 10 && BCD < 100){
        decimal[0] = BCD >> 4;
        decimal[1] = BCD & 15;
        return decimal[0]*10 + decimal[1];
    }
    else if(BCD >= 100 && BCD < 1000){
        decimal[0] = (BCD & 3840) >> 8;
        decimal[1] = (BCD & 240) >> 4;
        decimal[2] = BCD & 15;
        return decimal[0]*100 + decimal[1]*10 + decimal[2];
    }
}

int Write_to_Register(int register_address, int data){
  Wire.beginTransmission(TS_3032_C7_ADDRESS);
  Wire.write(register_address);
  Wire.write(data);   
  int error = Wire.endTransmission();  // Retorna 0 si la escritura fue exitosa
  return error;     

}

int Read_from_Register(int register_address){
  Wire.beginTransmission(TS_3032_C7_ADDRESS); 
  Wire.write(register_address);
  Wire.endTransmission(false);
  Wire.requestFrom(TS_3032_C7_ADDRESS, (uint8_t)1);
  delay(10);

  if (Wire.available()) {
    return Wire.read();  // Lee y retorna el valor del registro
  }

  return 0; 
}















