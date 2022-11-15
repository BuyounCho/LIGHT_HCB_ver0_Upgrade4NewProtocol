#include "mbed.h"
#include "setting.h"
#include "SPI_EEP_ENC.h"

// EEPROM
void spi_eeprom_ready(void){
    int temp1, temp2;
    do{
        eeprom_cs=0;
        eeprom.write(0x06);  //write enable
        eeprom_cs=1;
        
        eeprom_cs=0;
        temp1 = eeprom.write(0x05);     
        temp2 = eeprom.write(0x00);
        eeprom_cs=1;
        temp2=(temp2&(0x03))!= 0x02;
    } while(temp2); // before writing or reading 
 }
 
 void spi_eeprom_write(unsigned short add, int32_t data){
        spi_eeprom_ready();
        add=add*4;
        eeprom_cs=0;
        eeprom.write(0x02);
        eeprom.write(0xff&(add>>8));
        eeprom.write(0xff&add);
        eeprom.write(0xff&data);
        eeprom.write(0xff&(data>>8));
        eeprom.write(0xff&(data>>16));
        eeprom.write(0xff&(data>>24));
        eeprom_cs=1;
}
 
int32_t spi_eeprom_read(unsigned short add){
        add=add*4;
        eeprom_cs=0;
        eeprom.write(0x03);
        eeprom.write(0xff&(add>>8));
        eeprom.write(0xff&add);
        
        int a1 = eeprom.write(0x00);
        int a2 = eeprom.write(0x00);
        int a3 = eeprom.write(0x00);
        int a4 = eeprom.write(0x00);
        eeprom_cs=1;
        int32_t final = (int32_t) (a1 | a2 << 8 | a3 << 16 | a4 << 24);
        return final;
 } 
 
 
 // ENCODER
void spi_enc_set_clear(void){
    unsigned int temp;
    enc_cs = 0;
    temp = enc.write(0b00100000);
    enc_cs = 1;
}

void spi_enc_set_init(void){
    unsigned int temp, i, temp1, temp2;

    // write MDR0 -> 0b11 -> x4 quadrature count mode
    enc_cs = 0;
    temp = enc.write(0b10001000);     // WR + MDR0
    temp = enc.write(0b00000011);     // quadratue mode
    enc_cs = 1;

    // write MDR1 -> 0b10 -> 2-byte counter mode
    for(i=0;i<10000;i++);
    enc_cs = 0;
    temp = enc.write(0b10010000);     // WR + MDR1
    //temp = enc.write(0b00000010);     // 2 byte mode
    temp = enc.write(0b00000000);     // 4 byte mode
    enc_cs = 1;
    
    // clear
    spi_enc_set_clear();
}


int spi_enc_read(void){   
    //for(t_i=0;t_i<100;t_i++);
    unsigned int t_dummy, t_b1, t_b2, t_b3, t_b4, t_i;
    enc_cs = 0;
    t_dummy = enc.write(0b01100000); // Read Commend 
    t_b1 = enc.write(0x00);         // Dummy data for clock
    t_b2 = enc.write(0x00);         // Dummy data for clock
    t_b3 = enc.write(0x00);         // Dummy data for clock
    t_b4 = enc.write(0x00);         // Dummy data for clock
    enc_cs = 1;

    return((t_b1<<24) + (t_b2<<16) + (t_b3<<8) + t_b4);
}