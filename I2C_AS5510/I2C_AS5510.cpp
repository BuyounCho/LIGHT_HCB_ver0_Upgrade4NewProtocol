#include "mbed.h"
#include "setting.h"

void look_for_hardware_i2c()
{
    //pc.printf("\r\n\n\n");
    //pc.printf("Note I2C address 0x1C used by FXOS8700CQ 3-axis accelerometer and 3-axis magetometer\r\n");
    //pc.printf("Start hardware search..... \r\n");

    int count = 0;
    for (int address=12; address<256; address+=2) {
        if (!i2c.write(address, NULL, 0)) {         // 0 returned is OK
            //pc.printf(" - I2C device found at address 0x%02X\n\r", address >>1);
            count++;
        }
    }
    //pc.printf("%d devices found \n\r", count);
}

void init_as5510(int i2c_address)
{
    int i2c_adrs=0;
    char idata[2];
    int result=0;

    //pc.printf("\r\n");
    //pc.printf("Start AS5510 init.. \r\n");

    i2c_adrs= (i2c_address << 1);                   // AS5510 Slave address lsb= 0 for write

    //---------- Magnet selection --------------------------------
    //----0x00= <50mT-----------Strong magnet
    //----0x01= <25mT
    //----0x02= <18.7mT
    //----0x03= <12.5mT---------Weak magnet
    //-----------------------------------------------------------
    idata[0]=0x0B;                                  // Register for Sensitivity
    idata[1]=0x00;                                  // Byte
    result= i2c.write(i2c_adrs, idata, 2, 0);       // Now write_sensitivity
//    if (result != 0) pc.printf("No ACK bit! (09)\n\r");

    //----------- Operation mode selection------------------------
    idata[0]=0x02;                                  // 0x02 address setup register for operation, speed, polarity
    idata[1]=0x04;                                  // Normal Operation, Slow mode (1), NORMAL Polarity (0), Power Up (0)
    result= i2c.write(i2c_adrs, idata, 2, 0);       // Now write_operation
//    if (result != 0) pc.printf("No ACK bit! (11)\n\r");

    //pc.printf("AS5510 init done\r\n");
}


int offset_comp(int i2c_address)
{
    int adrss=0;
    int oresult=0;
    char off_data[2];
    int ocf_done=0;

    // First, now Write pointer to register 0x00----------------------------
    adrss= (i2c_address << 1);                  // AS5510 Slave address lsb= 0 for write
    oresult= i2c.write(adrss, 0x00, 1, 0);      // write one byte
    if (oresult != 0) //pc.printf("No ACK bit! (33)\n\r");

    // Second, now Read register 0x00 and 0x01--------------------------------
    memset(off_data, 0, sizeof(off_data));
    adrss= (i2c_address << 1) | 0x01;           // AS5510 address lsb= 1 for read
    oresult= i2c.read(adrss, off_data, 2, 0);   // read two bytes

    // Now analyse register 0x01 ----------------------------------------------
    ocf_done= off_data[1] & 0x08;               // mask off bits, 1= done
    if (ocf_done== 0)  return(0);               // return(0)= compensation process is pending
    else return(1);                             // return(1)= compensation process is completed
}


void read_field(int i2c_address)
{
    int adr=0;
    char rx_data[2];
    int rresult=0;
    char lsb, msb;

    // First, now Write pointer to register 0x00----------------------------
    adr= (i2c_address << 1);                        // AS5510 address lsb= 0 for write
    rresult= i2c.write(adr, 0x00, 1, 0);            // write one byte to register 0x00 for magnetic field strength
//    if (rresult != 0) pc.printf("No ACK bit! (22)\n\r");

    // Second, now Read register 0x00 and 0x01--------------------------------
    memset(rx_data, 0, sizeof(rx_data));
    adr= (i2c_address << 1) | 0x01;                 // AS5510 address lsb= 1 for read
    rresult= i2c.read(adr, rx_data, 2, 0);          // read two bytes


    // Now analyse register 0x01 ----------------------------------------------
    lsb= rx_data[0];                                // get LSB
    msb= rx_data[1]&0x03;                           // need only 2 low bits og MSB
    value = ((msb & 0x03)<<8) + lsb;
//    pc.printf("I2C adres= 0x%02X, Magnetic Field => msb= 0x%02X, lsb= 0x%02X, decimal 10-bit value = %u \r\n ", i2c_address, rx_data[0],rx_data[1], value);

}