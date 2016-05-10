#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ILI9163C.h"
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define OUT_TEMP_L  0b00100000

static char string[100];
unsigned char addr = 0b1101011;
static unsigned char data[14];
static signed short dataout[7];

void initXL(){
    i2c_master_start();
    i2c_master_send(0xD6); //Write 0b1101011(D6h)
    i2c_master_send(0x10); //CTRL1_XL (10h)
    i2c_master_send(0x80);  //1.66 kHz
    i2c_master_stop();
}
void initG(){
    i2c_master_start();
    i2c_master_send(0xD6); //Write 0b1101011(D6h)
    i2c_master_send(0x11); //CTRL2_G (11h)
    i2c_master_send(0x80);  //1.66 kHz
    i2c_master_stop();
}
void initC(){
    i2c_master_start();
    i2c_master_send(0xD6); //Write 0b1101011(D6h)
    i2c_master_send(0x12); //CTRL3_C (12h)
    i2c_master_send(0x04);  //enabled IF_INC
    i2c_master_stop();
}

unsigned char I2C_read_multiple(char address,char reg, unsigned char * data, char length){
    i2c_master_start();
    i2c_master_send((address << 1) | 0); // write
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((address << 1) | 1); // read
    int count = 0;
    for(count = 0; count < length-1; count++)
    {
        data[count] = i2c_master_recv();
        i2c_master_ack(0); // continue reading 
    }
    data[length-1] = i2c_master_recv(); 
    i2c_master_ack(1);
    i2c_master_stop();
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;   // pin B4 as input
    
    TRISAbits.TRISA4 = 0;  // pin A4 as output
    //LATAbits.LATA4 = 1;    
    i2c_master_setup();
    initXL();
    initG();
    initC();    
    
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLUE);
    //LCD_drawcharactor('H', 30, 30);
    //int variable = 1337;
    //sprintf(string, "Hello World %d!", variable);
    //LCD_drawstring(string,28,32);
    __builtin_enable_interrupts();
    
    while(1) {       
        int length = 14;
        _CP0_SET_COUNT(0); // Reset the core counter
        I2C_read_multiple(addr, OUT_TEMP_L, data, length);
        while(_CP0_GET_COUNT() < 12000){;}
        int i = 0;
        for(i = 0; i < length/2; i++){    
            dataout[i] = (data[2*i+1] << 8 | data[2*i]);
        }
        
        sprintf(string, "temperature %5.2f *C", 25+(dataout[0]/16.0)); 
        LCD_drawstring(string,23,22); 
        sprintf(string, "g-x %5.3f", dataout[1]/32768.0*245); 
        LCD_drawstring(string,23,32); 
        sprintf(string, "g-y %5.3f", dataout[2]/32768.0*245); 
        LCD_drawstring(string,23,42); 
        sprintf(string, "g-z %5.3f", dataout[3]/32768.0*245); 
        LCD_drawstring(string,23,52);         
        sprintf(string, "acc-x %5.3f", dataout[4]/32768.0*2); 
        LCD_drawstring(string,23,62); 
        sprintf(string, "acc-y %5.3f", dataout[5]/32768.0*2); 
        LCD_drawstring(string,23,72); 
        sprintf(string, "acc-z %5.3f", dataout[6]/32768.0*2); 
        LCD_drawstring(string,23,82);         
        while(_CP0_GET_COUNT() < 24000){;} 
    }
      
}