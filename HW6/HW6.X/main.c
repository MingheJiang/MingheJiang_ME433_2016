#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include <math.h> 

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
#pragma config USERID = 0x1234 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

// PIC is the master, ram is the slave
// SDO1(RA1) -> SDI
// SDI1(RB8) -> VOUTA 
// SCK1(RB14) -> SCK
// SS1(RB7) -> CS

#define CS          LATBbits.LATB7       // chip select pin
#define OUT_TEMP_L  0b00100000
#define OUT_TEMP_H  0b00100001
#define OUTX_L_G    0b00100010
#define OUTX_H_G    0b00100011
#define OUTY_L_G    0b00100100
#define OUTY_H_G    0b00100101
#define OUTY_H_G    0b00100110
#define OUTZ_H_G    0b00100111
#define OUTX_L_XL   0b00101000
#define OUTX_H_XL   0b00101001
#define OUTY_L_XL   0b00101010
#define OUTY_H_XL   0b00101011
#define OUTZ_L_XL   0b00101100
#define OUTZ_H_XL   0b00101101

unsigned char read  = 0x00;
unsigned char addr = 0b1101011;
static unsigned char data[14];
static unsigned char dataout[7];

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

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
unsigned char I2C_read_multiple(char address,char register, unsigned char * data, char length){
    i2c_master_start();
    i2c_master_send((address << 1) | 0); // write
    i2c_master_send(register);
    i2c_master_restart();
    i2c_master_send((address << 1) | 1); // read
    int count = 0;
    for(count = 0; count < length; count++)
    {
        data[count] = i2c_master_recv();
        i2c_master_ack(0); // continue reading 
    }
    data[length] = i2c_master_recv(); 
    i2c_master_ack(1);
    i2c_master_stop();
}

unsigned char I2C_read(char register){
    i2c_master_start();
    i2c_master_send(0xD6); //Write 0b1101011(D6h) 
    i2c_master_send(register);
    i2c_master_restart();
    i2c_master_send(0xD7);//read 0b11010111 (D7h)
    read = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return read;
}
void initOC_PWM(){
    //remap RPB as OC3
    RPA0Rbits.RPA0R = 0b0101; //OC1:RA0
    RPA1Rbits.RPA1R = 0b0101; //OC2:RA1
    
    T2CONbits.TCKPS = 4;     // Timer2 prescaler N=16 (1:16)
	PR2 = 2999; // period = (4999+1) * 16 * 12.5 ns = 1000 us, 1 kHz
	TMR2 = 0; // initialize count to 0
   
    OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
    OC1CONbits.OCTSEL	= 0;			// use Timer 2
    OC1RS = 1500;             // duty cycle = OC1RS/(PR2+1) = 50%
    OC1R = 1500;              // initialize before turning OC1 on; afterward it is read-only
    OC2CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
    OC2CONbits.OCTSEL	= 0;			// use Timer 2
    OC2RS = 1500;             // duty cycle = OC1RS/(PR2+1) = 50%
    OC2R = 1500;              // initialize before turning OC2 on; afterward it is read-only    
    T2CONbits.ON = 1;        // turn on Timer2
    OC1CONbits.ON = 1;       // turn on OC1    
    OC2CONbits.ON = 1;       // turn on OC2    
}
int main(){
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
    initOC_PWM();
    __builtin_enable_interrupts();
           
                
    /*      
    i2c_master_start();
    i2c_master_send(0xD6);    
    i2c_master_send(0x0F);//WHO_AM_I (0Fh)
    i2c_master_restart();
    i2c_master_send(0xD7);//read 0b11010111 (D7h)
    read = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    */  
    while(1) {       
        int length = 14;
        _CP0_SET_COUNT(0); // Reset the core counter
        LATAbits.LATA4 = !LATAbits.LATA4;
        I2C_read_multiple(addr, OUT_TEMP_L, data, length);
        int i = 0;
        for(i = 0; i < length/2; i++){    
            dataout[i] = (data[2*i+1] << 8 | data[2*i]);
        }    
        
        
        
        /*
        while(_CP0_GET_COUNT() < 12000){;}
        if(read == 0x69){
            LATAbits.LATA4 = 1; 
        }
        else{
            LATAbits.LATA4 = 0; 
        }*/
    }
        while(_CP0_GET_COUNT() < 24000){;}    
}