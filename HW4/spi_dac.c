#include "NU32.h"       // constants, funcs for startup and UART
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

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

// PIC is the master, ram is the slave
// SDO1(RB2) -> SDI
// SDI1(RB8) -> VOUTA 
// SCK1(RB14) -> SCK
// SS1(RB15) -> CS
#define CS LATBbits.LATB15       // chip select pin
#define pi 3.1415926

static volatile int Waveforms[1000]; //wave form
static volatile int Waveformt[2000]; //wave form

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, char voltage){ //channel is 0 or 1 (for VoutA and VoutB)
    if(channel == 0){
    CS = 0; 
    spi_io(voltage >> 4 | 0b01110000);
    spi_io(voltage << 4);
    CS = 1;
    }  
    else{
    CS = 0; 
    spi_io(voltage >> 4 | 0b11110000);
    spi_io(voltage << 4);
    CS = 1; 
    }
}
void makesinewave(){
    int i = 0;
    for (i = 0; i < 1000; ++i){
        Waveforms[i] = 127 + (int)128*sin(2*pi*10*0.001*i);
        
    }
}
void maketrianglewave(){
    int i = 0;
    for (i = 0; i < 2000; ++i){    
        Waveformt[i] = (int)255*i*0.0005;
    }
}

void spi1_init() {
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB15 = 0;//Set RB15 as output (manual SS1)
  SDI1Rbits.SDI1R = 0b0100;//Set SDI1 to RPB8 
  RPB2Rbits.RPB2R = 0b0011;//Set RPB2 to SDO1
  CS = 1;
 
  // setup spi
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1;            // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.MODE32 = 0;  //MODE<32,16> = <0,0> ->communication is byte-wide (8 bits)
  SPI1CONbits.MODE16 = 0;
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
}
void expander_init(){
    i2c_master_start();
    i2c_master_send();
    i2c_master_send();
    i2c_master_send();
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
    
    __builtin_enable_interrupts();
    
    makesinewave();
    maketrianglewave();
    spi1_init();
    i2c_master_setup();
    expander_init();
    
    while(1) {
        _CP0_SET_COUNT(0); // Reset the core counter
        static int s = 0;
        static int t = 0;
        setVoltage(0,Waveforms[s]);
        setVoltage(1,Waveformt[t]);
        s++;
        t++;
        if(s == 1000){ s = 0;}
        if(t == 2000){ t = 0;}        
        while(_CP0_GET_COUNT() < 24000){;}    
}
