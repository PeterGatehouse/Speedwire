///////////////////////////////////////////////////////////////////

//18may the CONFIGURATION window isn't right so I DID edit these
//18may I enabled 2ndary oscillator on 32768Hz for TIMER1. it was off.


//DO NOT EDIT CONFIG HERE DIRECTLY
//USE THE Configuration Bits below and Make Source then Paste it below!



///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV


// PIC32MZ2048ECG144 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

#include "font.h"
// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = OFF            // Permission Group Lock One Way Configuration (Allow multiple reconfigurations)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (2x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_POSC      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_60        // System PLL Multiplier (PLL Multiply by 60)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)
#pragma config UPLLEN = OFF             // USB PLL Enable (USB PLL is disabled)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON            // Secondary Oscillator Enable (ENABLE SOSC)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = ON            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disabled, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = ALLOW_PG2       // Debug Mode CPU Access Permission (Allow CPU access to Permission Group 2 permission regions)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC5




// ^^^^^^^^^^^^^^ PASTE IN CONFIG BITS ABOVE ^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^ PASTE IN CONFIG BITS ABOVE ^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^ PASTE IN CONFIG BITS ABOVE ^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^ PASTE IN CONFIG BITS ABOVE ^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^ PASTE IN CONFIG BITS ABOVE ^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^ PASTE IN CONFIG BITS ABOVE ^^^^^^^^^^^^^^^^^

// VVVVVVVVV     START OF MY CODE  VVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVV     START OF MY CODE  VVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVV     START OF MY CODE  VVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVV     START OF MY CODE  VVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVV     START OF MY CODE  VVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVV     START OF MY CODE  VVVVVVVVVVVVVVVVVVVVVV

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

//#include "Compiler.h"
//#include "GenericTypeDefs.h"
#define BYTE char
#define WORD unsigned short int

#include "LCD1x9.h"
#include "LCD1x9_mapping.h"

/* DEFINE LOCAL TYPES HERE */

/* DEFINE LOCAL CONSTANTS HERE */
#define N8X8 18
//SPILEN is total length of string so SPILEN/SPIROWS per row!!
#define SPILEN 24   
#define SPIROWS 2
#define SPIDELAY 2
#define UEXT_SDA_TRIS TRISAbits.TRISA3
#define UEXT_SDA_LAT LATAbits.LATA3
#define UEXT_SDA_PORT PORTAbits.RA3

#define UEXT_SCL_TRIS TRISAbits.TRISA2
#define UEXT_SCL_LAT LATAbits.LATA2
#define UEXT_SCL_PORT PORTAbits.RA2

// LCD address is 0x70
#define LCD1x9_SLAVE_ADDR 0b01110000

#define ACK 0
#define NACK 1

/* DEFINE LOCAL VARIABLES HERE */
static BYTE lcdBitmap[20]; // 40segments * 4 = 160px, 160 / 8 = 20bytes
static BYTE spimat[8*SPIROWS][8*N8X8/SPIROWS]; // wastefully 1 byte per dot
static BYTE dhspimat[8*SPIROWS][8*N8X8/SPIROWS]; // wastefully 1 byte per dot
static WORD dmatSPI[8*N8X8];  // the words to send to 7219s
WORD dataSPI;
WORD darraySPI[N8X8];
char spistr[24];
char lcd[9];



unsigned int record[8]; //store by #lives
//try (9D07F000) or (BD07F000)  i.e. at the top of prog FLASH 
// BD gave no run at all, blank or random 
// 9D gave repeat restart bug
// 1D not aollowed, physical addr???
//*CAT
//const unsigned int __attribute__((section("settings"),address(0xBD07F000))) record[8];



//144 pixels perpixel 24 bits msbit first 8G 8R 8B    
//neopixel TH+TL 1.25us
//T0H 0.4us //T0L 0.85us
//T1H 0.8us //T1L 0.45us
//then at least 50us hold low to latch this all in.

//TBD nb the brightness needs gamma correction!!!!!!

//LED STRIP vars prefixed s on copy from lightorgan
    int sledval;
    int sled;
    int sledstart;
    int sledcolor;
    int sbitval;
    int sp;
    int sledbit;
    int spval;
    int spvalr, spvalg, spvalb;
    int swot; //why =1 not working
    int sd1;
    int stripg[144]={255};
    int stripr[144]={255};
    int stripb[144]={255};
    //end LED STRIP

    unsigned int brightness=0;
    unsigned int volume=0;
    BYTE ddata =0;
    int cusl=0;
    int cusr=0;    
    int probe=0;
    int start=0;
    
/* DECLARE LOCAL FUNCTIONS HERE */
static void local_I2C_Delay(void);
static void local_I2C_Start(void);
static void local_I2C_Stop(void);
static char local_I2C_WriteByte(BYTE data); // returns ack state

//onboard green LED
#define LED_TRIS    TRISHbits.TRISH2    // macro for direction register bit of the LED pin
#define LED_LAT     LATHbits.LATH2      // macro for output register bit of the LED pin

#define SPICLK_TRIS TRISKbits.TRISK7
#define SPICLK_LAT  LATKbits.LATK7
#define SPIDAT_TRIS TRISKbits.TRISK6
#define SPIDAT_LAT  LATKbits.LATK6
#define SPINCS_TRIS TRISKbits.TRISK5
#define SPINCS_LAT  LATKbits.LATK5

#define K0_TRIS TRISKbits.TRISK0
#define K0_LAT LATKbits.LATK0

#define START_TRIS TRISDbits.TRISD3
#define START_LAT LATDbits.LATD3
#define START_PORT PORTDbits.RD3

#define LIV0_TRIS TRISDbits.TRISD4
#define LIV0_LAT LATDbits.LATD4
#define LIV0_PORT PORTDbits.RD4
#define LIV1_TRIS TRISDbits.TRISD5
#define LIV1_LAT LATDbits.LATD5
#define LIV1_PORT PORTDbits.RD5
#define LIV2_TRIS TRISDbits.TRISD6
#define LIV2_LAT LATDbits.LATD6
#define LIV2_PORT PORTDbits.RD6
#define METER_TRIS TRISDbits.TRISD11
#define METER_RPD RPD11R
#define METER_LAT LATDbits.LATD11
#define METER_PORT PORTDbits.RD11
#define SPKR_TRIS TRISDbits.TRISD12
#define SPKR_LAT LATDbits.LATD12
#define SPKR_PORT PORTDbits.RD12

/////////////////////////////////////////////////////
//144 pixels perpixel 24 bits msbit first 8G 8R 8B    
//neopixel TH+TL 1.25us
//T0H 0.4us //T0L 0.85us
//T1H 0.8us //T1L 0.45us
//then at least 50us hold low to latch this all in.
void stripfire() 
{
    int ii;
    
    for (sled=0; sled<144; sled++) {
    for (sledcolor=0; sledcolor<3; sledcolor++) {
    sp=128;    
    for (sbitval=7; sbitval>=0; sbitval--) {

    if (sledcolor==0) sledval=stripg[sled];  //ineff but equalize pixel timings
    if (sledcolor==1) sledval=stripr[sled];
    if (sledcolor==2) sledval=stripb[sled];
       
    if ( (sledval & sp) == sp) sledbit=1;     
    if ( (sledval & sp) == 0)  sledbit=0;     
   
    //T0H 0.4us //T0L 0.85us
    if (sledbit==0) {
    K0_LAT = 1;
    for (ii=0; ii<5; ii++) {} // 0.4us
    K0_LAT =0;
    for (ii=0; ii<4; ii++) {} // 0.85us <<including the rest of the loop
    }
    
    //T1H 0.8us //T1L 0.45us
    if (sledbit==1) {
    K0_LAT =1;
    for (ii=0; ii<12; ii++) {} // 0.85us
    K0_LAT =0;
    //for (ii=0; ii<1; ii++) {} // 0.4 <<including the rest of the loop
    }
    
    sp = sp / 2;
    } //next bitval
    } // next ledcolor within the pixel
    } // next led neopixel

    
    for (ii=0; ii<2000; ii++) {} // 100us approx
    
}



//half a bit time about 1us should be ok
void SPI_Delay()
{
    int kk;
    int k2=0;
    for (kk=0; kk<SPIDELAY;kk++);
}

/******************************************************************************
* Description: SPI_Send(..) 
 * THIS EXPECTS SPIWORD IS A N8X8-WORD ARRAY!!!!!
 *******************************************************************************/
SPI_Send(WORD *spiword)
{
    int lenspiword=16;
    int ii;
    int wordcount=0;
    int thisbit;
    
    SPI_Delay();
    SPINCS_LAT=0;
    SPI_Delay();
    for (wordcount=0; wordcount<N8X8;wordcount++) {
        WORD spibit=32768;
        for (ii=0; ii<lenspiword; ii++) {
            thisbit = spiword[wordcount] & spibit;
            spibit=spibit/2;
            if (thisbit==0) SPIDAT_LAT=0;
            if (thisbit!=0) SPIDAT_LAT=1;
            SPI_Delay();
            SPICLK_LAT=1;
            SPI_Delay();
//pg15apr            SPI_Delay();
            SPICLK_LAT=0;
            SPI_Delay();
        } //next spibit
    } //next wordcount

    //set NCS high for this clock cycle (leave it high)
    SPINCS_LAT=1;
    SPIDAT_LAT=0; //don't care just for neatness leave it 0
    SPI_Delay();
    SPICLK_LAT=1;
    SPI_Delay();
//pg15apr    SPI_Delay();
    SPICLK_LAT=0;
    SPI_Delay();
    }


/******************************************************************************
* Description: SPI_Clear(..) 
* THIS EXPECTS SPILEN in 8x6 Chars filled by 7x5 font
 * Plot the string starting col , row from top left (0,0)
 * 
 *******************************************************************************/

SPI_Clear ()
{
//3. send dmatSPI. note same reg to all 7219s so altered order here!
// now we really do have to send rightmost column first!
int colin7219,ii;
for (colin7219=1; colin7219<=8; colin7219++) //the column within each 7219 1=rightcol       
{  
    for (ii=0; ii<N8X8; ii++) // which 8*8
    {
        darraySPI[ii]= 256*colin7219;
    }
    SPI_Send(darraySPI);
    for (ii=10000; ii>0; ii--);    
} // next colin7219 address within each 8X8    

}

/******************************************************************************
* Description: SPI_String(..) 
* THIS EXPECTS SPILEN in 8x6 Chars filled by 7x5 font
 * Plot the string starting col , row from top left (0,0)
 * 
 *******************************************************************************/
SPI_String (char *spistr, int ocol, int orow)
{
    int i, irow, icol, ichar;
// 0.clear bitmap
    for (irow=0; irow<8*SPIROWS; irow++) {
        for (icol=0; icol<8*(N8X8/SPIROWS); icol++) {
            spimat[irow][icol]=0;
        }
    }
    

// 1. copy font bitmap into spimat
    for (ichar=0; ichar<SPILEN; ichar++)
    {
        char c = spistr[ichar];
        int ic= c-32;
        int cmap[7];
        int ledval;
        int tempchar;
        int temprow;
        
        for (i=0; i<7; i++) cmap[i] = font[ic][i];
        // {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04},  // 0x21, ! 
        
        for (irow=0; irow<7; irow++)
        {
            for (icol=0; icol<5; icol++)
            {
                ledval=cmap[irow];
                ledval= ledval & ( 16 >> icol);
                // the 8x8s update by columns
                // BYTE spimat[8*SPIROWS][8*N8X8/SPIROWS]
                tempchar=ichar;
                temprow=0;
                if (ichar>= (SPILEN/SPIROWS)) {
                    tempchar=ichar-(SPILEN/SPIROWS);
                    temprow=1;
                    }
                int tcol,trow;
                tcol = 6*tempchar+icol+ocol;
                trow = 8*temprow+irow+orow;
                int skip=0;  //don't write outside the array!
                if (tcol<0) skip=1;
                if (trow<0) skip=1;
                if (tcol>=6*(SPILEN/SPIROWS)) skip=1;
                if (trow>=8*SPIROWS) skip=1;
                if (skip==0) spimat[trow][tcol]=ledval;
            }
        }
    
    } //next char in spistr
    
// 2. rearrange spimat to commands for 7219 in dmatSPI
//send rightmost column first <<<this doesn't matter in step 2.
//so that's the end of SPIROWS    
    // the 8x8s update by columns
    // BYTE spimat[8*SPIROWS][8*N8X8]
        int coladdr7219=1; 
        int matrow=0;
        int matcol=0;
        for (icol=8*N8X8-1; icol>=0; icol--)
        {
            int rowval=128;  //sendvalue start at top row
            int sendval=0;
            
            //upper or lower row of the 2-row N8X8 layout
            matrow=0;
            matcol=0;
            if (icol>=(8*N8X8/SPIROWS)) {
                matrow=1;
                matcol=1;  //flag to move left!
            }
            for (irow=0; irow<8; irow++)
            {
                if (spimat[irow+8*matrow][icol-matcol*(8*N8X8/SPIROWS)]!=0) sendval=sendval+rowval;
                rowval=rowval/2;       

            } //next irow    

        dmatSPI[icol] = 256*coladdr7219+sendval;  // address4bits data8bits ! 7219datasheet
        coladdr7219=coladdr7219+1;
        if (coladdr7219>8) coladdr7219=1;
            
        } //next icol    
        
//3. send dmatSPI. note same reg to all 7219s so altered order here!
// now we really do have to send rightmost column first!
        int colin7219,ii;
for (colin7219=1; colin7219<=8; colin7219++) //the column within each 7219 1=rightcol       
{  
    ii=0; // which 8*8 (0 is the furthest to the right)
    for (icol=8*N8X8-colin7219; icol>=0; icol=icol-8) // because colin starts = 1!
    {
        darraySPI[ii]=dmatSPI[icol];  //send rightmost first!!
        //darraySPI[ii]=256*colin7219+colin7219+ii;  //send rightmost first!!
        ii++;
    }
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    
} // next colin7219 address within each 8X8    
        
}




/******************************************************************************
* Description: SPI_DHString(..) 
* THIS EXPECTS SPILEN/4 in  filled by doubled 7x5 font 14x10
 * 9x8 72leds across
 * Plot the string starting col , row from top left (0,0)
 * 
 *******************************************************************************/
SPI_DHString (char *spistr, int ocol, int orow)
{
    int i, irow, icol, ichar;
// 0.clear bitmap
    for (irow=0; irow<8*SPIROWS; irow++) {
        for (icol=0; icol<8*(N8X8/SPIROWS); icol++) {
            spimat[irow][icol]=0;
        }
    }
    

// 1. copy font bitmap into spimat
    for (ichar=0; ichar<SPILEN/4; ichar++)
    {
        BYTE c = spistr[ichar];
        int ic= c-32;
        int cmap[7];
        int ledval;
        int tempchar;
        int temprow;
        
        for (i=0; i<7; i++) cmap[i] = font[ic][i];
        // {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04},  // 0x21, ! 
        
        for (irow=0; irow<7; irow++)
        {
            for (icol=0; icol<5; icol++)
            {
                ledval=cmap[irow];
                ledval= ledval & ( 16 >> icol);

                // the 8x8s update by columns
                // BYTE spimat[8*SPIROWS][8*N8X8/SPIROWS]
                tempchar=ichar;
                temprow=0;
                if (ichar>= (SPILEN/SPIROWS)) {
                    tempchar=ichar-(SPILEN/SPIROWS);
                    temprow=1;
                    }
                int tcol,trow;
                tcol = 6*tempchar+icol;//+ocol;
                trow = 8*temprow+irow; //+orow;
                //int skip=0;  //don't write outside the array!
                //if (tcol<0) skip=1;
                //if (trow<0) skip=1;
                //if (tcol>=6*(SPILEN/SPIROWS)) skip=1;
                //if (trow>=8*SPIROWS) skip=1;
                spimat[trow][tcol]=ledval;
            }
        }
    
    } //next char in spistr
  
    
    //double spimat
    int id,jd;
    for (id=0; id<72; id++) {
        for (jd=0; jd<16; jd++) {
            dhspimat[jd][id]=0;            
            }
        }

    for (id=35; id>=0; id--) {
        for (jd=7; jd>=0; jd--) {
            
            int vd,hd,vt,ht,skip;
            for (vd=0;  vd<2; vd++) {
                for (hd=0;  hd<2; hd++) {
                    vt = 2*jd+vd+orow;
                    ht = 2*id+hd+ocol;
                    skip=0;
                    if (vt<0) skip=1;
                    if (ht<0) skip=1;
                    if (vt>15) skip=1;
                    if (ht>71) skip=1;
                    if (skip==0) dhspimat[vt][ht]=spimat[jd][id];
                }
            }

/*             
            spimat[2*jd+1][2*id+1]=spimat[jd][id];
            spimat[2*jd][2*id+1]=spimat[jd][id];
            spimat[2*jd+1][2*id]=spimat[jd][id];
            spimat[2*jd][2*id]=spimat[jd][id];
 */
            
        }
    }
  
    
    
    
// 2. rearrange spimat to commands for 7219 in dmatSPI
//send rightmost column first <<<this doesn't matter in step 2.
//so that's the end of SPIROWS    
    // the 8x8s update by columns
    // BYTE spimat[8*SPIROWS][8*N8X8]
        int coladdr7219=1; 
        int matrow=0;
        int matcol=0;
        for (icol=8*N8X8-1; icol>=0; icol--)
        {
            int rowval=128;  //sendvalue start at top row
            int sendval=0;
            
            //upper or lower row of the 2-row N8X8 layout
            matrow=0;
            matcol=0;
            if (icol>=(8*N8X8/SPIROWS)) {
                matrow=1;
                matcol=1;  //flag to move left!
            }
            for (irow=0; irow<8; irow++)
            {
                if (dhspimat[irow+8*matrow][icol-matcol*(8*N8X8/SPIROWS)]!=0) sendval=sendval+rowval;
                rowval=rowval/2;       

            } //next irow    

        dmatSPI[icol] = 256*coladdr7219+sendval;  // address4bits data8bits ! 7219datasheet
        coladdr7219=coladdr7219+1;
        if (coladdr7219>8) coladdr7219=1;
            
        } //next icol    
        
//3. send dmatSPI. note same reg to all 7219s so altered order here!
// now we really do have to send rightmost column first!
        int colin7219,ii;
for (colin7219=1; colin7219<=8; colin7219++) //the column within each 7219 1=rightcol       
{  
    ii=0; // which 8*8 (0 is the furthest to the right)
    for (icol=8*N8X8-colin7219; icol>=0; icol=icol-8) // because colin starts = 1!
    {
        darraySPI[ii]=dmatSPI[icol];  //send rightmost first!!
        //darraySPI[ii]=256*colin7219+colin7219+ii;  //send rightmost first!!
        ii++;
    }
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    
} // next colin7219 address within each 8X8    
        
}
//end doubleheight string write




/******************************************************************************
* Description: LCD1x9_Initialize(..) - initializes pins and registers of the LCD1x9
*				Also lights up all segments
* Input: 	none
* Output: 	none
* Return:	0 if sucessfully initialized, -1 if error occured 
*******************************************************************************/
char LCD1x9_Initialize(void)
{
	char result = 0, i;
	
	// the idea is to toggle pins as inputs/outputs to emulate I2C open drain mode of operation!	
	UEXT_SDA_TRIS = 1;
	UEXT_SCL_TRIS = 1;
	
	UEXT_SDA_LAT = 0;
	UEXT_SCL_LAT = 0;
	
	local_I2C_Delay();
	if( !(UEXT_SCL_PORT && UEXT_SDA_PORT) )
		return -1;

	local_I2C_Start();
	result |= local_I2C_WriteByte(LCD1x9_SLAVE_ADDR | 0x00);
	result |= local_I2C_WriteByte(0b11001000); // mode register
	result |= local_I2C_WriteByte(0b11110000); // blink register
	result |= local_I2C_WriteByte(0b11100000); // device select register
	result |= local_I2C_WriteByte(0b00000000); // pointer register
	
	// light up all the segments, initialize the local display buffer as well
	for(i = 0; i < 20; i++) {
		result |= local_I2C_WriteByte(0xFF);
		lcdBitmap[i] = 0xFF;
	}	
	
	local_I2C_Stop();
	
	return (result ? -1 : 0);
}



/******************************************************************************
* Description: LCD1x9_Wipe(..) - initializes pins and registers of the LCD1x9
*				Clear all segments
* Input: 	none
* Output: 	none
* Return:	0 if sucessfully initialized, -1 if error occured 
*******************************************************************************/
char LCD1x9_Wipe(void)
{
	char result = 0, i;
	
	// the idea is to toggle pins as inputs/outputs to emulate I2C open drain mode of operation!	
	UEXT_SDA_TRIS = 1;
	UEXT_SCL_TRIS = 1;
	
	UEXT_SDA_LAT = 0;
	UEXT_SCL_LAT = 0;
	
	local_I2C_Delay();
	if( !(UEXT_SCL_PORT && UEXT_SDA_PORT) )
		return -1;

	local_I2C_Start();
	result |= local_I2C_WriteByte(LCD1x9_SLAVE_ADDR | 0x00);
	result |= local_I2C_WriteByte(0b11001000); // mode register
	result |= local_I2C_WriteByte(0b11110000); // blink register
	result |= local_I2C_WriteByte(0b11100000); // device select register
	result |= local_I2C_WriteByte(0b00000000); // pointer register
	
	// light up all the segments, initialize the local display buffer as well
	for(i = 0; i < 20; i++) {
		result |= local_I2C_WriteByte(0x00);
		lcdBitmap[i] = 0x00;
	}	
	
	local_I2C_Stop();
	
	return (result ? -1 : 0);
}





/******************************************************************************
* Description: LCD1x9_enableSegment(..) - enables a segment in the display buffer
*		Note: Does not actually light up the segment, have to call the 'LCD1x9_Update(..)'
* Input: 	comIndex - backplate index
*			bitIndex - segment index
* Output: 	none
* Return:	none
*******************************************************************************/
void LCD1x9_enableSegment(BYTE comIndex, BYTE bitIndex)
{
	if(bitIndex >= 40)
		return;
		
	comIndex &= 0x3;
	
	if(bitIndex & 0x1)
		comIndex |= 0x4;
		
	bitIndex >>= 1;
	
	lcdBitmap[bitIndex] |= 0x80 >> comIndex;
}

/******************************************************************************
* Description: LCD1x9_disableSegment(..) - disables a segment in the display buffer
*		Note: Does not actually lights off the segment, have to call the 'LCD1x9_Update(..)'
* Input: 	comIndex - backplate index
*			bitIndex - segment index
* Output: 	none
* Return:	none
*******************************************************************************/
void LCD1x9_disableSegment(BYTE comIndex, BYTE bitIndex)
{
	if(bitIndex >= 40)
		return;
		
	comIndex &= 0x3;
	
	if(bitIndex & 0x1)
		comIndex |= 0x4;
		
	bitIndex >>= 1;
	
	lcdBitmap[bitIndex] &= ~(0x80 >> comIndex);
}

/******************************************************************************
* Description: LCD1x9_Update(..) - disables a segment in the display buffer
*		Note: Does not actually lights off the segment, have to call the 'LCD1x9_Update(..)'
* Input: 	comIndex - backplate index
*			bitIndex - segment index
* Output: 	none
* Return:	none
*******************************************************************************/
void LCD1x9_Update(void)
{
	BYTE i;
	
	local_I2C_Start();
	local_I2C_WriteByte(LCD1x9_SLAVE_ADDR | 0x00);
	local_I2C_WriteByte(0b11100000); // device select register
	local_I2C_WriteByte(0b00000000); // pointer register
	
	// send the local buffer to the device
	for(i = 0; i < 20; i++)
		local_I2C_WriteByte(lcdBitmap[i]);
		
	local_I2C_Stop();
}

/******************************************************************************
* Description: LCD1x9_Write(..) - writes a string to the display
* Input: 	string - the string to write, no more than 9 characters
*			bitIndex - segment index
* Output: 	none
* Return:	none
*******************************************************************************/
void LCD1x9_Write(char *string)
{
	BYTE data, length, index, i;
	WORD bitfield;
	BYTE com, bit;
	
	length = strlen(string);
	if(length > 9)
		return;
	
	index  = 0;
	/* fill out all characters on display */
	for (index = 0; index < 9; index++) {
		if (index < length) {
			data = (BYTE)string[index];
		} else {
			data = 0x20; // fill with spaces if string is shorter than display
		}

		data -= 0x20;
		bitfield = LCDAlphabet[data];
	
		for (i = 0; i < 16; i++) {
			bit = LCD1x9.Text[index].bit[i];
			com = LCD1x9.Text[index].com[i];
		
			if (bitfield & ((WORD)1 << i)) {
				LCD1x9_enableSegment(com, bit);
			} else {
				LCD1x9_disableSegment(com, bit);
			}
		}
	}

	LCD1x9_Update();
}


/* local functions */

static void local_I2C_Delay(void)
{
//pg15apr	int d = 3000; //pg 26mar 100 gave 150us per cycle
	int d = 300; //pg15apr
    while(d--) {
		Nop();
	}	
}	

static void local_I2C_Start(void)
{
	UEXT_SDA_TRIS = 0;
	local_I2C_Delay();
	UEXT_SCL_TRIS = 0;
	local_I2C_Delay();
	
}

static void local_I2C_Stop(void)
{
	UEXT_SDA_TRIS = 0;
	local_I2C_Delay();
	UEXT_SCL_TRIS = 1;
	local_I2C_Delay();
	UEXT_SDA_TRIS = 1;
	local_I2C_Delay();
}

// returns ack state, 0 means acknowledged
static char local_I2C_WriteByte(BYTE data)
{
	char i;

	// send the 8 bits
	for(i = 0; i < 8; i++) {
		UEXT_SDA_TRIS = (data & 0x80) ? 1 : 0;
		data <<= 1;
		local_I2C_Delay();
		UEXT_SCL_TRIS = 1;
		local_I2C_Delay();
		UEXT_SCL_TRIS = 0;
	}
	
	// read the ack
	UEXT_SDA_TRIS = 1;
	local_I2C_Delay();
	UEXT_SCL_TRIS = 1;
	local_I2C_Delay();
	i = UEXT_SDA_PORT;
	UEXT_SCL_TRIS = 0;
	local_I2C_Delay();
	
	return i;
}



static unsigned char local_I2C_ReadByte(char ack)
{
	unsigned char data = 0;
	char i;
	
	UEXT_SDA_TRIS = 1;
	for(i = 0; i < 8; i++) {
		local_I2C_Delay();
		UEXT_SCL_TRIS = 1;
		local_I2C_Delay();
		data |= UEXT_SDA_PORT & 0x01;
		if(i != 7)
			data <<= 1;
		UEXT_SCL_TRIS = 0;
	}
	
	// read the ack
	local_I2C_Delay();
	UEXT_SDA_TRIS = ack;
	local_I2C_Delay();
	UEXT_SCL_TRIS = 1;
	local_I2C_Delay();
	UEXT_SCL_TRIS = 0;
	local_I2C_Delay();
	
	return data;
}

//For Olimex IO board /////////////////////////////
//For Olimex IO board /////////////////////////////
//For Olimex IO board /////////////////////////////
//For Olimex IO board /////////////////////////////

#define MODIO_SLAVE_ADDR 0x58
#define I2C_GET_DINPUTS  0x20
#define I2C_GET_AIN_0 0x30
#define I2C_GET_AIN_1 0x31
#define I2C_GET_AIN_2 0x32
#define I2C_GET_AIN_3 0x33

/******************************************************************************
* Description: MODIO_ReadDINs(..) - reads the digital inputs of the board
* Input: 	none
* Output: 	none
* Return:       0 on success, -1 on error
*******************************************************************************/
char MODIO_ReadDINs(BYTE *data)
{
	char result = 0;
	BYTE bitmap;

	do {
		local_I2C_Start();
		result |= local_I2C_WriteByte( (2*MODIO_SLAVE_ADDR) | 0x00);
		result |= local_I2C_WriteByte(I2C_GET_DINPUTS);
		local_I2C_Stop();
		if(result) break;

		local_I2C_Start();
		result |= local_I2C_WriteByte( (2*MODIO_SLAVE_ADDR) | 0x01);
		bitmap = local_I2C_ReadByte(NACK);
		local_I2C_Stop();
		if(result) break;
		
		*data = bitmap;
	} while(0);

	return result;
}

/******************************************************************************
* Description: MODIO_ReadAIN(..) - reads an analog input of the board
* Input: 	channel - analog inpout to scan
* Output: 	data - value of the input level
* Return:       0 on success, -1 on error
*******************************************************************************/
char MODIO_ReadAIN(unsigned int *data, BYTE channel)
{
	char result = 0;
	int val;
    unsigned int lbyte;
    unsigned int hbyte;
//    char lcd[9];
    int i;
    
    
	do {
		local_I2C_Start();
		result |= local_I2C_WriteByte( (2*MODIO_SLAVE_ADDR) | 0x00);
		result |= local_I2C_WriteByte(I2C_GET_AIN_0 + (channel & 0x03));
		local_I2C_Stop();
		if(result) break;

		local_I2C_Start();
		result |= local_I2C_WriteByte( (2* MODIO_SLAVE_ADDR) | 0x01);
		lbyte = (unsigned int) local_I2C_ReadByte(ACK);
		hbyte = (unsigned int) local_I2C_ReadByte(NACK);
		local_I2C_Stop();
		if(result) break;


	} while(0);

/*        for (i=100000; i; i--);    
      sprintf(lcd,"%d %d",hbyte, lbyte);
      LCD1x9_Write(lcd);
          for (i=100000; i; i--);     
  */    
    
    *data = (unsigned int) (hbyte*(unsigned int)256) + (unsigned int)lbyte;
    
	return result;
}





static void IO_RelaysOn(BYTE relaybits)
{
    local_I2C_Start(); //Send start condition
    local_I2C_WriteByte(0xb0); //This is 0x58 shifted to left one time and added 0 as W
    local_I2C_WriteByte(0x10); //Command to set relays
    local_I2C_WriteByte(relaybits); //0x05 ? 0b00000101 ? This way we will set REL1 and REL3
    local_I2C_Stop(); //Send stop condition
};

//For Olimex IO board relays off
static void IO_RelaysOff(void)
{
    local_I2C_Start(); //Send start condition
    local_I2C_WriteByte(0xb0); //This is 0x58 shifted to left one time and added 0 as W
    local_I2C_WriteByte(0x10); //Command to set relays
    local_I2C_WriteByte(0x00); //0x00 ? 0b00000000 ? This way we will release REL1 and REL3
    local_I2C_Stop(); //Send stop condition
};


static void getbrivol()
{
   unsigned int  i, adata;
   MODIO_ReadAIN(&adata, 0);  //channel 0 = right-hand pot
   volume = (16*adata)/1024;
   if (volume<0) volume=0;
   if (volume>15) volume=15;
   MODIO_ReadAIN(&adata, 1);  //channel 1 = left-hand pot
   brightness = (16*adata)/1024;
   if (brightness<0) brightness=0;
   if (brightness>15) brightness=15;
   

    dataSPI = 0x0A00+brightness;  // address4bits data8bits ! 0= min brightness1/32 F=31/32
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

}


//7219 setup
static void Setup7219(int briteness)
{
    int i;
        // 7219 setup
    dataSPI = 0x0F00;  // address4bits data8bits ! display test off
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

    dataSPI = 0x0F00;  // address4bits data8bits ! display test off (repeat it))
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

    
    
    dataSPI = 0x0A00+briteness;  // address4bits data8bits ! 0= min brightness1/32 F=31/32
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

    dataSPI = 0x0B07;  // address4bits data8bits ! scan all 8 rows
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

    dataSPI = 0x0900;  // address4bits data8bits ! nodecodemode
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

    dataSPI = 0x0C01;  // address4bits data8bits ! normaloperationmode
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);    

};

static void getdin() {
   MODIO_ReadDINs(&ddata);
   
 //  sprintf(lcd,"%d",ddata);  seems ok 18may
 //  LCD1x9_Write(lcd);
   
   cusr=0;
   cusl=0;
   probe=0;
   if (ddata&8) cusl=1;
   if (ddata&4) cusr=1;
   if (ddata&2) probe=1;   
};
   


//defined at end because it refers to restart label

static int wait1s() {
    int i;
    int j;
    int k;
    int m;
    
 for (i=0; i<240; i++) {   //calibrated to 1second 18may
 if (START_PORT==0) return(1); //signal to goto restart what a PAIN
 for (j=0; j<240; j++) {
 for (k=0; k<240; k++) {
     m++;
 }  
 }
 }
    return(0);
}


static void wait1snostart() {
    int i;
    int j;
    int k;
    int m;
    
 for (i=0; i<240; i++) {   //calibrated to 1second 18may
 for (j=0; j<240; j++) {
 for (k=0; k<240; k++) {
     m++;
 }  
 }
 }
}

static void wait100ms() {
    int i;
    int j;
    int k;
    int m;
    
 for (i=0; i<24; i++) {   //calibrated to 100ms 18may
 for (j=0; j<240; j++) {
 for (k=0; k<240; k++) {
     m++;
 }  
 }
 }
}
    
static void wait10ms() {
    int i;
    int j;
    int k;
    int m;
    
 for (i=0; i<2; i++) {   //?? maybe about 10ms 18may
 for (j=0; j<240; j++) {
 for (k=0; k<240; k++) {
     m++;
 }  
 }
 }
}

 int checkcell(int ii,int jj) {
     if (ii<0) return(0);
     if (jj<0) return(0);
     if (ii>15) return(0);
     if (jj>71) return(0);
     if (spimat[ii][jj]>0) return(1);
     return(0);
 }



void life() {
    
    sprintf(spistr," SPEEDWIRE  Life Conway "); //24char long
    SPI_String(spistr,0,0); //col,row
    
    int ilife=0;
    BYTE tempmat[16][72];
    int i,j;
    int ii,jj,kk;

    for (i=0; i<10; i++) {
        wait100ms();
        if (START_PORT==0) break;
    }
    
    for (ilife=0; ilife<300; ilife++) {

        if (ilife==1)  
        {  // pause to show random start pattern
            for (i=0; i<6; i++) {
                wait100ms();
                if (START_PORT==0) break;
            }
        }
        
        if (ilife==0) {
            //random fill spimat
            int rfill;
            for (i=0; i<16; i++) {
                for (j=0; j<72; j++) {
                    rfill= rand()>>29;   // 0/1/2/3
                    spimat[i][j]=0;
                    if (rfill==2) spimat[i][j]=1;
                }
            }
        }    
        
        if (ilife>0) { //don't evolve the start pattern!    
            wait10ms();
            wait10ms();
            wait10ms();
            wait10ms();
            wait10ms();
            if (START_PORT==0) break;
            //static BYTE spimat[8*SPIROWS][8*N8X8/SPIROWS]; // wastefully 1 byte per dot
            for (i=0; i<16; i++) {
                for (j=0; j<72; j++) {
                    //count neighbours. off edge =0
                    kk=0;
                    ii=i+1; jj=j;   kk+=checkcell(ii,jj);
                    ii=i+1; jj=j-1; kk+=checkcell(ii,jj);
                    ii=i;   jj=j-1; kk+=checkcell(ii,jj);
                    ii=i-1; jj=j-1; kk+=checkcell(ii,jj);
                    ii=i-1; jj=j;   kk+=checkcell(ii,jj);
                    ii=i-1; jj=j+1; kk+=checkcell(ii,jj);
                    ii=i;   jj=j+1; kk+=checkcell(ii,jj);
                    ii=i+1; jj=j+1; kk+=checkcell(ii,jj);
                    if (kk<2)  tempmat[i][j]=0;
                    if (kk==2) tempmat[i][j]=spimat[i][j];
                    if (kk==3) tempmat[i][j]=1; 
                    if (kk>3)  tempmat[i][j]=0;
                }
            }
            //copy back to display matrix
            for (i=0; i<16; i++) {
                for (j=0; j<72; j++) {
                     spimat[i][j]=tempmat[i][j];
                }
            }
        } //endif ilife>0
    
/*
Any live cell with fewer than two live neighbours dies, as if caused by underpopulation.
Any live cell with two or three live neighbours lives on to the next generation.
Any live cell with more than three live neighbours dies, as if by overpopulation.
Any dead cell with exactly three live neighbours becomes a live cell, as if by reproduction.
*/    
    
    
    // take the pixel array and play LIFE with it
    
    // 2. rearrange spimat to commands for 7219 in dmatSPI
//send rightmost column first <<<this doesn't matter in step 2.
//so that's the end of SPIROWS    
    // the 8x8s update by columns
    // BYTE spimat[8*SPIROWS][8*N8X8]
    int irow, icol;
    int coladdr7219=1; 
    int matrow=0;
    int matcol=0;
    for (icol=8*N8X8-1; icol>=0; icol--)
    {
        int rowval=128;  //sendvalue start at top row
        int sendval=0;

        //upper or lower row of the 2-row N8X8 layout
        matrow=0;
        matcol=0;
        if (icol>=(8*N8X8/SPIROWS)) {
            matrow=1;
            matcol=1;  //flag to move left!
        }
        for (irow=0; irow<8; irow++)
        {
            if (spimat[irow+8*matrow][icol-matcol*(8*N8X8/SPIROWS)]!=0) sendval=sendval+rowval;
            rowval=rowval/2;       

        } //next irow    

    dmatSPI[icol] = 256*coladdr7219+sendval;  // address4bits data8bits ! 7219datasheet
    coladdr7219=coladdr7219+1;
    if (coladdr7219>8) coladdr7219=1;
            
    } //next icol    
        
    //3. send dmatSPI. note same reg to all 7219s so altered order here!
    // now we really do have to send rightmost column first!
            int colin7219;
    for (colin7219=1; colin7219<=8; colin7219++) //the column within each 7219 1=rightcol       
    {  
        ii=0; // which 8*8 (0 is the furthest to the right)
        for (icol=8*N8X8-colin7219; icol>=0; icol=icol-8) // because colin starts = 1!
        {
            darraySPI[ii]=dmatSPI[icol];  //send rightmost first!!
            //darraySPI[ii]=256*colin7219+colin7219+ii;  //send rightmost first!!
            ii++;
        }
        SPI_Send(darraySPI);
        for (i=15000; i; i--);    
    } // next colin7219 address within each 8X8    

} //next ilife
        
}





void stripoff() {
    for (sled=0; sled<144; sled++) {
       stripr[sled]=0;   
       stripg[sled]=0;   
       stripb[sled]=0;          
       }
   stripfire(); 
} 

void stripredmax(brightness) {
    for (sled=0; sled<144; sled++) {
       stripr[sled]=70+12*brightness;   
       stripg[sled]=0;   
       stripb[sled]=0;          
       }
   stripfire(); 
} 

void stripgreenmax(brightness) {
    for (sled=0; sled<144; sled++) {
       stripr[sled]=0;   
       stripg[sled]=70+12*brightness;   
       stripb[sled]=0;          
       }
   stripfire(); 
} 

void stripbluemax(brightness) {
    for (sled=0; sled<144; sled++) {
       stripr[sled]=0;   
       stripg[sled]=0;   
       stripb[sled]=70+12*brightness;          
       }
   stripfire(); 
} 

void stripwhitemax(brightness) {
    for (sled=0; sled<144; sled++) {
       stripr[sled]=50+10*brightness;    // don't risk full power
       stripg[sled]=50+10*brightness;   
       stripb[sled]=50+10*brightness;          
       }
   stripfire(); 
} 

void striprandom(int gap) {
    int rv,gv,bv,sv;
    sv = ( gap * (rand()>>23 ) ) / 256;
    for (sled=0; sled<144; sled++) {
       stripr[sled]=0;   
       stripg[sled]=0;   
       stripb[sled]=0;          
       }
    for (sled=sv; sled<144; sled+=gap) {
       rv = rand()>>23;
       gv = rand()>>23;
       bv = rand()>>23;
       stripr[sled]=rv;   
       stripg[sled]=gv;   
       stripb[sled]=bv;          
       }
   stripfire(); 
} 

void beep(int dur, int vol, int note)  {  // note 32 = mid C  44= C1
// dur 100 is about 1 sec long
    int i,j,k; //     10000 is roughly C1
//    f  =   2,pow(note*1/12)  * note * Const
//    T = 1/f
//  k = CONST/  pow (2, note * 1/12)
//  k = CONST * pow (2, -note*1/12)
    
    k = (int) 10000 * (  pow (2, -(double)note/12)  /  pow (2, -50/12)  ) ;
           
    int durvolon, durvoloff;
    durvolon = 10*(vol * dur) / 16;
    durvoloff = 10*dur - durvolon;
    durvolon = durvolon*10000/k;
    durvoloff= durvoloff*10000/k;
    
    for (i=durvolon; i>0; i--) {
            for (j=k; j>0; j--) { };
                SPKR_PORT=1;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
        }
    for (i=durvoloff; i>0; i--) {
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
        }
}

void chirp(int dur, int vol) {
    int durvolon, durvoloff;
    int i,j, k=20000;
    durvolon = 10*(vol * dur) / 16;
    durvoloff = 10*dur - durvolon;
    
    for (i=durvolon; i>0; i--) {
            k=k*0.95;
            for (j=k; j>0; j--) { };
                SPKR_PORT=1;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
        }
    k=20000;  //to hold overall duration if volume is low
    for (i=durvoloff; i>0; i--) {
            k=k*0.95;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
        }
}


void raspbeep(int dur, int vol, int note) {
    int durvolon, durvoloff;
    int i,j, k=40000;
    durvolon = 8*(vol * dur) / 16;
    durvoloff = 8*dur - durvolon;
    int m;
    
    for (i=durvolon; i>0; i--) {
            k=k*0.95;
            for (j=k; j>0; j--) { for (m=0; m<15; m++) {}; };
                SPKR_PORT=1;
            for (j=k; j>0; j--) { for (m=0; m<15; m++) {}; };
                SPKR_PORT=0;
        }
    k=20000;  //to hold overall duration if volume is low
    for (i=durvoloff; i>0; i--) {
            k=k*0.95;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
            for (j=k; j>0; j--) { };
                SPKR_PORT=0;
        }
}


////////////////////////////////////////////////////////

void meter(int lives) {                
       if (lives==0) OC4RS=0;
       if (lives==1) OC4RS=4096+410;
       if (lives==2) OC4RS=2*4096+380;
       if (lives==3) OC4RS=3*4096+280;       
       if (lives==4) OC4RS=4*4096+180;
       if (lives==5) OC4RS=5*4096+80;
       if (lives>5)  OC4RS=0x1000*lives;
}

/////////////////////////////////////////////////////
//write record array to NVRAM
void writenvrecord() {   

    int iii=0;
    int i=0;
    int ttt;
    
    // ERASE NEEDED BEFORE WRITE    
    // set destination page address
    // page physical address
    NVMADDR = (unsigned int) 0x1D1F0000;
    // define Flash operation
    NVMCONbits.NVMOP = 0x4; // NVMOP for Page Erase
    // Enable Flash Write
    NVMCONbits.WREN = 1;
    // commence programming
    int int_status;
    asm volatile("di  %0" : "=r" (int_status) );
    NVMKEY = 0x0;
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
    NVMCONSET = 1 << 15;
    if(int_status & 0x00000001) 
    {
        asm volatile("ei");
    }
    // Wait for WR bit to clear
    while(NVMCONbits.WR);
    // Disable future Flash Write/Erase operations
    NVMCONbits.WREN = 0;
    // Check Error Status
    //if(NVMCON & 0x3000)// mask for WRERR and LVDERR bits{ // process errors}

    // END ERASE CODE    
    
    wait10ms();
    
    for (iii=0; iii<8; iii++) {
        NVMDATA0=record[iii];
        NVMADDR = (unsigned int) 0x1D1F0000 + (unsigned int) (4*iii);
        NVMCONbits.WREN=0;    //ex52-2 in 52_FLASHMem PDF
        NVMCONbits.NVMOP=1;
        NVMCONbits.WREN=1;
        int int_status;
        asm volatile("di  %0" : "=r" (int_status) );
        NVMKEY = 0x0;
        NVMKEY = 0xAA996655;
        NVMKEY = 0x556699AA;
        NVMCONSET = 1 << 15;
        if(int_status & 0x00000001) 
        {
            asm volatile("ei");
        }
        while (NVMCONbits.WR);
        NVMCONbits.WREN=0;              
        wait10ms();
    } //next iii i.e. next record
    
    //
}  














//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//   MAIN 
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

int main()
{
    long int i;
    long int j=1;
    BYTE ddata =0;
    unsigned int  adata;
    char ack;
    int lives=0;
    int nlives=0;
    int golr=0;
    int gorl=0;
    char tstr[8];
    char lstr[8];

    wait1snostart(); // let everything settle!

    for (i=0; i<8; i++) record[i]=99;  //ready to init NVRAM if START pressed on powerup

    // PORT D SETUP
    START_TRIS=1;
    LIV0_TRIS=1;
    LIV1_TRIS=1;
    LIV2_TRIS=1;
    SPKR_TRIS=0;   // it is AC coupled!

   
    // LED init
    LED_TRIS = 0;   // LED set as output
    LED_LAT=0;


    // set timer 1 to run in /10th seconds
    // it runs off SOSC 32768Hz            
    T1CONbits.ON=1;
    T1CONbits.TGATE=0;
    T1CONbits.TCS=1;
    T1CONbits.TCKPS=3; //  32768Hz/256 = 128Hz = 7.8125ms increments.
    
    //TEST k0 BIT  HOW FAST????? 175ns per cycle
    // odd nowhere near 180MHz or even 90MHz???
    //try assembler in here????
    /*K0_TRIS=0;
    loopyk0:
    K0_LAT=0;
    K0_LAT=1;
    goto loopyk0;
    */
   
    //LED Strip setup 
    spval=0; // colour test variable only
    sledstart=0; // alt alternate LEDs
    //STRIP DRIVE to OUTPUT and sit low.
    K0_LAT=0;
    K0_TRIS=0;
    
    //setup quiet state of SPI
    SPINCS_LAT =1;
    SPINCS_TRIS=0;
    SPICLK_LAT=0;
    SPICLK_TRIS=0;
    SPIDAT_LAT=0;
    SPIDAT_TRIS=0;

    wait10ms();
    Setup7219(4);  //arb brightness don't know it yet
    wait10ms();
    sprintf(spistr,"                        "); //SH is 24 char long
    SPI_String(spistr,0,0); //col,row
    wait10ms();
    getbrivol();
    wait10ms();
    
    LCD1x9_Initialize();
    LCD1x9_Wipe();
    int SPIrcoff=0; 
    int relaybits=1;

    IO_RelaysOff();

    
    // retrieve records from NVRAM
    
    //read VMs and see if it went into FLASH
    unsigned int ttt;
    unsigned int iii;
    unsigned int kkk=0;

    if (START_PORT==0) {  //reset records in NVRAM
        sprintf(spistr,"RESETTING   record vals "); //24char long
        SPI_String(spistr,0,0); //col,row
        writenvrecord(); //record array is global variable
        wait100ms(); wait100ms(); wait100ms(); wait100ms(); wait100ms();
    }
    
    
    if (START_PORT==1) {
        sprintf(spistr,"Loading     record vals "); //24char long
        SPI_String(spistr,0,0); //col,row
        wait100ms(); wait100ms(); wait100ms(); wait100ms(); wait100ms();
        for (iii=0; iii<8; iii++) {
            ttt =  (unsigned int)  ( *  ( (unsigned int *) (0xBD1F0000+4*iii) ) );
            record[iii]=ttt;
            sprintf(spistr,"Lives %2d    Record %3d  ",iii+1,ttt); //24char long
            SPI_String(spistr,0,0); //col,row
            wait100ms(); wait100ms();
        } // next iii
    } //endif START_PORT==1
    

    
    
    getbrivol();  //???why not working????
 

   //PB3DIV = //drives all timers default is SYSCLK/2 which is 90MHz I think
   // if I prescale that in T2CON by 256 it's 352kHz per bit
   T2CONbits.ON=1;
   T2CONbits.TCKPS=1; // divide by 1
   T2CONbits.TCS=0;
   T2CONbits.TGATE=0;
   T2CONbits.T32=0;
   OC4RS=0;
   T2CONbits.TON=1;
   
   PR2=0x8000;  // TMR2 preset 0x8000  90MHz / 32768  = 2.75kHz
   
   //PWM setup to drive meter 
   METER_TRIS=0;
   METER_RPD=0x0B;  // OC4
   //METER_LAT LATDbits.LATD11
   //PWM from TIMER2
   CFGCONbits.OCACLK=0;   // table 18.1 page 300 of 32mz2048ecg144 data
   //CFGCONbits.OCTSEL=0;   // table 18.1 page 300 of 32mz2048ecg144 data
   OC4CONbits.ON=1;
   OC4CONbits.OC32=0; //16-bit compare mode OC4R and OC4RS against TIMER2
   OC4CONbits.OCM=6;
   OC4CONbits.OCTSEL=0;
   
   //OC4RS should now control the PWM running off TIMER2 !


    //beep to indicate entering mainloop    

    beep(12,volume,32);

    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    // MAIN LOOP /////////////////////////////////////////////
    
    
    restart:  //if start pressed during run or during 1s delay

 /*   sprintf(spistr,"4debug      atRESTART   "); //24char long
    SPI_String(spistr,0,0); //col,row

    wait1snostart(); 
    
    sprintf(spistr,"4debug      intoIDLELOOP"); //24char long
    SPI_String(spistr,0,0); //col,row
    
    wait1snostart(); 
   */
            
    while (START_PORT==0) {
        sprintf(spistr,"OK  Release START, WAIT!"); //24char long
        SPI_String(spistr,0,0); //col,row
        wait1snostart(); //don't let it jump to RSGLOOP!
        }

    
    //took this out 11jun {
    int ilco=0;
    int ilcorate=80;
    int delta=0;
    int probesr=0;
    int probesl=0;
    int sledstart=0;
    int ilcoco=0;
    
    srand( TMR1 & 255 );
    
    idle_loop:
    ilco++; if (ilco>2*ilcorate) {
        ilco=0;
        ilcoco++;
    }
    wait10ms();        // no point hammering it
    nlives = 4*LIV2_PORT + 2*LIV1_PORT + LIV0_PORT +1;
    meter(nlives);

    getbrivol();
    
    // IDLE WAITING FOR START BUTTON PUSH
    // if any copper is low use that to light red leds at that end    
    // show silly effects etc and show current leaders     
    // monitor volume control and brightness control    
    if (ilco==ilcorate) {

        // flash strip to get attention
        int flashcol;
        flashcol = rand()>>29;   // 0/1/2/3
        if (flashcol==0) stripredmax(brightness);
        if (flashcol==1) stripgreenmax(brightness);
        if (flashcol==2) stripbluemax(brightness);
        if (flashcol==3) striprandom(5);
        sprintf(spistr,"Speedwire   Push START  "); //24char long
        SPI_String(spistr,0,0); //col,row
        wait10ms(); wait10ms(); wait10ms(); wait10ms(); wait10ms();
        stripoff();
    }

    if (ilco==2*ilcorate)  {  
        
        
    sprintf(spistr,""); //SH is 24 char long
    if (nlives==1) sprintf(tstr,"%2ds  1 life ",record[nlives-1]); //SH is 24 char long
    if (nlives>1)  sprintf(tstr,"%2ds  %1d lives",record[nlives-1],nlives); //SH is 24 char long
    sprintf(spistr,"Record time %s", tstr); //24char long
    SPI_String(spistr,0,0); //col,row
    }

    // find probe end & light red leds there if either copper touched
    //remember which end in case it is intermittent, people messing with probe
    getdin();
    delta=0;
    if (cusr==1) {probesr=1; probesl=0; gorl=1; golr=0; delta=1;}
    if (cusl==1) {probesl=1; probesr=0; gorl=0; golr=1; delta=1;}
    
    //sprintf(lcd,"%d %d %d %d %d",probesl, probesr,golr,gorl,probe); 
    //LCD1x9_Write(lcd);
    
 /*   if (probe==1) { //blip the bell 
            if (volume>0) {IO_RelaysOn(8);}
            wait10ms();
            IO_RelaysOff(); 
            }
*/
    //light strip red led at that end
    if (delta=1) {
        delta=0;    
        spvalr = 16+8*brightness;
        for (sled=0; sled<144; sled++) {
           stripr[sled]=0;   
           if (probesl==1 && sled<3) {
               stripr[sled]=spvalr;   
           }
           if (probesr==1 && sled>140) {
               stripr[sled]=spvalr;   
           }
           stripg[sled]=0;   
           stripb[sled]=0;          
        }
        //SEND TO STRIP
        stripfire();   // the variables are all global
    } //endif delta=1
    
    
    //Speedwire is feeling bored. no-one's playing. attract attention
    if (ilcoco==4)
    {
        ilcoco=0;
        int attsel;
        attsel = rand()>>29;   // 0/1/2/3
        
    if (attsel==0) {
        int i,j,k;
        for (i=0; i<30;i++) {
            j = (rand()>>28) -3;  // -3...+3
            k = (rand()>>28) -3;  // -3...+3
            sprintf(spistr,"Wanna Try ?             "); //24char long
            SPI_String(spistr,3+j,4+k); //col,row
            striprandom(6);
            wait10ms(); wait10ms(); wait10ms(); wait10ms(); wait10ms();
            if (START_PORT==0) goto restart;
          }         
        stripoff();
    }  //endif attsel==0
    
    if (attsel==1) {
        life();
    }
        
    } // endif ilcoco
       
    /////////////////////////////
    LED_LAT=!LED_LAT; // blink green LED

    if (START_PORT==1) goto idle_loop;

    // END OF IDLE LOOP //////////////////////////////////////
    // END OF IDLE LOOP //////////////////////////////////////
    // END OF IDLE LOOP //////////////////////////////////////    
    ilcoco=0; //reset boredom counter
    
    
    
    // START was pressed
       
    
    
    sprintf(spistr,""); //SH is 24 char long
    sprintf(spistr,"TOUCH COPPERWAIT FOR GO ", lives);
    SPI_String(spistr,0,0); //col,row

    wait1snostart();
    wait1snostart();
    if (START_PORT==0) goto restart; //allow reset
    
    // If probe position is still unknown, IDLE WAITING FOR TOUCH COPPER    
    probeposloop:
    
        // find probe end & light red leds there if either copper touched
        //remember which end in case it is intermittent, people messing with probe
        getdin();
        if (cusr==1) {probesr=1; probesl=0; gorl=1; golr=0;}
        if (cusl==1) {probesl=1; probesr=0; gorl=0; golr=1;}
        //light strip red led at that end. 8-bits/LED
        spvalr = 16+8*brightness;    //brightness goes 0-16  
        for (sled=0; sled<144; sled++) {
           stripr[sled]=0;   
           if (probesl==1 && sled<3) {
               stripr[sled]=spvalr;   
           }
           if (probesr==1 && sled>140) {
               stripr[sled]=spvalr;   
           }
           stripg[sled]=0;   
           stripb[sled]=0;          
           }
        //SEND TO STRIP
        stripfire();   // the variables are all global
    
    if (probesr==0 && probesl==0) {goto probeposloop;}
    
    
    

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////    
//           READY SET GO
//           READY SET GO
//           READY SET GO
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
    rsg_loop:

    // READY SET GO
    // TBD FOUL??? too fussy may confuse people:if COPPER released before GO

        //send to SPI 8x8 display 
        sprintf(spistr,""); //DH is 6 char long
        sprintf(spistr,"READY!");
        SPI_DHString(spistr,SPIrcoff,SPIrcoff); //col,row
     
        beep(70,volume,12);
        getbrivol();
        if (wait1s()) goto restart;

        //light strip amber led at that end
        spvalr = 16+8*brightness;
        spvalg = spvalr/3;
        for (sled=0; sled<144; sled++) {
           stripr[sled]=0;   
           stripg[sled]=0;   
           if (probesl==1 && sled<3) {
               stripr[sled]=spvalr;   
               stripg[sled]=spvalg;   
           }
           if (probesr==1 && sled>140) {
               stripr[sled]=spvalr;
               stripg[sled]=spvalg;   
           }
           stripb[sled]=0;          
           }
        //SEND TO STRIP
        stripfire();   // the variables are all global

        //send to SPI 8x8 display 
        sprintf(spistr,""); //DH is 6 char long
        sprintf(spistr," SET! ");
        SPI_DHString(spistr,SPIrcoff,SPIrcoff); //col,row
        beep(80,volume,13);
        getbrivol();
        if (wait1s()) goto restart;
        //send to SPI 8x8 display 
        sprintf(spistr,""); //DH is 6 char long
        sprintf(spistr,"GO  !!");
        SPI_DHString(spistr,SPIrcoff,SPIrcoff); //col,row
        if (START_PORT==0) goto restart; // allow reset
        chirp(1500,volume);
        getbrivol();
        wait100ms();
        if (START_PORT==0) goto restart; //allow reset
        
/////////////////////////////////////////////////////////////////////
////     GO LOOP    /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////    
    int t;
    int timeup=0;
    int dtouchco=5;
    int touchco=0;
    int tflag=0;
        
    TMR1=1;  //because we had 1 sec delay
    t=0;
    lives=nlives;
    
    // loop iteration time needs to be quite fast to check for touches  
    int goct=0;
    int goct2=0;
    gook_loop:    
    LED_LAT=!LED_LAT; // blink green LED for info on iterate
    goct++;        
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////    
            
    // TMR1 and PR1 are 16-bit registers
    // 65536 * 0.0078125 = 512 seconds = 8minutes 32 seconds max play time!!!
    t = TMR1 / 128;
    if (t>100) timeup=1;
    
    meter(lives);
    
    // LED strip show scrolling dots    
    // LED strip speed, accelerate with time!!!
    int accelt=0;
    if (record[nlives-1]>0) accelt = (7 * t) / record[nlives-1];
    if (accelt>7) accelt=7;
    if (goct> (10-accelt))  // need to cal this against loop iteration time
    {
        goct=0;
        if (lives>1) {
            spvalr=0;
            spvalg=16+8*brightness;
            spvalb=0;
        }
        if (lives==1) {
            spvalr=16+8*brightness;
            spvalg=16+8*brightness;  //pg2022 was 4*brightness
            spvalb=0;
        }

        for (sled=0; sled<144; sled++) {
           stripr[sled]=0;   
           stripg[sled]=0;   
           stripb[sled]=0;          
        }

        int sledgap=5;
        if (golr==1) {
            sledstart=sledstart+1;
            if (sledstart>=sledgap) sledstart=0;
        }
        if (gorl==1) {
            sledstart=sledstart-1;
            if (sledstart<0) sledstart=sledgap-1;
        }

        for (sled=sledstart; sled<144; sled+=sledgap) {
           stripr[sled]=spvalr;   
           stripg[sled]=spvalg;   
           stripb[sled]=spvalb;          
           }

        stripfire(); 

        getbrivol();

    //reset 8x8s in case of spike
    dataSPI = 0x0C00;  // address4bits data8bits ! shutdown
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);  

    Setup7219(brightness);  //try to reset 7219s no does not work
    wait10ms();

        
    } // endif on goct strip update rate

    
    
    // 8*8 show time/record and lives
    sprintf(spistr,""); //SH is 24 char long
    sprintf(tstr,"%2ds/%2ds ",t, record[nlives-1]);
    sprintf(lstr," %1d  ",lives);
    sprintf(spistr,"Time/R Lives%s%s", tstr,lstr);
    SPI_String(spistr,0,0); //col,row
    
    // monitor for touches and end
    // touchco is an integrator to blur over contact noise
 
    getdin();
    if (probe==0) {
        if (tflag==1) { // bell and strip are on turn off
            tflag=0;
            IO_RelaysOff(); 
            stripoff();
        }
        if (touchco>=1) {touchco=touchco-1;} //slow turnoff against contact noise
        if (touchco==0) {dtouchco=15;}       //but reset the trap when settled!
    }
    
    if (lives==0) goto nolives;
  
    
    /*pg2022
    if (probe==1 && lives>0) {
        if (tflag==0) tflag=1;

        // threshold to take a life
        touchco=touchco+dtouchco;
        if (touchco>=15) 
        {  // LOST A LIFE
                lives=lives-1;
                meter(lives);
                touchco=0; //don't take another life immediately 
                dtouchco=1; //and be kind in next few seconds
                if (lives>0) {
                    stripbluemax(brightness); //blue flash lost life!
                    if (volume>0) {IO_RelaysOn(8);}
                }
    
        }
        
        //the following should be responsive to wire touch even if life not lost
        //set bell & red flash strip  (unless blue is on)
        else
        {
            if (volume>0) {IO_RelaysOn(8);}
            stripredmax(brightness);
        }
    }
    */

    //pg2022
    if (probe==1 && lives>=0) {
        if (tflag==0) tflag=1;

        // threshold to take a life
        touchco=touchco+dtouchco;

        //the following should be responsive to wire touch even if life not lost
        //set bell & red flash strip  (unless blue is on)
        if (touchco<15)
        {
            if (volume>0) {IO_RelaysOn(8);}
            stripredmax(brightness);
        }

        if (touchco>=15) 
        {  // LOST A LIFE
                if (lives>0) {lives=lives-1; }
                meter(lives);
                touchco=0; //don't take another life immediately 
                dtouchco=1; //and be kind in next few seconds
                if (lives>=0) {
                    if (volume>0) {IO_RelaysOn(8);}
                    stripbluemax(brightness); //blue flash lost life!
                    wait100ms();  //pg2022 I added these to show blue but careful this alters loop timing
                    //wait100ms();
                    //wait100ms();
                    //stripoff(); //blue flash lost life!
                }
        }
        
    }
    //end pg2022




    
    if (golr==1 && cusr==1) goto success;    
    if (gorl==1 && cusl==1) goto success;    
    
    // three ways to exit:
    // push start button, it will jump back to power-up
    if (START_PORT==0) goto restart;
    
    // Exit if time exceeds 100 seconds!
    if (timeup==0) goto gook_loop; 
    
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    // END OF GOOK LOOP /////////////////////////////////////// 
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////    
    ///////////////////////////////////////////////////////////    
    ///////////////////////////////////////////////////////////    
    
    
    
    
    
    // TIMEUP FAIL

    getbrivol();

    IO_RelaysOff(); // incase reached during strike
    // Keep the reason on the 8x8 in case of disputes!!!!!
    sprintf(spistr,""); //SH is 24 char long
    sprintf(spistr," Time's up! 100s maximum");
    SPI_String(spistr,0,0); //col,row
    
    failfx:
            
    for (i=0; i<3; i++) {
            stripredmax(brightness);
            wait100ms();
            stripoff();
            wait100ms();
    }
            
    stripredmax(brightness);

    
    getbrivol();
    
    if (timeup==0)   {   // only if out of lives
    // klaxon!
    int randur;
        
    dataSPI = 0x0C00;  // address4bits data8bits ! shutdown
    for (i=0;i<N8X8;i++) {darraySPI[i]=dataSPI;};
    SPI_Send(darraySPI);
    for (i=10000; i; i--);  

    
    
    if (volume>2) {
    // randomise duration of sound a little or it gets tedious
//pg2022        randur=TMR1 & 3;  //0 to 3
//pg2022        randur+=3; // 3 to 6 at full volume
        randur=TMR1 & 1;  //0 to 1  //pg2022
        randur+=5; // 5 to 6 at full volume  //pg2022
//pg2022        randur = (randur * volume) / 15;  //pg2022 it can be turned off and is off for vol<=2
        IO_RelaysOn(4); 
        for (i=0; i<randur; i++) {wait100ms();}
        IO_RelaysOff(); 
        }

    Setup7219(brightness);  //try to reset 7219s no does not work
    wait10ms();
    //klaxon zaps 8x8s
    // rewrite the reason on the 8x8 in case of disputes!!!!!
    sprintf(spistr,""); //SH is 24 char long
    sprintf(spistr,"Touched wiretoo often!  ");
    SPI_String(spistr,0,0); //col,row
    
    } // endif timeup==0 do this only if out of livss



    
    raspbeep(200,volume,4);

    
    retloop:


    Setup7219(brightness);  //try to reset 7219s no does not work
    wait10ms();
    
    
            
    // show blue return pattern until probe is returned to cusr or cusl
    goct=100;
    goct2=0;

    int sledgap, sledstartr, sledstartl;
    sledgap=5;
    sledstartl=0;
    sledstartr=5;
            
    retpatloop:
            
    // LED strip show scrolling dots    
    goct=goct+1;
    
    if (goct2==100)
    {sprintf(spistr,""); //SH is 24 char long
    sprintf(spistr,"Move to     either end  ");
    SPI_String(spistr,0,0); //col,row
    }

    
    if (goct>30)  // need to cal this against loop iteration time
    {
        goct=0;
        goct2=goct2+1;
        spvalb=16+8*brightness;    
        for (sled=0; sled<144; sled++) 
        {
            stripr[sled]=0;   
            stripg[sled]=0;   
            stripb[sled]=0;          
        }
        sledstartl=sledstartl+1;
        if (sledstartl>=sledgap) sledstartl=0;
        sledstartr=sledstartr-1;
        if (sledstartr<0) sledstartr=sledgap-1;
    
        for (sled=sledstartr; sled<64; sled+=sledgap) {
            stripb[sled]=spvalb;          
        }
        for (sled=sledstartl+80; sled<144; sled+=sledgap) {
            stripb[sled]=spvalb;          
        }
    stripfire(); 
    } // endif on goct strip update rate
    
    getdin();
    if (cusl==0 && cusr==0) goto retpatloop;
    
    stripoff();
    
    //restart
    goto restart;
    
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    // OUT OF LIVES FAIL     
    nolives:
    IO_RelaysOff();   // incase reached during strike!!
    // Keep the reason on the 8x8 in case of disputes!!!!!
    sprintf(spistr,""); //SH is 24 char long
    //blank 8x8 s because klaxon zaps it
    if (volume>0)  sprintf(spistr,"                        ");
    if (volume==0) sprintf(spistr,"Touched wiretoo often!  ");
    SPI_String(spistr,0,0); //col,row
    
    goto failfx; // run fail effects then restart

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    //   SUCCESS 
    //   REACHED OTHER COPPER
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    
    success:
    IO_RelaysOff();   // incase reached during strike!!
    stripoff();

    getbrivol();
    
    //keep this on display maybe flash it full brightness/invert
    sprintf(spistr,""); //DH is 24 char long
    sprintf(spistr,"Timed at %2dsRecord = %2ds",t,record[nlives-1]);
    SPI_String(spistr,0,0); //col,row

    // some initial effects
    // play arpeggio    randomise it a bit
    int randval1 = (TMR1 & 0x3);
    int randval2 = (TMR1 & 0x70)/ 0x10;
    int beepdur = 9+randval1;
    int arpkey = 29+randval2;

    int arptimes;
    for (arptimes=0; arptimes<1; arptimes++) {
        stripredmax(brightness);
        beep(beepdur,volume,arpkey);
        stripgreenmax(brightness);
        beep(beepdur,volume,arpkey+4);
        stripbluemax(brightness);
        beep(beepdur,volume,arpkey+7);
        stripredmax(brightness);
        beep(beepdur,volume,arpkey+12);
        stripgreenmax(brightness);
        beep(beepdur,volume,arpkey+16);
        stripbluemax(brightness);
        beep(beepdur,volume,arpkey+19);
        stripwhitemax(brightness);
        beep(4*beepdur,volume,arpkey+24);
        stripoff();
    }
    
    wait1snostart();
    wait1snostart();
    
    if (t > record[nlives-1]) goto retloop;

    record[nlives-1]=t;
    sprintf(spistr,""); //DH is 24 char long
    if (nlives==1) sprintf(tstr,"%2ds @ 1 life ",record[nlives-1]); //SH is 24 char long
    if (nlives>1)  sprintf(tstr,"%2ds @ %1dlives",record[nlives-1],nlives); //SH is 24 char long
    sprintf(spistr,"NEW RECORD! %s",tstr);
    SPI_String(spistr,0,0); //col,row

    
        for (arptimes=0; arptimes<2; arptimes++) {
            if (arptimes==0) arpkey+=2;
            if (arptimes==1) arpkey+=2;
            stripredmax(brightness);
        beep(beepdur,volume,arpkey);
        stripredmax(brightness);
        beep(beepdur,volume,arpkey+4);
        stripgreenmax(brightness);
        beep(beepdur,volume,arpkey+7);
        stripgreenmax(brightness);
        beep(beepdur,volume,arpkey+12);
        stripbluemax(brightness);
        beep(beepdur,volume,arpkey+16);
        stripbluemax(brightness);
        beep(beepdur,volume,arpkey+19);
        stripwhitemax(brightness);
        beep(4*beepdur,volume,arpkey+24);
        stripoff();
    }

    // bigger fuss if new record
    striprandom(1);
    chirp(2000,volume);
    striprandom(1);
    chirp(2000,volume);
    striprandom(1);
    chirp(2000,volume);
    striprandom(1);
    chirp(2000,volume);
    striprandom(1);
    chirp(2000,volume);
    stripoff();
    
    // new record write to NVRAM and make loud fuss about it
    stripoff();
    record[nlives-1]=t;
    wait10ms();
    writenvrecord(); //record array is global variable
    wait10ms();

    rec_loop:
    sprintf(spistr,""); //DH is 24 char long
    if (nlives==1) sprintf(tstr,"%2ds @ 1 life ",record[nlives-1]); //SH is 24 char long
    if (nlives>1)  sprintf(tstr,"%2ds @ %1dlives",record[nlives-1],nlives); //SH is 24 char long
    sprintf(spistr,"NEW RECORD! %s",tstr);
    SPI_String(spistr,0,0); //col,row
    for (i=0;i<17;i++) {
        wait100ms();
        if (START_PORT==0) goto rec_end;
        };
    sprintf(spistr,""); //DH is 24 char long
    sprintf(spistr,"Maybe enter your name?  ");
    SPI_String(spistr,0,0); //col,row
    for (i=0;i<17;i++) {
        wait100ms();
        if (START_PORT==0) goto rec_end;
        };
    sprintf(spistr,""); //DH is 24 char long
    sprintf(spistr,"Push START  when done   ");
    SPI_String(spistr,0,0); //col,row
    for (i=0;i<17;i++) {
        wait100ms();
        if (START_PORT==0) goto rec_end;
        };
    goto rec_loop;
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    rec_end:
    sprintf(spistr,"                        ");
    SPI_String(spistr,0,0); //col,row
    stripoff();
    IO_RelaysOff();
    wait1snostart(); 
    goto retloop;
    //took this out } 11jun
    
    }    


