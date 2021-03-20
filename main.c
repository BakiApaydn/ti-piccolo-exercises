#include "DSP28x_Project.h"
#include "F2806x_Examples.h"
#include "main.h"
//#include "gpio_init.h"


#define DATA GPIO16
#define CLOCK GPIO18
#define LATCH GPIO20
#define ENABLE GPIO22
#define LOAD GPIO24

void delay (unsigned int ms);
void pulseClock(void);
void shiftOut(int);
void enable(void);
void disable(void);
void init(void);
void pinWrite(unsigned int bit,unsigned char val );
void delay_loop1();

unsigned char val;
unsigned int ms;

void delay_loop1(void); // Telling the compiler that this function is defined later
void gpio_init(void);
void main(void)
{
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

#ifdef FLASH_LINK
    // Copy time critical code and Flash setup code to RAM
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    InitFlash();
#endif

    // Set the High-Speed Peripheral Clock Prescaler (HISPCP) such that HSPCLK = 25 MHz
    // HSPCLK = SYSCLKOUT/(HISPCP x 2)
 /*
    EALLOW;
#if CPU_FRQ_150MHZ
    SysCtrlRegs.HISPCP.all = 3;
#else
    SysCtrlRegs.HISPCP.all = 2;
#endif
    EDIS;
*/
    // Step 2. Initalize GPIO:
    gpio_init();

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    InitPieVectTable();

    // Step 4. Initialize all the Device Peripherals:

    // Step 5. User specific code, enable interrupts:

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    enable();
    int i;
    while (1)
    {
        for (i = 0; i < 8; i++)
        {
            shiftOut(1<<i);
            delay_loop1();
            //delay(50);
        }
        for (i = 7; i > 0; i--)
        {
        shiftOut(1<<i);
        delay_loop1();
        //delay(50);
        }



        GpioDataRegs.GPATOGGLE.bit.GPIO31=1; // GPIOx bit will be toggled
        GpioDataRegs.GPBTOGGLE.bit.GPIO34=1; // GPIOx bit will be toggled
        delay_loop1(); // Some delay to increase the time between toggles
        delay_loop1(); // Some delay to increase the time between toggles

        //GpioDataRegs.GPATOGGLE.bit.GPIO20=1; // GPIOx bit will be toggled
        //delay_loop1(); // Some delay to increase the time between toggles

        //GpioDataRegs.GPATOGGLE.bit.GPIO18=1; // GPIOx bit will be toggled
       // delay_loop1(); // Some delay to increase the time between toggles

        //GpioDataRegs.GPATOGGLE.bit.GPIO16=1; // GPIOx bit will be toggled
        //delay_loop1(); // Some delay to increase the time between toggles


        asm(" NOP");
    }

void delay (unsigned int ms)
  {
    while(ms--)
      {
          __delay_cycles(1000);
      }
  }

// Delay function to provide some delay. Timer delay is recommended instead of this kind of delay loop.
void delay_loop1(void)
  {
      long i; // defining a variable type long

      for (i=0;i<1000000;i++)
      {
          // Do nothing
      }
  }
void gpio_init(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO31=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPASET.bit.GPIO31=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPAMUX2.bit.GPIO31=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPADIR.bit.GPIO31=1;   // Set GPIOx pin as output. 1 for output, 0 for input.

    GpioCtrlRegs.GPBPUD.bit.GPIO34=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPBSET.bit.GPIO34=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPBMUX1.bit.GPIO34=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPBDIR.bit.GPIO34=1;   // Set GPIOx pin as output. 1 for output, 0 for input.


    GpioCtrlRegs.GPAPUD.bit.GPIO18=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPASET.bit.GPIO18=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPAMUX2.bit.GPIO18=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPADIR.bit.GPIO18=1;   // Set GPIOx pin as output. 1 for output, 0 for input.


    GpioCtrlRegs.GPAPUD.bit.GPIO20=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPASET.bit.GPIO20=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPAMUX2.bit.GPIO20=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPADIR.bit.GPIO20=1;   // Set GPIOx pin as output. 1 for output, 0 for input.



    GpioCtrlRegs.GPAPUD.bit.GPIO16=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPASET.bit.GPIO16=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPAMUX2.bit.GPIO16=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPADIR.bit.GPIO16=1;   // Set GPIOx pin as output. 1 for output, 0 for input.

    GpioCtrlRegs.GPAPUD.bit.GPIO22=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPASET.bit.GPIO22=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPAMUX2.bit.GPIO22=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPADIR.bit.GPIO22=1;   // Set GPIOx pin as output. 1 for output, 0 for input.

    GpioCtrlRegs.GPAPUD.bit.GPIO24=0;   // Enable pullup on GPIO pin
    GpioDataRegs.GPASET.bit.GPIO24=0;   // Load 0 to output latch, you can load either 0 or 1.
    GpioCtrlRegs.GPAMUX2.bit.GPIO24=0;  // 0 to pass the latch value to the output, 1 to configure as EPWM
    GpioCtrlRegs.GPADIR.bit.GPIO24=1;   // Set GPIOx pin as output. 1 for output, 0 for input.
    EDIS;
}

void pinWrite(unsigned int bit,unsigned char val )
{
    if(val)
    {
        GpioDataRegs.GPASET.bit.GPIO16=1;   //P1OUT |=bit;
        GpioDataRegs.GPASET.bit.GPIO18=1;
        GpioDataRegs.GPASET.bit.GPIO20=1;
        GpioDataRegs.GPASET.bit.GPIO22=1;
        GpioDataRegs.GPASET.bit.GPIO24=1;
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO16=1;   // P1OUT &=  ~bit;
        GpioDataRegs.GPACLEAR.bit.GPIO18=1;
        GpioDataRegs.GPACLEAR.bit.GPIO20=1;
        GpioDataRegs.GPACLEAR.bit.GPIO22=1;
        GpioDataRegs.GPACLEAR.bit.GPIO24=1;
    }
}

void pulseClock(void)
{
    GpioDataRegs.GPASET.bit.GPIO18=1;        //P1OUT |= CLOCK;
    GpioDataRegs.GPATOGGLE.bit.GPIO18=1;     //P1OUT ^= CLOCK;
}

void shiftOut(int)
{
    GpioDataRegs.GPACLEAR.bit.GPIO18=1;  //P1OUT &=  ~LATCH;

    char i;
    for(i=0;i<8;i++)
    {
        pinWrite(DATA, (val &(1<<i)));
        pulseClock();
    }
    GpioDataRegs.GPASET.bit.GPIO20=1;     //P1OUT |= LATCH;
    GpioDataRegs.GPACLEAR.bit.GPIO20=1;   //P1OUT &= ~LATCH;
    GpioDataRegs.GPASET.bit.GPIO24=1;     //P1OUT |= LOAD;
    GpioDataRegs.GPACLEAR.bit.GPIO24=1;   //P1OUT &= ~LOAD;
}
void enable (void)
{
    GpioDataRegs.GPASET.bit.GPIO22=1; //P1OUT &=  ~ENABLE;
}
void disable (void)
{
    GpioDataRegs.GPASET.bit.GPIO22=1; //P11OUT |= ENABLE;
}


