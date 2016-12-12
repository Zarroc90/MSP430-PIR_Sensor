//*****************************************************************************
//  Code for application report SLAA283 - "Ultra-low Power Motion Detection
//    using the MSP430F2013"
//
//  Version 0-01
//
//  Version Summary:
//  0-00 released 12-22-2005- Initial release
//  0-01 release pending
//       - Added code to allow LED to be turned on/off via pushbutton
//         (enables valid Icc measurement during operation)
//
//  Z. Albus
//  Texas Instruments Inc.
//  May 2006
//  Built with IAR Embedded Workbench Version: 3.41A
//*****************************************************************************
// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
//
//******************************************************************************
#include  <msp430x20x3.h>

#define   	LED_OUT         BIT0              	// Bit location for LED
#define		SCLK			BIT5				// Bit Location for SCLK
#define		PIR				BIT6				// Bit Location for PIR
#define   	SENSOR_PWR      BIT7              	// Bit location for power to sensor
#define		TIME_5ms		0x03
#define		TIME_10ms		0x13
#define		TIME_21ms		0x23
#define		TIME_42ms		0x33
#define		TIME_84ms		0x12
#define		TIME_170ms		0x22
#define		TIME_341ms		0x32
#define		TIME_683ms		0x01
#define		TIME_1366ms		0x11
#define		TIME_2732ms		0x21
#define		TIME_5464ms		0x31
#define		TIME_10928ms	0x20
#define		TIME_21856ms	0x30

static unsigned int result_old = 0;         // Storage for last conversion
static unsigned int reed = 0;					// Variable for Reed Status
static unsigned int prevreed = 0;				// Previous reed status
static unsigned int i2c_object = 0;			//
static unsigned int i2c_value = 0;			//
static unsigned int THRESHOLD = 50;                	// Threshold for motion
static unsigned int Interrupt_Package = 0;	//0x01= Motion 0x02=Reed
static unsigned int Motion_Timeout = TIME_10928ms;	//
int I2C_State, Bytecount, transmit = 0;     // State variables
char SLV_Addr = 0x90;                  // Address is 0x48<<1 for R/W
char LED_ENABLE = 1;                        // LED control

void main(void) {
	WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL; // ACLK/32768, int timer: ~10s
	BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;
	BCSCTL3 |= LFXT1S_2;						// ACLK source VLO Clk
	BCSCTL1 |= DIVA_2;                        // ACLK = VLO/4

	P1OUT = 0xE0;        		        	    // P1OUTs P1.5,1.6,1.7
	P1SEL = 0x08;                             // Select VREF function
	P1DIR = 0xEF;                             // Unused pins as outputs
	P1REN |= 0xD0;		                    // P1.4 1.6 1.7 pullup
	//P1IE |= 0x10;                             // P1.4 interrupt enabled
	//P1IES |= 0x10;                            // P1.4 Hi/lo edge
	//P1IFG &= ~0x10;                           // P1.4 IFG cleared

	P2OUT = 0x40 + SENSOR_PWR;                // P2OUTs 2.6/2.7
	P2SEL &= ~(0x40 + SENSOR_PWR);              // P2.6/P2.7 = GPIO
	P2DIR = 0xff;                             // Unused pins as outputs

	//Setup I2C Config
	USICTL0 = USIPE6 + USIPE7 + USISWRST;    // Port & USI mode setup
	USICTL1 = USII2C + USIIE + USISTTIE;     // Enable I2C mode & USI interrupts
	USICKCTL = USICKPL;                  // Setup clock polarity
	USICNT |= USIIFGCC;                  // Disable automatic clear control
	USICTL0 &= ~USISWRST;                // Enable USI
	USICTL1 &= ~USIIFG;                  // Clear pending flag

	//Setup PIR Sensor
	SD16CTL = SD16VMIDON + SD16REFON + SD16SSEL_1;            // 1.2V ref, SMCLK
	SD16INCTL0 = SD16GAIN_4 + SD16INCH_4;     // PGA = 4x, Diff inputs A4- & A4+
	SD16CCTL0 = SD16SNGL + SD16IE;      // Single conversion, 256OSR, Int enable
	SD16CTL &= ~SD16VMIDON;                  // VMID off: used to settle ref cap
	SD16AE = SD16AE1 + SD16AE2;              // P1.1 & P1.2: A4+/- SD16_A inputs

	// Wait for PIR sensor to settle: 1st WDT+ interval
	P1SEL |= LED_OUT;                     // Turn LED on with ACLK (for low Icc)
	while (!(IFG1 & WDTIFG))
		;                  // ~5.4s delay: PIR sensor settling
	P1SEL &= ~LED_OUT;                   // Turn LED off with ACLK (for low Icc)

	// Reconfig WDT+ for normal operation: interval of ~341msec
	WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL + 0x01; // ACLK/8192, int timer: 341msec*2=682ms
	BCSCTL1 &= ~DIVA_2;
	BCSCTL1 |= DIVA_0;                        // ACLK = VLO(12khz)/1
	IE1 |= WDTIE;                             // Enable WDT interrupt

	//Initial Outputs
	P1OUT |= SCLK;							//Pull SCLK High
	P2OUT |= PIR;								//Pull PIR High

	_BIS_SR(LPM3_bits + GIE);                 // Enter LPM3 with interrupts
}

/******************************************************
 // SD16_A interrupt service routine
 ******************************************************/
#pragma vector = SD16_VECTOR
__interrupt void SD16ISR(void) {
	unsigned int result_new;

	SD16CTL &= ~SD16REFON;                    // Turn off SD16_A ref
	result_new = SD16MEM0;                    // Save result (clears IFG)

	if (result_new > result_old)              // Get difference between samples
		result_old = result_new - result_old;
	else
		result_old = result_old - result_new;

	if (result_old > THRESHOLD)               // If motion detected...
			{

			P1OUT |= LED_OUT;                   // Turn LED on
			P1OUT &= ~SCLK;						//Pull SCLK LOW
			_delay_cycles(5000);				//wait 5ms
			P1OUT |= SCLK;						//Pull SCLK High
			Interrupt_Package|= 1;
			//Set timer and go for Timeout in Sleepmode
			WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL + (Motion_Timeout & 0x03); // WDTISx
			BCSCTL1 &= ~0x30;						//Reset BCSCTL1 ACLK Divider
			BCSCTL1 |= (Motion_Timeout & 0x30);           // SET ACLK Divider

			}
	else{
		Interrupt_Package &= ~1;
	}
	result_old = SD16MEM0;                    // Save last conversion

	__bis_SR_register_on_exit(SCG1 + SCG0);     // Return to LPM3 after reti
}

/******************************************************
 // Watchdog Timer interrupt service routine
 ******************************************************/

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void) {
	/*test=P1IN;
	 if ((P1IN&0x10)==0x10)
	 {
	 reed=1;
	 }
	 else
	 {
	 reed=0;
	 }

	 if (prevreed!=reed)
	 {
	 P1OUT &= ~SCLK;						//Pull SCLK LOW
	 __delay_cycles(10000);				//wait 5ms
	 P1OUT |= SCLK;						//Pull SCLK High
	 }*/

	if (!(P1OUT & LED_OUT))                 // Has motion already been detected?
	{
		SD16CTL |= SD16REFON;                   // If no, turn on SD16_A ref
		SD16CCTL0 |= SD16SC;                  // Set bit to start new conversion
		prevreed = reed;
		__bic_SR_register_on_exit(SCG1 + SCG0); // Keep DCO & SMCLK on after reti
	} else
		P1OUT &= ~LED_OUT;         // If yes, turn off LED, measure on next loop

	prevreed = reed;

}

// Port 1 interrupt service routine
/*#pragma vector=PORT1_VECTOR
 __interrupt void Port_1(void)
 {

 if ((P1IFG&0x10)==0x10) {
 P1OUT &= ~SCLK;						//Pull SCLK LOW
 __delay_cycles(10000);					//wait 5ms
 P1OUT |= SCLK;						//Pull SCLK High
 P1IFG &= ~0x10;							// P1.4 IFG cleared
 }

 }*/

//******************************************************************************
// USI interrupt service routine
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USI_VECTOR
__interrupt void USI_TXRX(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USI_VECTOR))) USI_TXRX (void)
#else
#error Compiler not supported!
#endif
{
	if (USICTL1 & USISTTIFG)             // Start entry?
	{
		P1OUT |= 0x01;                     // LED on: sequence start
		I2C_State = 2;                     // Enter 1st state on start
	}

	switch (I2C_State) {
	case 0: // Idle, should not get here
		break;

	case 2: // RX Address
		USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX address
		USICTL1 &= ~USISTTIFG;   // Clear start flag
		I2C_State = 4;           // Go to next state: check address
		break;

	case 4: // Process Address and send (N)Ack

		if (USISRL & 0x01) {            // If master read...
			SLV_Addr = 0x91;             // Save R/W bit
			transmit = 1;
		} else {
			transmit = 0;
			SLV_Addr = 0x90;
		}
		USICTL0 |= USIOE;             // SDA = output
		if (USISRL == SLV_Addr)       // Address match?
				{
			USISRL = 0x00;              // Send Ack
			if (transmit == 0) {
				I2C_State = 6;
			}           // Go to next state: RX data
			if (transmit == 1) {
				I2C_State = 18;
			}          // Else go to next state: TX data
		} else {
			USISRL = 0xFF;              // Send NAck
			I2C_State = 6;              // next state: prep for next Start
		}
		USICNT |= 0x01;               // Bit counter = 1, send (N)Ack bit
		break;

	case 6: // Prep for Start condition
		USICTL0 &= ~USIOE;       // SDA = input
		SLV_Addr = 0x90;         // Reset slave address
		I2C_State = 0;           // Reset state machine
		break;

	case 8: // Receive data byte I
		USICTL0 &= ~USIOE;       // SDA = input
		USICNT |= 0x08;         // Bit counter = 8, RX data
		I2C_State = 10;          // Go to next state: Test data and (N)Ack
		break;

	case 10:          // Data & TX (N)Ack
		USICTL0 |= USIOE;        // SDA = output
		i2c_object = USISRL;		//Receive Object
		USISRL = 0x00;         	// Send Ack
		USICNT |= 0x01;          // Bit counter = 1, send (N)Ack bit
		I2C_State = 12;           // Go to next state: Receive data byte II
		break;

	case 12: // Receive data byte II
		USICTL0 &= ~USIOE;       // SDA = input
		USICNT |= 0x08;         // Bit counter = 8, RX data
		I2C_State = 14;          // Go to next state: Test data and (N)Ack
		break;

	case 14:          // Data & TX (N)Ack
		USICTL0 |= USIOE;        	// SDA = output
		i2c_value = USISRL;			//Receive Value
		USISRL = 0x00;         		// Send Ack
		USICNT |= 0x01;          	// Bit counter = 1, send (N)Ack bit
		I2C_State = 16;           	// Go to next state: Apply I2C Commands
		break;

	case 16:           	// Apply I2C Commands
		switch (i2c_object) {
		case 1:           	//Timeout

			break;
		case 2:           	//Abtastrate
							// Reconfig WDT+ for normal operation: interval of ~341msec
			WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL + (i2c_value & 0x03); // WDTISx
			BCSCTL1 &= ~0x30;						//Reset BCSCTL1 ACLK Divider
			BCSCTL1 |= (i2c_value & 0x30);           // SET ACLK Divider
			IE1 |= WDTIE;                            // Enable WDT interrupt
			break;
		case 3:                            //Threshold
			THRESHOLD = i2c_value;
			break;
		}
		I2C_State = 6;           	// Go to next state: prep for next Start
		break;

	case 18: // Send Data byte
		USICTL0 |= USIOE;             // SDA = output
		USISRL = Interrupt_Package;
		USICNT |= 0x08;              // Bit counter = 8, TX data
		I2C_State = 20;               // Go to next state: receive (N)Ack
		break;

	case 20:               // Receive Data (N)Ack
		USICTL0 &= ~USIOE;            // SDA = input
		USICNT |= 0x01;               // Bit counter = 1, receive (N)Ack
		I2C_State = 22;               // Go to next state: check (N)Ack
		break;

	case 22:               // Process Data Ack/NAck
		if (USISRL & 0x01)               // If Nack received...
				{
			USICTL0 &= ~USIOE;            // SDA = input
			SLV_Addr = 0x90;              // Reset slave address
			I2C_State = 0;                // Reset state machine
			Bytecount = 0;
			// LPM0_EXIT;                  // Exit active for next transfer
		}
		I2C_State = 6;						//Prepare for start condition
		break;
	}

	USICTL1 &= ~USIIFG;                  // Clear pending flags
}
