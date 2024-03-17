 /*
	main
	
	MIDI processor
	Ben Hekster
	
	2024/01/05	PICDEM FS USB demonstration board
	2024/03/16	PIC18F2450 prototype
*/


// PIC18F2450 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 21        // Brown-out Reset Voltage bits (2.1V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = BB1K     // Boot Block Size Select bit (1KW Boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) or (001000-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) or (000000-000FFFh) is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) or (001000-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) or (000000-000FFFh) is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) or (001000-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) or (000000-000FFFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>



/*	gTick
	Timer tick counter
	
	I'm not actually using the timer anymore right now (used it previously
	for debugging) but expect to be needing it again, so leaving it in
*/
static volatile uint8_t gTick;


/*	Node
	MIDI bytes state machine state
	
	In a leaf node (i.e., fTransitions is NULL) the fOutBytes define the equivalent
	output that should be sent in place of the input that was consumed to reach
	that leaf state.
	
	In an intermediate node (fTransitions refer to further states), the fOutBytes
	refer to output that should be sent in case a transition *cannot* be made.
	In this case, fOutBytes would simply be a copy of all the input that had been
	consumed to reach that intermediate state.
 
	This approach has the advantage of using fOutBytes consistently, and is static;
	another option would be to just buffer input as it is consumed, at the cost
	of some buffer management.
*/
struct Node {
	// transitions (of count fTransitionsN) out from this state, or NULL
	const struct Node *fTransitions;
	
	// bytes (of length fOutBytesN) to put out if exiting this state
	const uint8_t	*fOutBytes;
	
	// byte that will lead to this transition
	uint8_t		fInByte;
	
	uint8_t		fOutBytesN;
	uint8_t		fTransitionsN;
	};


/*

	input strings

*/

static const uint8_t
	gInStart[] = { 0xF0, 0x7F, 0x7F, 0x06, 0x02, 0xF7 },
	gInStop[] = { 0xF0, 0x7F, 0x7F, 0x06, 0x09, 0xF7 };

/*

	output strings

*/
static const uint8_t
	gOutStart[] = { 0xFA },
	gOutStop[] = { 0xFC };


/*
 
	state machine
	
	The two MIDI strings that we replace are MIDI Machine Control (MMC) SysEx sequences,
	and map them to System Real-Time messages:
	
		F0 7F 7F 06 02 F7	=>	FA
		F0 7F 7F 06 09 F7	=>	FC

*/

static const struct Node
	gTransitionsG[] = {
		{ NULL, gOutStart, 0xF7, sizeof gOutStart, 0 }
		},

	gTransitionsH[] = {
		{ NULL, gOutStop, 0xF7, sizeof gOutStop, 0 }
		},
	
	gTransitionsF[] = {
		{ gTransitionsG, gInStart, 0x02, 5, sizeof gTransitionsG / sizeof *gTransitionsG },
		{ gTransitionsH, gInStop, 0x09, 5, sizeof gTransitionsH / sizeof *gTransitionsH }
		},
	
	gTransitionsE[] = {
		{ gTransitionsF, gInStart, 0x06, 4, sizeof gTransitionsF / sizeof *gTransitionsF }
		},
	
	gTransitionsD[] = {
		{ gTransitionsE, gInStart, 0x7F, 3, sizeof gTransitionsE / sizeof *gTransitionsE }
		},
	
	gTransitionsC[] = {
		{ gTransitionsD, gInStart, 0x7F, 2, sizeof gTransitionsD / sizeof *gTransitionsD }
		},
	
	gTransitionsB[] = {
		{ gTransitionsC, gInStart, 0xF0, 1, sizeof gTransitionsC / sizeof *gTransitionsC }
		},
	
	gTransitionsA = {
		gTransitionsB, NULL, 0x00, 0, sizeof gTransitionsB / sizeof *gTransitionsB
		};


/*	gState
	Current state of machine
*/
static const struct Node *gState = &gTransitionsA;


/*	SendBuffer
	Send a string of bytes to output
	*** implement
*/
#if 0
static void SendBuffer(
	const uint8_t	*out,
	uint8_t		n
	)
{
}
#endif


/*	SendOne
	Send a single byte to output
	This helps to optimize for the case where we are sending one byte
	and there is no other pending buffered output
*/
static inline void SendOne(
	const uint8_t	out
	)
{
// TXREG is empty and nothing pending?
/* We'll only enable transmit interrupts if we have data pending for output */
if (PIR1bits.TXIF && !PIE1bits.TXIE)
	TXREG = out;

else
	// buffer
	; // SendBuffer(&out, 1);
}


/*	SendState
	Send the output bytes of the current state
	This is optimized to send the first byte as quickly as possible
*/
static void SendState()
{
// number of bytes to send from current state
uint8_t outBytesN = gState->fOutBytesN;

// any?
if (outBytesN > 0) {
	// bytes to send from current state
	const uint8_t *outBytes = gState->fOutBytes;
	
	// TXREG is empty and nothing pending?
	/* We'll only enable transmit interrupts if we have data pending for output */
	if (PIR1bits.TXIF && !PIE1bits.TXIE) {
		// send the first byte immediately
		TXREG = *outBytes++;
		outBytesN--;
		}
	
	// any bytes left?
	if (outBytesN > 0)
		; // SendBuffer(outBytes, outBytesN);
	}
}


/*	ReceiveOne
	Process one MIDI input byte
	
	Some version of this could not complete with a 1 MHz system clock
	(corresponding to a 250 kHz instruction clock).  MIDI bytes arrive
	at 3125 Hz, corresponding to only 80 instructions!
*/
static void ReceiveOne(
	const uint8_t	in
	)
{
// process the input
for (;;) {
	// look for transitions for that input in current state
	const struct Node
		*transition = gState->fTransitions,
		*const transitionE = transition + gState->fTransitionsN;
	for (; transition < transitionE; transition++)
		if (in == transition->fInByte)
			break;
	
	// found a transition for that input?
	if (transition < transitionE) {
		// make transition
		gState = transition;
		
		// play/stop mode
		if (gState == &gTransitionsG[0]) LATBbits.LATB1 = 1;
		if (gState == &gTransitionsH[0]) LATBbits.LATB1 = 0;
		
		// matched through to a leaf state?
		if (!gState->fTransitions) {
			// send output of leaf state
			SendState();
			
			// continue in root state
			gState = &gTransitionsA;
			}
		}
	
	// no transition found?
	else
		// in root state?
		if (gState == &gTransitionsA)
			// just relay single input byte
			SendOne(in);
		
		else {
			// send output of intermediate state
			SendState();
			
			// back to root state
			gState = &gTransitionsA;
			
			// reprocess input byte at root state
			continue;
			}
	
	// input was processed
	break;
	}
}


/*	ISR
	Interrupt Service Routine
*/
void __interrupt(high_priority) ISR(void)
{
// timer?
if (INTCONbits.TMR0IF) {
	// tick
	gTick++;
	
	LATBbits.LATB0 = gTick % 2;
	
	// clear receive indicator LED
	//LATBbits.LATB1 = 0;
	
	// 3906 Hz timer clock = 15 * 256 + 66
	// 7812 Hz timer clock = 30 * 256 + 132
	TMR0H = 256 - 30;
	TMR0L = 256 - 132;
	
	// clear timer interrupt flag
	INTCONbits.TMR0IF = 0;
	}

// waiting to send more bytes, and TXREG empty?
/* We'll only enable transmit interrupts if we have data pending for output */
if (PIE1bits.TXIE && PIR1bits.TXIF)
	;

// UART receive?
if (PIR1bits.RCIF) {
	ReceiveOne(RCREG);
	
	// receive
	//LATBbits.LATB1 = 1;
	
	// PIR1bits.RCIF cleared when RCREG is read
	}
}


int main(void)
{
// disable priority levels
RCONbits.IPEN = 0;

// disable global interrupts during setup
INTCONbits.GIE = 0;

// primary oscillator is already default

// enable Idle (as opposed to Sleep) modes
OSCCONbits.IDLEN = 1;

// configure all pins as digital
ADCON1bits.PCFG = 0xF;

// enable LED B0
LATBbits.LATB0 = 0; // off
TRISBbits.TRISB0 = 0; // output

// enable LED B1
LATBbits.LATB1 = 0; // off
TRISBbits.TRISB1 = 0; // output

// enable UART
TXSTAbits.BRGH = 1; // high speed
BAUDCONbits.BRG16 = 0; // 8-bit counter

/* f_osc/(16(n + 1)) */
SPBRG = 15; // 8 MHz / (16*16) = 31.25 kHz
TXSTAbits.SYNC = 0; // asynchronous
RCSTAbits.SPEN = 1;
TRISCbits.TRISC7 = 1; // input
TRISCbits.TRISC6 = 0; // output
TXSTAbits.TXEN = 1; // enable transmitter
PIE1bits.RCIE = 1; // enable receive interrupt
RCSTAbits.CREN = 1; // enable continuous receiver
PIE1bits.TXIE = 0; // disable transmit interrupt

// enable timer
/* 8 MHz system clock; 2000 kHz instruction clock; 2000 / 256 kHz timer clock */
T0CONbits.T08BIT = 0; // 16-bit timer
T0CONbits.T0CS = 0; // timer mode
T0CONbits.T0PS = 7; // 1:256
T0CONbits.PSA = 0; // prescaler enabled
TMR0H = (uint8_t) ~0; // trigger timer immediately
TMR0L = (uint8_t) ~0;
T0CONbits.TMR0ON = 1;
INTCONbits.TMR0IF = 0;
INTCONbits.TMR0IE = 1;

// enable interrupts
INTCONbits.PEIE = 1; // peripheral
INTCONbits.GIE = 1; // global

#if 0
// table read accesses flash
// CFGS doesn't seem to make a difference, either
EECON1bits.CFGS = 0;

// EEPGD access program
EECON1bits.EEPGD = 1;

TBLPTRU = 0x30;
TBLPTRH = 0x00;
TBLPTRL = 0x00;

while(1) {
    asm("TBLRD*+");
    uint8_t x = TABLAT;

    for (unsigned i = 8; i > 0; i--) {
        LATDbits.LATD3 = x & 1;
        LATDbits.LATD2 = 1 /* on */;
        sleepFor(1);
        
        LATDbits.LATD3 = 0;
        LATDbits.LATD2 = 0; /* off */
        sleepFor(1);
        
        x >>= 1;
        }

    sleepFor(2);
    }
#endif

// all processing done in interrupts
for (;;) SLEEP();
}
