 /*
	main
	
	MIDI processor
	
	2024/01/05	Ben Hekster
*/

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
if (PIR1bits.TXIF == 1 && PIE1bits.TXIE == 0)
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
	if (PIR1bits.TXIF == 1 && PIE1bits.TXIE == 0) {
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
		if (gState == &gTransitionsG[0]) LATDbits.LATD3 = 1;
		if (gState == &gTransitionsH[0]) LATDbits.LATD3 = 0;
		
		// matched through to a leaf state?
		if (!gState->fTransitions) {
			// send output of leaf state
			SendState();
			
			// restart in root state
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
	
	// clear receive indicator LED
	LATDbits.LATD2 = 0;
	
	// 3906 Hz timer clock = 15 * 256 + 66
	TMR0H = 256 - 15;
	TMR0L = 256 - 66;
	
	// clear timer interrupt flag
	INTCONbits.TMR0IF = 0;
	}

// waiting to send more bytes, and TXREG empty?
/* We'll only enable transmit interrupts if we have data pending for output */
if (PIE1bits.TXIE == 1 && PIR1bits.TXIF == 1)
	;

// UART receive?
if (PIR1bits.RCIF) {
	ReceiveOne(RCREG);
	
	// receive
	LATDbits.LATD2 = 1;
	
	// PIR1bits.RCIF cleared when RCREG is read
	}
}


int main(void)
{
// disable priority levels
RCONbits.IPEN = 0;

// disable global interrupts during setup
INTCONbits.GIE = 0;

// enable Idle (as opposed to Sleep) modes
OSCCONbits.IDLEN = 1;

// change Internal Oscillator Block frequency from 1 MHz to 4 MHz
OSCCONbits.IRCF = 5;

// enable LED D3
LATDbits.LATD2 = 0; // off
TRISDbits.TRISD2 = 0; // output
ANSELDbits.ANSD2 = 0; // digital

// enable LED D4
LATDbits.LATD3 = 0; // off
TRISDbits.TRISD3 = 0; // output
ANSELDbits.ANSD3 = 0; // digital

// enable UART
TXSTAbits.BRGH = 1; // high speed
BAUDCONbits.BRG16 = 0; // 8-bit counter

/* f_osc/(16(n + 1)) */ {
	//SPBRG = 1; // = 1 MHz / (16*2) = 31.25 kHz
	//SPBRG = 3; // 2 MHz / (16*4) = 31.25 kHz
	SPBRG = 7; // 4 MHz / (16*8) = 31.25 kHz
	}
TXSTAbits.SYNC = 0; // asynchronous
RCSTAbits.SPEN = 1;
TRISCbits.TRISC7 = 1; // input
TRISCbits.TRISC6 = 0; // output
TXSTAbits.TX9 = 0; // 8-bit data words
TXSTAbits.TXEN = 1; // enable transmitter
PIE1bits.RCIE = 1; // enable receive interrupt
RCSTAbits.RX9 = 0; // 8-bit data words
RCSTAbits.CREN = 1; // enable receiver
ANSELCbits.ANSC6 = 0;
ANSELCbits.ANSC7 = 0;
PIE1bits.TXIE = 0; // disable receive interrupt

// enable timer
/* 1 MHz system clock; 250 kHz instruction clock; 250/256 kHz timer clock */
/* 4 MHz system clock; 1000 kHz instruction clock; 1000 / 256 kHz timer clock */
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
