// PIC32MX460L Example Code for a security system
// Author: Byron Lambrou
// Input: Keypad PMod Microphone Pmod
// Output: 2xSS pmods
// Comments: Programmed in MPLABx for the chipkit MX4 PIC32MX460F512L
// Used with the XC32 compiler, free version.
#include <p32xxxx.h>
#include <plib.h>
#include <ctype.h>
#include <stdlib.h>
#include "dsplib_dsp.h"
#include "fftc.h"

/* SYSCLK = 8MHz Crystal/ FPLLIDIV * FPLLMUL/ FPLLODIV = 80MHz
PBCLK = SYSCLK /FPBDIV = 80MHz*/
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

/* Input array with 16-bit complex fixed-point twiddle factors.
 this is for 16-point FFT. For other configurations, for example 32 point FFT,
 Change it to fft16c32 */

#define fftc fft16c1024

/* defines the sample frequency*/
#define SAMPLE_FREQ 2048

/* number of FFT points (must be power of 2) */
#define N 1024

// 7 Segment Display pmod using the TOP JA & JB jumpers
// Segments
#define SegA_R LATEbits.LATE0
#define SegB_R LATEbits.LATE1
#define SegC_R LATEbits.LATE2
#define SegD_R LATEbits.LATE3
#define SegE_R LATGbits.LATG9
#define SegF_R LATGbits.LATG8
#define SegG_R LATGbits.LATG7
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel_R LATGbits.LATG6

// 7 Segment Display pmod using the TOP JC and JD jumpers
#define SegA_L LATGbits.LATG12
#define SegB_L LATGbits.LATG13
#define SegC_L LATGbits.LATG14
#define SegD_L LATGbits.LATG15
#define SegE_L LATDbits.LATD7
#define SegF_L LATDbits.LATD1
#define SegG_L LATDbits.LATD9
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel_L LATCbits.LATC1

//Keypad pmod using the JJ input
//Column pins, should be inputs?
#define Col4 PORTBbits.RB0
#define Col3 PORTBbits.RB1
#define Col2 PORTBbits.RB2
#define Col1 PORTBbits.RB3
//Row pins, should be outputs?
#define Row4 LATBbits.LATB4
#define Row3 LATBbits.LATB5
#define Row2 LATBbits.LATB8
#define Row1 LATBbits.LATB9

//On Board LED pins
#define BLed1   LATBbits.LATB10
#define BLed2   LATBbits.LATB11
#define BLed3   LATBbits.LATB12
#define BLed4   LATBbits.LATB13

//Port mapping: Mic- port JK input

/* [Var]Variable Declarations */
int for_Loop = 0;
int SSD_blink = 0;
int trigger = 0;

int left_ssd_val = 1;
int test = 1;

unsigned int dummy;
int key_to_react = 1;
int key_pressed = 0;
int key_released = 0;
int pressed_key;
int last_key = 0;
int key_detected;
int button_lock = 0;

int passcode = 0;
int guess = 0;
int length_held = 0;

int changed_to_mode_3 = 0;
int first_press_in_mode_3 = 0;
int first_press_time_mode_3 = 0;

int samp = 0;
int mode = 1;
int time_counter_ssd = 0;
int time_counter_seconds = 0;

int sample_counter = 0;

// log base 2 of N(1024) = 102
int log2N = 10;

/* Input array with 16-bit complex fixed-point elements. */
/* int 16c is data struct defined as following:
  typedef struct
{
        int16 re;
        int16 im;
} int16c; */
int16c sample_buffer[N];

/* intermediate array */
int16c scratch[N];

/* intermediate array holds computed FFT until transmission*/
int16c dout[N];

/* intermediate array holds computed single side FFT */
int single_sided_fft[N];

/* array that stores the corresponding frequency of each point in frequency domain*/
short freq_vector[N];

/* indicates the dominant frequency */
int freq = 0;

// Function definitions
int compute_FFT(int16c sample_buffer[]);

unsigned char SSD_number[] = {
    0b0111111, //0
    0b0000110, //1
    0b1011011, //2
    0b1001111, //3
    0b1100110, //4
    0b1101101, //5
    0b1111101, //6
    0b0000111, //7
    0b1111111, //8
    0b1101111, //9
    0b1110111, //A
    0b1111100, //B
    0b0111001, //C
    0b1011110, //D
    0b1111001, //E
    0b1110001, //F
    0b1110110, //H
    0b0111000, //L
    0b0011100, //u
    0b0000000 //clear
};
char keypad_number[] = {
    1, //0
    2, //1
    3, //2
    'A', //3
    4, //4
    5, //5
    6, //6
    'B', //7
    7, //8
    8, //9
    9, //10
    'C', //11
    0, //12
    'F', //13
    'E', //14
    'D', //15
    'Q' //error16
};

//Timer 5 interrupt, we can use this for the first interrupt

void __ISR(_TIMER_5_VECTOR, ipl5) _T5Interrupt(void) {
    //This timer should occur on a 2048Hz frequency, and is used for the ADC and FFT
    //first we should read from the ADC to find the value of the sample
    //using readADC();
    //Because we're listening to a real valued signal, we will only write to the 're' value
    //We can keep the 'im' value at 0, as set in the declaration loop later.
    sample_buffer[sample_counter].re = readADC();
    sample_counter++;
    //Once we have the voltage value of the sample, we must put that into the buffer at the corresponding spot
    //We are taking 2048 samples per second- Each time we take a sample, we write it to the FFT buffer
    //The FFT buffer has a length N of 1024
    //Thus, every time we take in 1024 samples, we must call the FFT to find the frequency

    if (sample_counter == 1024) {
        //call compute_FFT() here;
        freq = freq_vector[compute_FFT(&sample_buffer)];

        sample_counter = 0;
    }


    IFS0CLR = 0x100000; // Clear Timer5 interrupt status flag (bit 20)
}

//Timer 2/3 type B interrupt: we can use this for the SSD/1Sec and 2Sec displays
//Note that this uses interrupt flag for Timer 3

void __ISR(_TIMER_3_VECTOR, ipl6) _T3Interrupt(void) {
    //This timer occurs on a 80Hz frequency- every time it occurs, we should switch the SSD
    //side
    left_ssd_val = !(left_ssd_val);
    time_counter_ssd++;
    //Then we need to set up counting such that every 80th interrupt it increases the seconds timer:
    if (time_counter_ssd % 80 == 0) {
        //Now, in our logic, when we press a button, we simply set both time_counter_ssd and _seconds to 0
        //and check their values on release
        SSD_blink = !SSD_blink;
        time_counter_seconds++;
        time_counter_ssd = 0;
    }
    switch (mode) {
        case 1:
            /* Pmod msd should show 's', and last 3 digits should show passcode */
            displaySSD(passcode, 5);
            break;
        case 2:
            /* Pmod msd should show 'u', and last 3 digits should show off */
            displaySSD(0, 18);
            break;
        case 3:
            /* Pmod msd should show 'L', and last 3 digits should show entered pass */
            displaySSD(guess, 17);
            break;
        case 4:
            /* Pmod msd should Flash "AAAA" */
            displaySSD(0, 4);
            break;
    }
    IFS0bits.T3IF = 0; // Clear Timer3 interrupt status flag
}


/* This is the ISR for the keypad CN interrupts */

void __ISR(_CHANGE_NOTICE_VECTOR, ipl4) ChangeNotice_Handler(void) {
    // PLEASE NOTE: Sections of this code have been REMOVED to ensure academic honesty of future students! :-)
    // 1. Disable interrupts
    // 2. Debounce keys
    // 3. Decode which key was pressed
    //First, read the inputs to clear the CN mismatch condition
    //Now, walk through the row variables setting them equal to zero
    if (pressed_key != 16 && !(button_lock)) {
        //If we had a key press and not release, activate buttonlock
        button_lock = 1;
        //Start counting how long the button is held
        if (mode == 2) {
            time_counter_ssd = 0;
        }
        //Store the pressed key in key_detected
        key_pressed = 1;
        //Store the fact that we should react to the press in a flag
        key_to_react = 1;
    }
    if (pressed_key == 16 && button_lock) {
        //If we had a key release and not press, turn off buttonlock
        button_lock = 0;
        //We started the time counter at zero on the last press, thus length held
        //is equal to the timer
        //Store the key release in a flag
        key_released = 1;
        key_to_react = 1;
        //Keep track of what the key was that we released
        last_key = key_detected;
    }
    // Return row variables to zero so we detect changes
    Row1 = Row2 = Row3 = Row4 = 0;
    //Clears interrupt flag
    // 5. Enable interrupts
}

int key_detected_toregint(int a) {
    //Translated the key_pressed number from the keypad to an integer based on
    // the values coded into the ISR function
    int to_Return = 0;
    if (a >= 0 && a <= 2) {
        to_Return = a + 1;
    } else if (a >= 4 && a <= 6) {
        to_Return = a;
    } else if (a >= 8 && a <= 10) {
        to_Return = a - 1;
    }
    return to_Return;
}

int readADC() {
    // Operation of ADC in manual mode
    AD1CON1bits.SAMP = 1; // 1. start sampling
    for (TMR1 = 0; TMR1 < 100; TMR1++); //2. wait for sampling time
    AD1CON1bits.SAMP = 0; // 3. start the conversion
    while (!AD1CON1bits.DONE); // 4. wait conversion complete
    return ADC1BUF0; // 5. read result

}

void displayDigit(unsigned char value, unsigned int left_ssd, unsigned int leftdisp) {
    //For left=1, display on left SSD, else, on right SSD
    if (left_ssd == 1) {
        if (leftdisp == 1) {
            SegA_L = value & 1;
            SegB_L = (value >> 1) & 1;
            SegC_L = (value >> 2) & 1;
            SegD_L = (value >> 3) & 1;
            SegE_L = (value >> 4) & 1;
            SegF_L = (value >> 5) & 1;
            SegG_L = (value >> 6) & 1;
            DispSel_L = 0;
        } else {
            SegA_L = value & 1;
            SegB_L = (value >> 1) & 1;
            SegC_L = (value >> 2) & 1;
            SegD_L = (value >> 3) & 1;
            SegE_L = (value >> 4) & 1;
            SegF_L = (value >> 5) & 1;
            SegG_L = (value >> 6) & 1;
            DispSel_L = 1;
        }

    } else {
        if (leftdisp == 1) {
            SegA_R = value & 1;
            SegB_R = (value >> 1) & 1;
            SegC_R = (value >> 2) & 1;
            SegD_R = (value >> 3) & 1;
            SegE_R = (value >> 4) & 1;
            SegF_R = (value >> 5) & 1;
            SegG_R = (value >> 6) & 1;
            DispSel_R = 0;
        } else {
            SegA_R = value & 1;
            SegB_R = (value >> 1) & 1;
            SegC_R = (value >> 2) & 1;
            SegD_R = (value >> 3) & 1;
            SegE_R = (value >> 4) & 1;
            SegF_R = (value >> 5) & 1;
            SegG_R = (value >> 6) & 1;
            DispSel_R = 1;
        }

    }
}

void showNumber(int digit, unsigned int left_ssd_num, unsigned int leftdisp) {
    displayDigit(SSD_number[digit % 20], left_ssd_num, leftdisp);
}

void clearSSDS() {
    displayDigit(0b0000000, 0, 0);
    displayDigit(0b0000000, 1, 0);
    displayDigit(0b0000000, 0, 1);
    displayDigit(0b0000000, 1, 1);
}

void displaySSD(int input, int mode_input) {
    //input 0 - 999 range
    if (mode == 1 || mode == 3) {
        if (left_ssd_val) {
            showNumber(input % 10, 0, 1);
            showNumber((input / 100) % 10, 1, 1);
        } else {
            showNumber((input / 10) % 10, 0, 0);
            showNumber(mode_input, 1, 0);
        }
    }
    if (mode == 2) {
        showNumber(18, 1, 0);
    }
    if (mode == 4) {
        if (SSD_blink) {
            if (left_ssd_val) {
                showNumber(10, 0, 1);
                showNumber(10, 1, 1);
            } else {
                showNumber(10, 0, 0);
                showNumber(10, 1, 0);
            }
        } else {
            clearSSDS();
        }
    }
}

main() {


    INTDisableInterrupts();
    TRISB = 0x80F;
    //ADC manual config
    AD1PCFG = 0xF7FF; // all PORTB = digital but RB11 = analog
    AD1CON1 = 0; // manual conversion sequence control
    AD1CHS = 0x000B0000; // Connect RB7/AN7 as CH0 input
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x1F02; // Tad = 128 x Tpb, Sample time = 31 Tad
    AD1CON1bits.ADON = 1; // turn on the ADC


    //Configure ports C~G to be output ports
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISF = 0;
    TRISG = 0;
    // initialize C~G to 0
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    PORTF = 0x00;
    PORTG = 0x00;

    int i;

    // Configure Timer for the ADC sampling
    // This will be a type A timer
    T5CONbits.ON = 0; // Stop timer, clear control registers
    TMR5 = 0; // Timer counter
    PR5 = 39062; //Timer count amount for interupt to occur - 2048Hz frequency
    IPC5bits.T5IP = 5; //prioirty 5
    IFS0bits.T5IF = 0; // clear interrupt flag
    T5CONbits.TCKPS = 0; // prescaler at 1:256, internal clock sourc
    T5CONbits.ON = 1; // Timer 5 module is enabled
    IEC0bits.T5IE = 1; //enable Timer 5


    //Configure Timer for the SSD display and 1 and 5 second timers respectively
    //This will be a Type B timer of TMR 2 and TMR 3
    T2CONbits.ON = 0; //Turn Timer 2 off
    T3CONbits.ON = 0; //Turn Timer 3 off
    T2CONbits.T32 = 1; //Enable 32 bit mode
    T3CONbits.TCKPS = 0; //Select prescaler = 1
    TMR2 = 0; //Clear Timer 4 register
    T3CONbits.TCS = 0; //Select internal clock
    //Event freq(80Hz)=(system frequency80MHz)/(prescaler(1)*(PR value +1))
    PR2 = 1000000; //Load period Register - 80Hz frequency
    IFS0bits.T3IF = 0; //Clear Timer 3 interupt flag
    IPC3bits.T3IP = 6; //Set priority level to 6
    IPC3bits.T3IS = 2; //Set sub priority
    IPC2bits.T2IP = 6;
    IPC2bits.T2IS = 2; 
    IEC0bits.T3IE = 1; // Enable Timer 2 interrupts
    T2CONbits.ON = 1; //Turn Timer  2 on.



    //Configure Change Notice for the keypad
    // 1. Configure CNCON, CNEN, CNPUE
    //First, turn on Change interrupts
    CNCON = 0x8000;
    //Then we want CN enable on pins 2,3,4,5
    CNEN = 0x003C;
    //We also want to enable the pull up resistors on the board
    CNPUE = 0x003C;

    // 2. Perform a dummy read to clear mismatch
    PORTB;

    // 3. Configure IPC5, IFS1, IEC1
    //These set the priority and subpriority to 4 and 1 respectively
    //This priority needs to be lower than the timer interrupts, at 5 and 6
    IPC6bits.CNIP = 4;
    IPC6bits.CNIS = 1;
    //Clear the Interrupt flag status bit
    IFS1CLR = 0x0001;
    //Enable Change Notice Interrupts
    IEC1SET = 0x0001;

    // 4. Enable vector interrupt
    INTEnableSystemMultiVectoredInt();
    //We want all the Row pins at zero so we can detect any inputs on the buttons.
    Row1 = Row2 = Row3 = Row4 = 0;



    // assign values to sample_buffer[] 
    for (i = 0; i < N; i++) {
        sample_buffer[i].re = i;
        sample_buffer[i].im = 0;
    }
    // compute the corresponding frequency/bins of each data point in frequency domain
    for (i = 0; i < N / 2; i++) {
        freq_vector[i] = i * (SAMPLE_FREQ / 2) / ((N / 2) - 1);
    }

    while (1) {
	// The output logic here was moved to the timer 3 ISR above.
	// In the while loop we placed our output/display logic at the beginning
	// and then after that had our next state logic, to avoid undesirable/undefined behavior
	// through the separation.
	
        //        switch (mode) {
        //            case 1:
        //                /* Pmod msd should show 's', and last 3 digits should show passcode */
        //                displaySSD(passcode,5);
        //                break;
        //            case 2:
        //                /* Pmod msd should show 'u', and last 3 digits should show off */
        //                displaySSD(0,18);
        //                break;
        //            case 3:
        //                /* Pmod msd should show 'L', and last 3 digits should show entered pass */
        //                displaySSD(guess,17);
        //                break;
        //            case 4:
        //                /* Pmod msd should Flash "AAAA" */
        //                displaySSD(0,4);
        //                break;
        //        }

        // Next state logic here(Logic that determines what the next machine state should be)
        switch (mode) {
            case 1:
		// Mode 1 is "setup". In this mode, the user must enter a numerical passcode from 300-999.
                // 0-9 should input digit, 'C' should clear, 'D' should delete, 
                // 'E' enter if valid input pass
                if (key_to_react) {
                    //perform various tests and actions on key accordingly
                    if (key_pressed) {
                        key_pressed = 0;
                        key_to_react = 0;
                    } else if (key_released) {
                        if ((key_detected >= 0 && key_detected <= 2) || (key_detected >= 4 && key_detected <= 6) || (key_detected >= 8 && key_detected <= 10) || key_detected == 12) {
                            if (passcode == 0) {
                                passcode = key_detected_toregint(key_detected);
                            } else if (passcode > 0 && passcode < 10) {
                                passcode = (10 * passcode) + key_detected_toregint(key_detected);
                            } else if (passcode >= 10 && passcode < 100) {
                                passcode = (10 * passcode) + key_detected_toregint(key_detected);
                            }
                        } else if (isalpha(keypad_number[key_detected])) {

                            if (keypad_number[key_detected] == 'D') {
                                passcode = (passcode - (passcode % 10)) / 10;
                            }
                            if (keypad_number[key_detected] == 'C') {
                                passcode = 0;
                            }
                            if (keypad_number[key_detected] == 'E') {
                                if (passcode >= 300 && passcode <= 999) {
                                    guess = 0;
                                    mode = 2;
                                    clearSSDS();
                                }
                            }
                        }
                        key_released = 0;
                        key_to_react = 0;
                    }
                }
                break;
            case 2:
		// mode 2 is our "armed" state. It indicates that a passcode has been set, and we can go into locked or back to setup
                // if a button is pressed and held for >=1 second, go back to mode 1 
                // if a button is pressed and held for <1 second, go to mode 3 
                if (key_to_react) {
                    if (key_released) {
                        if (length_held >= 1) {
                            mode = 1;
                            key_to_react = 0;
                            key_released = 0;
                            passcode = 0;
                        }
                        if (length_held < 1) {
                            mode = 3;
                            key_to_react = 0;
                            key_released = 0;
                        }
                    } else {
                        key_to_react = 0;
                        key_pressed = 0;
                    }
                }
                break;
            case 3:
		// mode 3 is our "locked" state. It should display the current guess on the SSD and take input from the user.
		// If the user gets the wrong password, inputs the wrong frequency or if there is a sudden loud noise, 
		// it will move to mode 4, "alarm"
                // 0-9 should input digit, 'C' should clear, 'D' should delete, 
                // 'E' enter if valid input pass. If pass is correct, enter mode 2
                // if pass incorrect, enter mode 4. If microphone frequency is equal 
                // to pass frequency, enter mode 2, else to mode 4. 
                if (changed_to_mode_3) {
                    guess = 0;
                    changed_to_mode_3 = 0;
                }

                if (first_press_in_mode_3) {
                    if (time_counter_seconds > first_press_time_mode_3 + 5) {
                        mode = 4;
                        time_counter_ssd = 0;
                        time_counter_seconds = 0;
                    }
                }
                if (key_to_react) {
                    //perform various tests and actions on key accordingly
                    if (!first_press_in_mode_3) {
                        first_press_in_mode_3 = 1;
                        first_press_time_mode_3 = time_counter_seconds;
                    }
                    if (key_released) {
			    //First, if we pressed a number on the keypad, we change the guess variable accordingly
                        if ((key_detected >= 0 && key_detected <= 2) || (key_detected >= 4 && key_detected <= 6) || (key_detected >= 8 && key_detected <= 10) || key_detected == 12) {
                            if (guess == 0) {
                                guess = key_detected_toregint(key_detected);
                            } else if (guess > 0 && guess < 10) {
				    //If the guess is 1-9, we want to go from, for ex, 9 to 99.
                                guess = (10 * guess) + key_detected_toregint(key_detected);
                            } else if (guess >= 10 && guess < 100) {
				    //Similar logic as above, but going from 99 to 993
                                guess = (10 * guess) + key_detected_toregint(key_detected);
                            }
                        }
                        if (isalpha(keypad_number[key_detected])) {
				//If we recieve an alphanumeric output, then we perform the corresponding action
				//e.g. D 'backspaces', C clears the input, E trys to enter.

                            if (keypad_number[key_detected] == 'D') {
                                guess = (guess - (guess % 10)) / 10;
                            }
                            if (keypad_number[key_detected] == 'C') {
                                guess = 0;
                            }
                            if (keypad_number[key_detected] == 'E') {
				    //If we successfully guess the passcode, then we return to mode 2, where the password can be changed
                                if (guess == passcode) {
                                    mode = 2;
                                    clearSSDS();
                                    guess = 0;
                                } else {
                                    mode = 4;
                                    time_counter_ssd = 0;
                                    time_counter_seconds = 0;
                                }
                            }
                        }
                        key_to_react = 0;
                        key_released = 0;
                    } else if (key_pressed) {
                        key_pressed = 0;
                        key_to_react = 0;
                    }

                }

                if (readADC() > 400) {
			//If the security system detects a loud noise, then we should trigger the alarm per the specifications.
			//In this case, we simply set a hardcoded threshold for noise value, but this should be changed per specific use case
			//It's also possible to keep a running average of the room noise level and then trigger only on noise
			//which is significantly louder.
                    if (freq >= .97 * passcode && freq <= 1.03 * passcode) {
			//In addition to unlocking when the passcode is input numerically, the system also unlocks when it hears a signal of the same
			//frequency as the passcode with a 3% tolerance.
                        mode = 2;
                        clearSSDS();
                        guess = 0;
                    } else {
                        mode = 4;
                        time_counter_ssd = 0;
                        time_counter_seconds = 0;
                    }
                }
                break;
            case 4:
		// Mode 4 is alarm mode. It flashes the SSD for 5 seconds on an incorrect guess and returns to mode 3, locked.
                // If SSDs have been flashing for 5 seconds, then enter mode 3.
                if (time_counter_seconds == 5) {
                    mode = 3;
                    guess = 0;
                    time_counter_ssd = 0;
                    time_counter_seconds = 0;
                    changed_to_mode_3 = 1;
                    first_press_in_mode_3 = 0;
                }


                break;
        }
    }
}

int compute_FFT(int16c *sample_buffer) {
    int i;
    int dominant_freq = 1;

    // computer N point FFT, taking sample_buffer[] as time domain inputs
    // and storing generated frequency domain outputs in dout[] 
    mips_fft16(dout, sample_buffer, fftc, scratch, log2N);

    // compute single sided fft of our input signal. 
    for (i = 0; i < N / 2; i++) {
        single_sided_fft[i] = 2 * ((dout[i].re * dout[i].re) + (dout[i].im * dout[i].im));
    }

    // find the index of dominant frequency, which is the index of the largest data points 
    for (i = 1; i < N / 2; i++) {
        if (single_sided_fft[dominant_freq] < single_sided_fft[i])
            dominant_freq = i;
    }

    return dominant_freq;
}
