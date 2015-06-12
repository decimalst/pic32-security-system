#A simple home security system
Note that this particular project was originally developed on a Cerebot MX4. All peripheral modules(pmods) were made by Digilent.
##Specifications:
1. Receive input via a keypad and microphone, using the change notice interrupt handler and the onboard ADC respectively
2. Display status to two seven segment displays
3. Use timer interrupts and handlers for display logic
4. Implementation of FFT to process the microphone input, and a +-3% sensitivity to frequency
5. The security system must trigger an alarm on incorrect passcode entry, incorrect frequency entry, or on loud noises

##Basic Description:
The specific behavior of the security system is relatively simple, it starts from an initial setup state where you can set a passcode from 300-999.
After setting a valid passcode, we then enter an armed state, where we can either go back to the setup state or move into the locked state.
In the locked state, we can either input the passcode numerically using the keypad, or we can use a tone generator to generate the passcode frequency within +-3% of the correct frequency: e.g. for a passcode of 400, the device should unlock for any tone from 388-412 Hz.
This particular version has parts of the logic removed at the request of my professor, as this was for a class project.
