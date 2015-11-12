#include "BarrettHand.h"
#include "SerialPort.h"
#include <stdio.h>
#include <string.h>

BarrettHand::BarrettHand()
{
	mMode = MODE_READBACK;
	port = new SerialPort(SerialPort::COM2, 9600);
	if ( port->getError() ) {
		fprintf(stderr,"Raw Barrett Hand failed to initialize serial port\n");
	}
	port->useTimeouts();
	fprintf(stderr,"Barrett Hand interface constructed\n");
}

BarrettHand::~BarrettHand()
{
	delete port;
	fprintf(stderr,"Barrett Hand interface destructed\n");
}

void BarrettHand::MotorCommand(int fingers, char *command) {
	char buf[80];

	char *point = buf;
	if(fingers & BARRETT_FINGER1) {
		*point = '1';
		point++;
	}
	if(fingers & BARRETT_FINGER2) {
		*point = '2';
		point++;
	}
	if(fingers & BARRETT_FINGER3) {
		*point = '3';
		point++;
	}
	if(fingers & BARRETT_SPREAD) {
		*point = 'S';
		point++;
	}
	*point = '\0';
	
	strncat(buf, command, sizeof(buf)-strlen(buf)-1);
	strncat(buf, "\r", sizeof(buf)-strlen(buf)-1);

	if(!port->writeString(buf)) {
		fprintf(stderr, "Write to serial port failed\n");
		return;
	}

	if (mMode == MODE_RETURN) return;

	//if(!port->readStringWithTerminator(sizeof(buf), buf, '\r')) {
	//	fprintf(stderr, "Read from serial port failed\n");
	//	return;
	//}
	//printf("\nSKIPPING OVER ECHO [");
	//for(char *debug = buf; *debug; debug++) {
	//	if(*debug >= ' ' && *debug < 127) printf("%c", *debug);
	//	else printf("%%%02X", *debug);
	//}
	//printf("]\n");
	//fflush(stdout);
}

void BarrettHand::SkipUntilPrompt() {

	if (mMode == MODE_RETURN) return;

	char buf[80];

	while(1) {
		if(!port->readNBytes(3, (unsigned char *) &buf[0])) {
			fprintf(stderr, "Read from serial port failed\n");
			return;
		}
		if(buf[0] == '=' && buf[1] == '>' && buf[2] == ' ') {
			//fprintf(stderr, "\nGOT PROMPT\n");
			break;
		}
		if(!port->readStringWithTerminator(sizeof(buf)-3, buf+3, '\r')) {
			fprintf(stderr, "Read from serial port failed\n");
			return;
		}
		/*
		printf("\nSKIPPING OVER [");
		for(char *debug = buf; *debug; debug++) {
			if(*debug >= ' ' && *debug < 127) printf("%c", *debug);
			else printf("%%%02X", *debug);
		}
		printf("]\n");
		fflush(stdout);
		*/
		if(buf[0] == 'E' && buf[1] == 'R' && buf[2] == 'R') {
			char *point = buf+3;
			while(*point && !isdigit(*point)) point++;
			int code = atoi(point);
			fprintf(stderr, "Error %i from the Barrett hand\n", code);
		}
	}
}

void BarrettHand::GetValuesN(int n, int &a, int &b, int &c, int &d) {
	char buf[80];

	if(!port->readStringWithTerminator(sizeof(buf), buf, '\r')) {
		fprintf(stderr, "Read from serial port failed\n");
		goto error;
	}

	//printf("\nGETTING VALUES [");
	//for(char *debug = buf; *debug; debug++) {
	//	if(*debug >= ' ' && *debug < 127) printf("%c", *debug);
	//	else printf("%%%02X", *debug);
	//}
	//printf("]\n");
	//fflush(stdout);

	char *point = buf;
	for(int i = 0; i < n; i++) {
		while(*point && !isdigit(*point)) point++;
		if(!point) goto error;

		int value = atoi(point);
		switch(i) {
		case 0: a = value; break;
		case 1: b = value; break;
		case 2: c = value; break;
		case 3: d = value; break;
		}

		while(isdigit(*point)) point++;
	}
	
	return;
error:
	a = -1;
	b = -1;
	c = -1;
	d = -1;
}

void BarrettHand::GetValues4(int &a, int &b, int &c, int &d) {
	this->GetValuesN(4, a, b, c, d);
}

void BarrettHand::GetValues3(int &a, int &b, int &c) {
	int placeholder;
	this->GetValuesN(3, a, b, c, placeholder);
}

void BarrettHand::GetValues2(int &a, int &b) {
	int placeholder;
	this->GetValuesN(2, a, b, placeholder, placeholder);
}

void BarrettHand::GetValues1(int &a) {
	int placeholder;
	this->GetValuesN(1, a, placeholder, placeholder, placeholder);
}

/*! Invokes the Barrett hand's "Hand Initialize" command, behaving as documented in the hand's manual.
    This must be called before using the hand for anything else, although the manual doesn't explain why. */
void BarrettHand::HandInitialize(int fingers)
{
	this->MotorCommand(fingers, "HI");
	this->SkipUntilPrompt();
}

void BarrettHand::smoothSpreadParams()
{
	MotorCommand(BARRETT_SPREAD,"FSET SAMPLE 50");
	this->SkipUntilPrompt();
	fprintf(stderr,"SAMPLE 50\n");
	MotorCommand(BARRETT_SPREAD,"FSET ACCEL 1");
	this->SkipUntilPrompt();
	fprintf(stderr,"ACCEL 1\n");
	MotorCommand(BARRETT_SPREAD,"FSET MCV 18");
	this->SkipUntilPrompt();
	fprintf(stderr,"MCV 18\n");
	MotorCommand(BARRETT_SPREAD,"FSET MOV 18");
	this->SkipUntilPrompt();
	fprintf(stderr,"MOV 18\n");
	fprintf(stderr,"All smooth spread params sent to hand\n");
}

//! Invokes the Barrett hand's "Home" command, behaving as documented in the hand's manual.
void BarrettHand::Home(int fingers)
{
	this->MotorCommand(fingers, "HOME");
	this->SkipUntilPrompt();
}

/*! Invokes the Barrett hand's "Incremental Open" command, behaving as documented in the hand's manual.
    Passing a distance of -1 uses the default distance which has been previously set by SetDefaultStep(). */
void BarrettHand::IncrementalOpen(int fingers, int distance)
{
	if(distance != -1) {
		char buf[80];
		sprintf(buf, "IO %i", distance);
		this->MotorCommand(fingers, buf);
	} else {
		this->MotorCommand(fingers, "IO");
	}
	this->SkipUntilPrompt();
}

/*! Invokes the Barrett hand's "Incremental Close" command, behaving as documented in the hand's manual.
    Passing a distance of -1 uses the default distance which has been previously set by SetDefaultStep(). */
void BarrettHand::IncrementalClose(int fingers, int distance)
{
	if(distance != -1) {
		char buf[80];
		sprintf(buf, "IC %i", distance);
		this->MotorCommand(fingers, buf);
	} else {
		this->MotorCommand(fingers, "IC");
	}
	this->SkipUntilPrompt();
}

/*! Invokes the Barrett hand's "Loop" command, behaving as documented in the hand's manual.
    Although this will correctly pass the command to the hand, it invokes complex behavior which this
	module doesn't know how to react to, so using it is a bad idea; it's only included because it's
	indubitably easier to implement now and document the fact than to rediscover the situation later. */
void BarrettHand::Loop(int fingers)
{
	this->MotorCommand(fingers, "LOOP");
	this->SkipUntilPrompt();
}

/*! Invokes the Barrett hand's "Move" command, behaving as documented in the hand's manual.
    Passing a position of -1 uses the default distance which has been previously set by SetDefaultPosition(). */
void BarrettHand::Move(int fingers, int position)
{
	if(position != -1) {
		char buf[80];
		sprintf(buf, "M %i", position);
		this->MotorCommand(fingers, buf);
	} else {
		this->MotorCommand(fingers, "M");
	}
	this->SkipUntilPrompt();
}

/*! Invokes the Barrett hand's "Open" command, behaving as documented in the hand's manual.  Note that the
	position it uses is the one which has been previously set by SetOpenTarget(). */
void BarrettHand::Open(int fingers)
{
	this->MotorCommand(fingers, "O");
	this->SkipUntilPrompt();
}

/*! Invokes the Barrett hand's "Close" command, behaving as documented in the hand's manual.  Note that the
	position it uses is the one which has been previously set by SetCloseTarget(). */
void BarrettHand::Close(int fingers)
{
	this->MotorCommand(fingers, "C");
	this->SkipUntilPrompt();
}

//! Invokes the Barrett hand's "Terminate Power" command, behaving as documented in the hand's manual.
void BarrettHand::TerminatePower(int fingers)
{
	this->MotorCommand(fingers, "T");
	this->SkipUntilPrompt();
}

//! Invokes the Barrett hand's "Torque-Controlled Close" command, behaving as documented in the hand's manual.
void BarrettHand::TorqueControlledClose(int fingers)
{
	this->MotorCommand(fingers, "TC");
	this->SkipUntilPrompt();
}

//! Invokes the Barrett hand's "Torque-Controlled Open" command, behaving as documented in the hand's manual.
void BarrettHand::TorqueControlledOpen(int fingers)
{
	this->MotorCommand(fingers, "TO");
	this->SkipUntilPrompt();
}

/*! Queries the current position of one of the Barrett hand's fingers (the spread counts as a finger).  Returns
    -1 in the event of any communication failure.  Since this function returns only a single value, it does not
	make sense to call it with a bitmask of more than one of the finger constants. */
int BarrettHand::GetFingerPosition(int finger)
{
	if (mMode = MODE_RETURN) {
		//flush the port as garbage might have accumulated

	}

	this->MotorCommand(finger, "FGET P");
	int result;
	this->GetValues1(result);
	this->SkipUntilPrompt();
	return result;
}

/*! Queries the breakaway-detected flag of one of the Barrett hand's fingers (the spread counts as a finger).  Returns
    -1 in the event of any communication failure.  Since this function returns only a single value, it does not
	make sense to call it with a bitmask of more than one of the finger constants. */
int BarrettHand::GetBreakawayDetected(int finger)
{
	if (mMode = MODE_RETURN) {
		//flush the port as garbage might have accumulated
	}
	this->MotorCommand(finger, "FGET BD");
	int result;
	this->GetValues1(result);
	this->SkipUntilPrompt();
	return result;
}

/*! Queries the stord breakaway-position of one of the Barrett hand's fingers (the spread counts as a finger).  Returns
    -1 in the event of any communication failure.  Since this function returns only a single value, it does not
	make sense to call it with a bitmask of more than one of the finger constants. */
int BarrettHand::GetBreakawayPosition(int finger)
{
	if (mMode = MODE_RETURN) {
		//flush the port as garbage might have accumulated
	}
	this->MotorCommand(finger, "FGET BP");
	int result;
	this->GetValues1(result);
	this->SkipUntilPrompt();
	return result;
}


void BarrettHand::SetDefaultPosition(int finger, int position)
{
	char buf[80];
	sprintf(buf, "FSET DP %i", position);
	this->MotorCommand(finger, buf);
	this->SkipUntilPrompt();
}

/*! This version of the function moves all motors simultaneously to the desired positions. This has to be done in
	2 steps: first the desired position is set as default for each motor. Then, all motors are intructed to move
	to the default position together.
*/
void BarrettHand::MoveTogether(double *destinations)
{
	int finger_indices[4];
	finger_indices[0] = BARRETT_SPREAD;
	finger_indices[1] = BARRETT_FINGER1;
	finger_indices[2] = BARRETT_FINGER2;
	finger_indices[3] = BARRETT_FINGER3;

	for (int i=0; i<4; i++) {
		int internal = RadiansToInternal( finger_indices[i], destinations[i] );
		SetDefaultPosition( finger_indices[i], internal);
	}

	Move(BARRETT_ALL, -1);
}

/*! Performs a move for multiple fingers, by interpolating between the current and desired positions for each
	finger, and sending a series of move-single-finger commands.  The first parameter is a pointer to four values
	representing the spread and fingers 1, 2, and 3, in that order.  The second is the step size.  All values are
	in radians.  The order of the degrees of freedom for this function is the same as what the simulated hand uses. */
void BarrettHand::MoveMultiple(double *destinations, double step) {
	int finger_indices[4];
	finger_indices[0] = BARRETT_SPREAD;
	finger_indices[1] = BARRETT_FINGER1;
	finger_indices[2] = BARRETT_FINGER2;
	finger_indices[3] = BARRETT_FINGER3;
	
	{
		int internal = this->RadiansToInternal(BARRETT_SPREAD, destinations[0]);
		this->Move(BARRETT_SPREAD, internal);
	}

	double current[4];
	for(int i = 0; i < 4; i++) {
		int internal = this->GetFingerPosition(finger_indices[i]);
		current[i] = this->InternalToRadians(finger_indices[i], internal);
	}
	
	int fingersDone = 0;
	while(fingersDone < 3) {
		int nextIterationFingersDone = 0;

		for(int i = 0; i < 4; i++) {
			if(current[i] == destinations[i]) {
				nextIterationFingersDone++;
				continue;
			}
			
			if(fingersDone != 4) {
				int sign = destinations[i] > current[i] ? 1 : -1;
				current[i] += step*sign;
				if((current[i] - destinations[i])*sign > 0)
					current[i] = destinations[i];
			} else {
				current[i] = destinations[i];
			}
			
			int internal = this->RadiansToInternal(finger_indices[i], current[i]);
			this->Move(finger_indices[i], internal);
		}

		fingersDone = nextIterationFingersDone;
	}

	for(int i = 0; i < 4; i++) {
		if(current[i] != destinations[i]) {
			int internal = this->RadiansToInternal(finger_indices[i], destinations[i]);
			this->Move(finger_indices[i], internal);
		}
	}
}


int BarrettHand::RadiansToInternal(int finger, double radians) {
	switch(finger) {
	case BARRETT_FINGER1:
	case BARRETT_FINGER2:
	case BARRETT_FINGER3:
		return (int) (radians * (17800.0 / 2.51327));
	case BARRETT_SPREAD:
		return (int) (radians * (3150.0 / 3.14159));
	default:
		return 0;
	}
}


double BarrettHand::InternalToRadians(int finger, int internal) {
	switch(finger) {
	case BARRETT_FINGER1:
	case BARRETT_FINGER2:
	case BARRETT_FINGER3:
		return ((double) internal) * (2.51327 / 17800.0);
	case BARRETT_SPREAD:
		return ((double) internal) * (3.14159 / 3150.0);
	default:
		return 0;
	}
}
