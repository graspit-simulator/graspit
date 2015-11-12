#ifndef _barretthand_h_
#define _barretthand_h_

class SerialPort;

#define BARRETT_FINGER1	1
#define BARRETT_FINGER2	2
#define BARRETT_FINGER3	4
#define BARRETT_SPREAD	8
#define BARRETT_ALL_FINGERS (BARRETT_FINGER1 | BARRETT_FINGER2 | BARRETT_FINGER3)
#define BARRETT_ALL (BARRETT_ALL_FINGERS | BARRETT_SPREAD)

class BarrettHand {
public:
	enum Mode{MODE_RETURN, MODE_READBACK};
private:
	Mode mMode;

	SerialPort *port;
	void MotorCommand(int fingers, char *command);
	virtual void SkipUntilPrompt();
	void GetValuesN(int n, int &a, int &b, int &c, int &d);
	void GetValues4(int &a, int &b, int &c, int &d);
	void GetValues3(int &a, int &b, int &c);
	void GetValues2(int &a, int &b);
	void GetValues1(int &a);

	void SetDefaultPosition(int finger, int position);

public:
	BarrettHand();
	~BarrettHand();
	void SetMode(Mode m){mMode = m;}
	void HandInitialize(int fingers);
	void smoothSpreadParams();
	void Home(int fingers);
	void IncrementalOpen(int fingers, int distance);
	void IncrementalClose(int fingers, int distance);
	void Loop(int fingers);
	void Move(int fingers, int position);
	virtual void Open(int fingers);
	virtual void Close(int fingers);
	void TerminatePower(int fingers);
	void TorqueControlledClose(int fingers);
	void TorqueControlledOpen(int fingers);
	int GetFingerPosition(int finger);
	int GetBreakawayDetected(int finger);
	int GetBreakawayPosition(int finger);
	void MoveMultiple(double *destinations, double step);
	virtual void MoveTogether(double *destinations);
	int RadiansToInternal(int finger, double radians);
	double InternalToRadians(int finger, int internal);
	//for operating in multiple threads
	virtual bool isBusy(){return false;}
};

#endif