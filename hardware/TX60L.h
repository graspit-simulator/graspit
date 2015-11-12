#ifndef _TX60L_H_
#define _TX60L_H_
#include <string>
#include <vector>

#define PI 3.14
#define DEGREE_2_RADIAN(X) (X*PI/180)
#define RAIDAN_2_DEGREE(X) (X*180/PI)

class CS8ServerV0Proxy;
class CS8ServerV1Proxy;
class CS8ServerV3Proxy;
class _ns1__login;
class _ns1__loginResponse;
class ns6__Frame;
class ns6__MotionDesc;
class ns6__Config;

// class to communicate with staubli robot, unit used is Meter and Degree
class TX60L {
private:
	bool mIsLoggedIn;
	std::string mEndpoint, mEndpointV1, mEndpointV3;
	CS8ServerV0Proxy *mCS8ServerV0;
	CS8ServerV1Proxy *mCS8ServerV1;
	CS8ServerV3Proxy *mCS8ServerV3;
	_ns1__login *mLogin;
	_ns1__loginResponse *mLoginResponse;

public:
	TX60L::TX60L() : mIsLoggedIn(false), mCS8ServerV0(NULL), mLogin(NULL), mLoginResponse(NULL), mCS8ServerV1(NULL), mCS8ServerV3(NULL){}
	//------------------Server V0----------------//
	bool IsLoggedIn(){ return mIsLoggedIn; }
	bool Login(const std::string& url, const std::string& userName, const std::string& password);
	void Logoff();
	bool SetJoints(const std::vector<double>& joints);
	bool GetRobotJoints(std::vector<double>& joints);
	bool GetRobotCartesianPosition(std::vector<double>& position);
	bool GetRobots(std::vector<int>& robots);

	//-----------------Server V1-----------------//
	bool GetApplications(std::vector<std::string>& appNames);
	bool GetJointRange(std::vector<double>& min, std::vector<double>& max);

	//----------------- Server V3---------------//
	bool ResetMotion();
	bool MoveJoints(std::vector<double> jnts);
	bool InverseKinematics(std::vector<double> pos, std::vector<double> start, std::vector<double> &jnts);
	bool ForwardKinematics(std::vector<double> jnts, std::vector<double> &pos);
	bool MoveLine(std::vector<double> pos);
	bool Power(bool on);
	bool Stop();
	bool Restart();

	//----------------- Helper functions--------//
	void InitializeMotionDesc(ns6__MotionDesc * md);
	void InitializeConfig(ns6__Config * conf);
	void GetRxRyRzCoord(ns6__Frame *x_fr, double *x_Rx, double *x_Ry, double *x_Rz);
	void SetFrameFromRxRyRz(ns6__Frame *x_fr, double x_Rx, double x_Ry, double x_Rz);
};
#endif