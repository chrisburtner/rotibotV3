//============================================================================
// Name        : controller.cpp  | with  dot following
// Authors     : Jason N Pitt and Nolan Strait
// Version     :
// Copyright   : MIT LICENSE
// Description : robot controller for worm lifespans
//============================================================================



#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>
#include <inttypes.h>
#include <vector>
#include <SerialStream.h>
#include <fcntl.h>
#include <linux/kd.h>

#include <boost/algorithm/string.hpp> 
#include <boost/lexical_cast.hpp>



#include <fstream>
#include <iomanip>
#include <cstdio>
#include <ctime>
#include <numeric>

#include <errno.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/statvfs.h>

#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>
#include <opencv2/videoio.hpp>

//determine camera type for includes

#ifdef USE_BASLER


	#include <pylon/PylonIncludes.h>
	#include <GenApi/GenApi.h>;
	#include <pylon/BaslerUniversalInstantCamera.h>

#endif

#include "constants.h"




using namespace cv;

uint8_t *buffer;

using namespace std;
using namespace LibSerial;
using namespace boost;

#ifdef USE_BASLER
	using namespace Pylon;
	using namespace GenApi;
#endif

#define TESTING false // speeds up process for faster debugging

#define ZAXIS false // if there is a Z Axis for focusing

#define BLANKUPDATE ",,,,,,,"

#define PORT "/dev/ttyACM0" //This is system-specific"/tmp/interceptty" we need to make this configurable

#define DOUBLEFRAMES true // take two images instead of 1 to detect movement
#define DOUBLESPEED 5 //number of seconds between double frames


#define MAX_PLATES 12
#define MAX_WELLS 12
#define WELL_WAIT_PERIOD 1  //pause between wells
#define SCAN_PERIOD (TESTING ? 30 : 1800)   // time between scans (default 1200sec/20min)
#define LOAD_WAIT_PERIOD (TESTING ? 20 : 120) // default 120sec/2min
#define SCAN_COMPLETE_TIMEOUT 1800//maximum time to wait for a scan before resetting robot state 30 min

#define WELL_STATE_START 2
#define WELL_STATE_ACTIVE 1
#define WELL_STATE_STOP 0

#define ROBOT_STATE_SCANNING 1
#define ROBOT_STATE_WAIT 2
#define ROBOT_STATE_LOAD 3
#define ROBOT_STATE_DISK_FULL 4

#define MONITOR_STATE_OFF -1 // experimenter doesn't want monitoring
#define MONITOR_STATE_START -2 // experimenter requests monitoring

#define NUM_WELLS 144

#define SECONDS_IN_DAY 86400
#define SECONDS_IN_HOUR 3600

#define MAX_DISK_USAGE 95 //maximum percent disk space to tolerate before pausing the controller and 
#define DISK_USAGE_WARNING_THRESHOLD 90 //threshold for warning emails
#define DISK_WARNING_FREQ 12 //how often to send the warning emails
#define STATUS_UPDATE_FREQ 142 //how often to send update emails

//defines for frame counter flags
#define BRIGHTFIELD 1
#define GFP 2
#define CHERRY 3
#define UV 4
#define BLIND 5

#define CAPTURE_BF 0
#define CAPTURE_GFP 1
#define CAPTURE_CHERRY 2
#define CAPTURE_UV 3

#define DEFAULT_EXPOSURE 10000

#define ACCEPTABLE_JITTER 2
#define JITTER_WAIT 500
#define CALIBRATE_FREQ 200 //number of scans between calibration runs, 144 once per day

#define FRAMECYCLE 5 //sets a number of frames to grab from the camera for each still, often the first frames a garbage as the camera adjusts lighting

//focus params
#define NEGATIVE 0
#define POSITIVE 1
#define WINDOWSIZE 10
#define STARTSTEPS 128

//gain params
#define DEFAULT_GAIN 0.0f
#define GFP_GAIN 15.0f
#define UV_GAIN 15.0f
#define CHERRY_GAIN 15.0f



//GLOBALS

stringstream root_dir;

SerialStream ardu;
string datapath;
int cameranum=0;
int sendwarning =0; //email warning counter
int sendstatus = 0; //email update counter
string logfilename = datapath + "/robot.log";
ofstream logfile(logfilename.c_str(), ofstream::app);
streambuf *coutbuf = std::cout.rdbuf(); //save old buf
string VERSION = "Release 2.01";

//pylon stuff
/*
int pylonCameraActive = 0;
CGrabResultPtr ptrGrabResult; //grabptr
const uint8_t *pImageBuffer; //ptr to image  capture buffer
*/


int calibration_counter=0;
int currMonitorSlot;

bool doDotFollow = false;

//color variables for finding pink markers
int iLowH = 152;
int iHighH = 179;
int iLowS = 123; 
int iHighS = 255;
int iLowV = 36;
int iHighV = 255;



//returns the number of seconds since midnight
time_t getSecondsSinceMidnight() {
    time_t t1, t2;
    struct tm tms;
    time(&t1);
    localtime_r(&t1, &tms);
    tms.tm_hour = 0;
    tms.tm_min = 0;
    tms.tm_sec = 0;
    t2 = mktime(&tms);
    return t1 - t2;
}


// calculates current monitor slot based on time of day
int calcCurrSlot() {
	
	int currTime = getSecondsSinceMidnight() ; // get time of day
	//cout << "calccurrtime currtime =" << currTime << endl; 
	return (int) ((double) currTime / SECONDS_IN_DAY * NUM_WELLS);
}

string readArduino(void){
	string read;
	getline(ardu,read);
	return read;
}//end readArduno

int blindSend(string command){
	string read;
	ardu << command << endl;
	//delay(100);
	cout << "bout to read" << endl;
	getline(ardu,read); //wait for RR
	cout << "read it " << endl;	
}//end blind send


string sendCommand(string command){
	/*
	string read;	
	ardu << command << endl;
	cout << "Sent Serial: " << command << endl;
	getline(ardu,read); //wait for RR
	cout << "Serial Received: " << read << endl;
	sleep(1);
	*/
	
	vector<string> returnbuff;
	string read;
	ardu << command << endl;
	cout << "Sent Serial: " << command << endl;
	//wait for readysignal
	while (read.find("RR") == string::npos){ 
		getline(ardu,read);
		cout << "Serial Received: " << read << endl;
		returnbuff.push_back(read);
	}
	
	//find a temp log
	for (vector<string>::iterator citer = returnbuff.begin(); citer != returnbuff.end(); citer++){
		if ((*citer).find("C*") != string::npos){
			stringstream thetemp;
			thetemp << (*citer);
			string result;
			getline(thetemp,result,'*');
			getline(thetemp,result);
			return result;
		}//end if found temp
	} //end for each returned line
	return read;

}//end sendcommand

//returns the average of a vector of int
double avg1(std::vector<int> const& v) {
    return 1.0 * std::accumulate(v.begin(), v.end(), 0LL) / v.size();
}



void setCameraSaturation(int sat){
	


	//set camera to color:

		string camfile(datapath + string("camera.config"));
		ifstream inputfile(camfile.c_str());
		string cameradevice;
		getline(inputfile,cameradevice);
		stringstream camerasettings;
		camerasettings << "v4l2-ctl -d " << cameradevice <<" -c saturation=" << sat  << endl;
		cout << "set sat command:" << camerasettings.str() << endl;
		system(camerasettings.str().c_str());
}//end setcameratocolor


void writeToLog(string logline);
void setLamp(int intensity);
void setGFP(int intensity);
void setCherry(int intensity);
string setupCamera(string filename);


class UserStatus {

public:

	string email;
	string explist;

	UserStatus(void){
		
	}//end constructor

	void addEmail(string mail){
		email = mail;
	}

	void addExperiment(long expid, string title, long frame){
		stringstream mss;
		mss << "exp:" << expid << "," << title << ",currframe:" << frame << endl;
		explist = explist + mss.str();
	}//end addExperiment

	string getExperiments(void){
	 	return explist;
	}

};//end userstatus

class Timer {
public:
	long systemstarttime,seconds,delay;
	double msdelay;
	struct timeval start;
	bool ms;

	Timer(long sseconds){
		gettimeofday(&start, NULL);
		delay = sseconds;
		ms=0;
		msdelay=0;
	}

	Timer(void){
		gettimeofday(&start, NULL);
		delay = 0;
		ms=0;
		msdelay=0;
	}

	Timer(double msec,bool mms){
		gettimeofday(&start,NULL);
		msdelay=msec;
		ms=1;
	}

	void startTimer(long time){
		gettimeofday(&start, NULL);
		if (ms){
			msdelay=time;
		}else{
			delay=time;
		}
	}

	void startTimer(double time){
		gettimeofday(&start, NULL);
		if (ms){
			msdelay=time;
		}else{
			delay=time;
		}
	}

	double getTimeElapsed(void){
		struct timeval currtime;
		gettimeofday(&currtime, NULL);
		stringstream ss;
		ss << currtime.tv_sec;

		string seconds(ss.str());
		ss.str("");
		ss.clear();
		ss << (double)currtime.tv_usec/(double)1000000;
		string micros(ss.str());

		micros.erase(0,1);
		seconds.append(micros);
		double curr = atof(seconds.c_str());
		ss.str("");
		ss.clear();

		ss << start.tv_sec;

		string sseconds(ss.str());
		ss.str("");
		ss.clear();
		ss << (double)start.tv_usec/(double)1000000;
		string smicros(ss.str());
		smicros.erase(0,1);
		sseconds.append(smicros);
		double scurr = atof(sseconds.c_str());

		return curr-scurr;
	}

	int getSeconds(void){
		struct timeval currtime;
		gettimeofday(&currtime,NULL);
		return (int)(currtime.tv_sec-start.tv_sec);
	}

	void printTimer(void){
		struct timeval currtime;
		gettimeofday(&currtime,NULL);
		cout << (currtime.tv_sec-start.tv_sec);
	}

	bool checkTimer(void){
		struct timeval currtime;
		gettimeofday(&currtime, NULL);
		if(ms){
			if (getTimeElapsed() >= msdelay)
				return true;
			else
				return false;
		}else{
			if (currtime.tv_sec-start.tv_sec >= delay)
				return true;
			else
				return false;
		}
	}

};

class FocusReading{

	public:


	long x,y,z;
	double focus;

	FocusReading(void){
		x,y,z=0;
		focus=-1;
	}//end constructor

	FocusReading( long pz , double pfocus){
		z=pz;
		focus = pfocus;
	}//end cons


};//end class focus reading

bool FocusSort(FocusReading const& left, FocusReading const& right) {
    if (left.focus != right.focus)
        return left.focus < right.focus;
    
}//end focus sort

class Well {
public:
	string email;
	string title;
	string investigator;
	string description;
	//struct timeval starttime;
	time_t starttime;
	long currentframe;
	int active;
	int status;
	string directory;
	int startingN;	   //number of worms put onto plate
	int startingAge;   //in days;
	int busy;
	int finished;
	long expID;
	long xval;
	long yval;
	long zval; //store the last focal plane
	int plate;
	int rank;
	bool transActive;
	long activeGFP;
	long activeCherry;
	long activeUV; 
	bool timelapseActive;
	int monitorSlot;
	string wellname;
	string strain;
	int targetx; //hold the centroid coordinates of pink target
	int targety; //	
	bool hasTracking;


	Well(void){}
	//int getRank(string thewelltorank);

	Well(string inputline) {
		stringstream ss(inputline);
		string token;

		busy = 0;
		finished = 0;
		active = 1;

		//gettimeofday(&starttime, NULL);
		getline(ss, token, ',');
		expID = atol(token.c_str());
		getline(ss, token, ',');
		status = atoi(token.c_str());
		getline(ss, token, ',');
		plate = atoi(token.c_str());
		getline(ss, wellname, ',');
		getline(ss, token, ',');
		xval = atol(token.c_str());
		getline(ss, token, ',');
		yval = atol(token.c_str());
		getline(ss, directory, ',');
		getline(ss, token, ',');
		timelapseActive = atoi(token.c_str());
		getline(ss, token, ',');
		monitorSlot = atoi(token.c_str());
		getline(ss, token, ',');
		activeUV = atol(token.c_str());
		getline(ss, token, ',');
		activeGFP = atol(token.c_str());
		getline(ss, token, ',');
		activeCherry = atol(token.c_str());
		getline(ss, email, ',');
		getline(ss, investigator, ',');
		getline(ss, title, ',');
		getline(ss, description, ',');
		getline(ss, token, ',');
		startingN = atoi(token.c_str());
		getline(ss, token, ',');
		startingAge = atoi(token.c_str()); //days old the worms are at start of recording
		getline(ss, strain, ',');
		getline(ss, token, ',');
		currentframe = atol(token.c_str());
		token = "";
		getline(ss, token, ',');
		if (token != "") {
			starttime = (time_t) atol(token.c_str());
		} else { // this is only here for backwards compatilibity
			string fileToCheck;
			if (timelapseActive)
				fileToCheck = directory + string("/frame000000.png");
			else
				fileToCheck = directory + string("/day0.avi");
			struct stat t_stat;
			stat(fileToCheck.c_str(), &t_stat);
			starttime = t_stat.st_mtime;
		}

		string torank = boost::lexical_cast<string>(plate) + wellname;
		rank = getRank(torank);

		//set default target locks to -1
		targetx=-1;
		targety=-1;

		//check for presence of a loc-nar file
		string locfile;
		string targets;
		locfile= directory + "loc-nar.csv";			
		ifstream locnar(locfile.c_str());
		if (locnar.good()){
			token="";
			getline(locnar,token, ',');
			targetx = atol(token.c_str());
			token="";
			getline(locnar,token, ',');
			targety = atol(token.c_str());
			hasTracking=true;
						
		} else hasTracking=false;//end if there was a loc-nar
			
		//printDescriptionFile();
	}   //end construct

	/**
	 * Creates a new Well object.
	 * @filename : the JSON file to extract well data from
	 */
	/*Well (string filename) {
		// read in JSON from file
		ifstream s(filename);
		json j;
		s >> j;

		gettimeofday(&starttime, NULL);

		expID = j["expID"];
		status = j["status"];
		plate = j["plate"];
		wellname = j["name"];
		xval = j["x"];
		yval = j["y"];
		directory = j["dir"];
		timelapseActive = j["TLA"];
		dailymonitorActive = j["DLA"];
		email = j["email"];
		investigator = j["investigator"];
		title = j["title"];
		description = j["description"];
		startingN = j["startNum"];
		startingAge = j["startAge"];
		strain = j["strain"];
		currentframe = j["frame"];

		string torank = boost::lexical_cast<string>(plate) + wellname;
		rank = getRank(torank);
	}*/

	void setStatus(int setstat){
		status = setstat;
	}

    string printWell(void){
    	stringstream ss;
    	ss << expID << "," << status << "," << plate << "," << wellname << "," << xval
				<< "," << yval<< "," << directory
    			<< "," << timelapseActive << "," << monitorSlot << "," <<
			activeUV << "," << activeGFP << "," << activeCherry << "," 
			<< email
    			<< "," << investigator << "," << title << "," << description << "," << startingN
				<< "," << startingAge << "," << strain
    			<<  "," << currentframe << "," << starttime << endl;
    	return (ss.str());
    }


	int getRank(string thewelltorank) {

		vector<string> wellorder;

		string filename;
		filename = datapath + string("/platecoordinates.dat");
		ifstream ifile(filename.c_str());
		string readline;

		string token;

		while (getline(ifile, readline)) {
			stringstream awell(readline);
			string thewell;
			getline(awell, thewell, ',');

			wellorder.push_back(thewell);

		}   //end while lines in the file

		for (int i = 0; i < (int) wellorder.size(); i++) {
			if (thewelltorank.find(wellorder[i]) == 0)
				return (i);
		}   //end for each well

		return (0);
	}


	bool operator <(const Well str) const {
		return (rank < str.rank);
	}


	void printDescriptionFile(void){
		string filename;
		filename = directory + string("description.txt");
		ofstream ofile(filename.c_str());

		ofile << "****************************************************************\n";
		ofile << title << endl;
		ofile << email << endl;
		ofile << investigator << endl;
		ofile << description << endl;
		ofile << starttime << endl;
		ofile << currentframe << endl;
		ofile << strain << endl;
		ofile << active << endl;
		ofile << directory << endl;
		ofile << startingN << endl;	   //number of worms put onto plate
		ofile << startingAge << endl;   //in days;
		ofile << "expID:" << expID << endl;
		ofile << xval << endl;
		ofile << yval << endl;
		ofile << plate << endl;
		ofile << wellname << endl;
		ofile << activeGFP << endl;
		ofile << activeCherry << endl;
		ofile << activeUV << endl;
		ofile << "****************************************************************\n";
		ofile << "::br::" << endl;
		ofile.close();

		//change write permissions on descriptionfile so www-data can write to it
		stringstream chs;
		chs << "chmod a+wr " << filename << endl; 
		system(chs.str().c_str());
	}


	void printSpecs(void) {
		cout << "****************************************************************\n";
		cout << title << endl;
		cout << email<< endl;
		cout << investigator<< endl;
		cout << description<< endl;
		cout << starttime << endl;
		cout <<currentframe<< endl;
		cout << active<< endl;
		cout << directory<< endl;
		cout << startingN<< endl;	   //number of worms put onto plate
		cout << startingAge<< endl;   //in days;
		cout << "expID:" << expID<< endl;
		cout << xval<< endl;
		cout << yval<< endl;
		cout << plate<< endl;
		cout << wellname<< endl;
		cout << "rank " << rank <<endl;
		cout << "****************************************************************\n";
	}


	// returns age of worms in days
	int getCurrAge(void) {
		time_t current;
		time(&current);
		return (int)(current - starttime + SECONDS_IN_HOUR) / SECONDS_IN_DAY + startingAge;
	}

	int incrementFrame(int SOURCEFLAG){
	//code determines when to implement the frame counter

		if (SOURCEFLAG == BLIND) {
			currentframe++;
			return(1);
		}//end if auto inc
	
		if (SOURCEFLAG == BRIGHTFIELD && !activeGFP && !activeCherry && !activeUV){
		currentframe++;
		return(1);
		}

		if (SOURCEFLAG == UV && !activeCherry && !activeGFP){
			currentframe++;
			return(1);		
		}

		if (SOURCEFLAG == GFP && !activeCherry){
			currentframe++;
			return(1);		
		}
		if (SOURCEFLAG == CHERRY){
			currentframe++;
			return(1);
		}
	return(0);


	}//end increment flag

	int setGFPparams(void){
		setupCamera("GFP.config");

	}//end setGFPparams

	int setCherryparams(void){
		setupCamera("Cherry.config");

	}//end setCherryParams

	int setBrightfieldParams(void){
		setupCamera("camera.config");

	}//end setBrightfieldParams


	void recordTemp(string tempnum){
		stringstream ss,filename;
		time_t current;
		time(&current);
		
		ss << "TR" ;
		
		string treturn=sendCommand(ss.str());
		cout << "\ntemp =" << treturn << endl;
		
		//log the temp to a file
		filename << directory << "temp" << tempnum << ".csv";
		ofstream ofile(filename.str().c_str());
		ofile << current << "," << treturn << endl;
		ofile.close();
		
		
	}//end recordTemp

	long getExposure(int chan){

		switch(chan) {
			case CAPTURE_BF:
				return DEFAULT_EXPOSURE;
			case CAPTURE_GFP:
				return activeGFP;
			case CAPTURE_UV:
				return activeUV;
			case CAPTURE_CHERRY:
				return activeCherry;
	
		}//end switch
		return DEFAULT_EXPOSURE;


	}//end getExposure

	float getGain(int chan){

		switch(chan) {
			case CAPTURE_BF:
				return DEFAULT_GAIN;
			case CAPTURE_GFP:
				return GFP_GAIN;
			case CAPTURE_UV:
				return UV_GAIN;
			case CAPTURE_CHERRY:
				return CHERRY_GAIN;
	
		}//end switch
		return DEFAULT_EXPOSURE;


	}//end getgain


	void writeTimestamp(Mat &thisMat, string filenamepath){
		
	    				stringstream textadd;	    				
	    				stringstream filetime;

				        time_t currtime;
					time(&currtime);
					filetime << ctime(&currtime);
	    				string formattedtime(filetime.str().substr(0,filetime.str().size()-1)); //get the ctime string
	    				string fileframenumber;
	    				replace_all(formattedtime,":",".");  //strip out : for FFMPEG compat
					textadd << filenamepath << " " << formattedtime; //put filename and timestamp into text
					Point lowerleft(10,thisMat.size().height-10);
					Point rectlowerleft(5,thisMat.size().height-5);
					Point rectedge(640,thisMat.size().height-25);
					rectangle(thisMat,rectlowerleft,rectedge,(0,0,0),-1);
					putText(thisMat, textadd.str(), lowerleft,FONT_HERSHEY_COMPLEX_SMALL, 0.8,cvScalar(255, 255, 255), 1, CV_AA);

	
	}//end write time stamp


	int capture_frame(int doAlign){

		try {

			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //(CV_IMWRITE_PXM_BINARY);
			compression_params.push_back(0);

			setBrightfieldParams();

			setCameraSaturation(0);

			VideoCapture cap(cameranum); // open the default camera

			long c = 0;
			while (!cap.isOpened()) { // check if we succeeded
				if (c < 3) sleep(1);
				else {
					cout << "  camera could not be opened" << endl;
					return -1;
				}
				c++;
			} //end while not opened

			cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
			cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
			//cap.set(CV_CAP_PROP_MODE, CV_CAP_MODE_YUYV);
			cout << "cv sat set:" << cap.get(CV_CAP_PROP_SATURATION) << endl;

			if ((int)cap.get(CV_CAP_PROP_FRAME_WIDTH) != 1920
				|| (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) != 1080)
				cout << "  cannot adjust video capture properties!" << endl;

			Mat frame;
			Mat frame_gray;

			// capture frame
			cap >> frame;
			int i = 0;
			while ( i < FRAMECYCLE) {
				//sleep(1);
				cap >> frame;
				i++;
			}

			if (frame.empty()) { // no frame captured
				cout << "  unable to capture frame! (expID: " << expID << ")" << endl;
				return 0;
			}

		    

			Mat lastframe;
			Mat im2_aligned;

			stringstream filename,gfpFilename,cherryFilename;
			stringstream lastfilename;
			stringstream number;
			number << setfill('0') << setw(6) << currentframe;
			recordTemp(number.str());
			
			filename << directory << "frame" << number.str() << ".png";
			gfpFilename << directory << "GFP" << number.str() << ".png";
			cherryFilename << directory << "cherry" << number.str() << ".png";

			// for first frame try to find a pink bead and if found generate a loc-nar.csv file
			cout << "wellname:" << wellname << " currentframe:" << currentframe << endl;
			if (currentframe ==0){
				cap.set(CV_CAP_PROP_SATURATION,100);
				cout << "frame 0 cv sat set:" << cap.get(CV_CAP_PROP_SATURATION) << endl;
				
				cap >> frame;
				i = 0;
				while (frame.empty() && i < 3) {
					sleep(1);
					cap >> frame;
					i++;
				}
				Mat imgHSV;

				cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  				Mat imgThresholded;

  				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

				//morphological opening (remove small objects from the foreground)
				erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
				dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

				//morphological closing (fill small holes in the foreground)
				dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
				erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
				
				

				if (countNonZero(imgThresholded) > 300 && doDotFollow) { //if saw a bead
					string dotfile = directory + string("legendarydots.png");
					cout << "dotfile:" << dotfile << endl;	
					imwrite(dotfile, imgThresholded, compression_params);
					Moments m = moments(imgThresholded, true);
					targetx= m.m10/m.m00;
					targety= m.m01/m.m00;
					string denofearth = directory + string("loc-nar.csv");
					cout << "making locnar file:" << denofearth << endl;
					ofstream den(denofearth.c_str());
					den << targetx << "," << targety << endl;
					den.close();					
				}//end if saw a bead
				else cout << "mass was:" << countNonZero(imgThresholded);


				setCameraSaturation(0);								
				
				 


			}//end if first frame

			
			cvtColor(frame, frame_gray, CV_BGR2GRAY); //make it gray




			if (doAlign == 0) {
				//write timestamp
				writeTimestamp(frame_gray, filename.str());				


				imwrite(filename.str(), frame_gray, compression_params); //frame vs frame_gray
				cout << "frame capture" << endl;
			}

			// for other frames
			else {
				number.str("");
				number << setfill('0') << setw(6) << currentframe - 1;
			
				lastfilename << directory << "frame" << number.str() << ".pgm";
				Mat im1 = imread(lastfilename.str());
				Mat im1_gray;
				cvtColor(im1, im1_gray, CV_BGR2GRAY);

				// Define the motion model
				const int warp_mode = MOTION_TRANSLATION;

				// Set a 2x3 warp matrix
				Mat warp_matrix;
				warp_matrix = Mat::eye(2, 3, CV_32F);

				// Specify the number of iterations.
				int number_of_iterations = 5000;

				// Specify the threshold of the increment
				// in the correlation coefficient between two iterations
				double termination_eps = 1e-10;

				// Define termination criteria
				TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS,
						number_of_iterations, termination_eps);

				// Run the ECC algorithm. The results are stored in warp_matrix.
				findTransformECC(im1_gray, frame_gray, warp_matrix, warp_mode, criteria);
				warpAffine(frame, im2_aligned, warp_matrix, im1.size(),
						INTER_LINEAR + WARP_INVERSE_MAP);
				Mat im2_aligned_gray;
				cvtColor(im2_aligned, im2_aligned_gray, CV_BGR2GRAY);
				imwrite(filename.str(), im2_aligned_gray, compression_params);
				cap.release();

			}

			

			//incrementFrame(BRIGHTFIELD);
			cap.release();
		} catch (cv::Exception ex) {
			cout << " frame was bad! try again" << endl;
			return (0);
		} //end caught exception trying to load

		return (1);

	}

	int capture_GFP(int doaligner){

		try {
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //(CV_IMWRITE_PXM_BINARY);
			compression_params.push_back(0);

			//setCameraSaturation(100);
			setGFPparams();

			VideoCapture cap(cameranum); // open the default camera

			long c = 0;
			while (!cap.isOpened()) { // check if we succeeded
				if (c < 3) sleep(1);
				else {
					cout << "  camera could not be opened" << endl;
					return -1;
				}
				c++;
			} //end while not opened

			cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
			cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
			cout << "cv sat set:" << cap.get(CV_CAP_PROP_SATURATION) << endl;

			if ((int)cap.get(CV_CAP_PROP_FRAME_WIDTH) != 1920
				|| (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) != 1080)
				cout << "  cannot adjust video capture properties!" << endl;

			Mat frame;
			Mat frame_gray;

			// capture frame
			cap >> frame;
			int i = 0;
			while ( i < FRAMECYCLE) {
				//sleep(1);
				cap >> frame;
				i++;
			}

			if (frame.empty()) { // no frame captured
				cout << "  unable to capture frame! (expID: " << expID << ")" << endl;
				return 0;
			}
			Mat lastframe;
			Mat im2_aligned;

			stringstream filename,gfpFilename,cherryFilename;
			stringstream lastfilename;
			stringstream number;
			number << setfill('0') << setw(6) << currentframe;
			
			filename << directory << "frame" << number.str() << ".png";
			gfpFilename << directory << "GFP" << number.str() << ".png";
			cherryFilename << directory << "cherry" << number.str() << ".png";

			// for first frame try to find a pink bead and if found generate a loc-nar.csv file
			cout << "wellname:" << wellname << " currentframe:" << currentframe << endl;

			cvtColor(frame, frame_gray, CV_BGR2GRAY); //make it gray

			imwrite(gfpFilename.str(), frame_gray, compression_params); //frame vs frame_gray
			//incrementFrame(GFP);

		} catch (cv::Exception ex) {
			cout << " GFPframe was bad! try again" << endl;
			return (0);
		} //end caught exception trying to load

		return(1);

	}//end capture GFP

	int capture_Cherry(int doaligner){
		try {
			//incrementFrame(CHERRY);
		} catch (cv::Exception ex) {
			cout << " cherryframe was bad! try again" << endl;
			return (0);
		} //end caught exception trying to load

	}//end captureCherry

	
	int capture_UV(int doaligner){
		try {
			//incrementFrame(UV);
		} catch (cv::Exception ex) {
			cout << " UVframe was bad! try again" << endl;
			return (0);
		} //end caught exception trying to load

	}//end captureUV

#ifdef USE_BASLER
		
	double getFocusMeasure(void){
		

			// Number of images to be grabbed.
			static const uint32_t c_countOfImagesToGrab = 1;
			int gotit = 0;

			

			  try
			    {
						
				// Create an instant camera object with the camera device found first.
				//CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());
				CBaslerUniversalInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice() );
				INodeMap& nodemap = camera.GetNodeMap();
				camera.Open();
		
	       // Set the AOI:

	       // Get the integer nodes describing the AOI.
		CIntegerParameter offsetX( nodemap, "OffsetX" );
		CIntegerParameter offsetY( nodemap, "OffsetY" );
		CIntegerParameter width( nodemap, "Width" );
		CIntegerParameter height( nodemap, "Height" );
		CFloatParameter exposure(nodemap, "ExposureTime");
		CFloatParameter gamma(nodemap, "Gamma");

		
		exposure.SetValue(340.0);
		gamma.SetValue(0.55);
	       

			
				// The parameter MaxNumBuffer can be used to control the count of buffers
				// allocated for grabbing. The default value of this parameter is 10.
				camera.MaxNumBuffer = 5;

				


				// Start the grabbing of c_countOfImagesToGrab images.
				// The camera device is parameterized with a default configuration which
				// sets up free-running continuous acquisition.
				camera.StartGrabbing( c_countOfImagesToGrab);

				// This smart pointer will receive the grab result data.
				CGrabResultPtr ptrGrabResult;

				// Camera.StopGrabbing() is called automatically by the RetrieveResult() method
				// when c_countOfImagesToGrab images have been retrieved.

				bool infocus = false;
				int focuscounter = 0;

				while ( !gotit )
				{
					
				    // Wait for an image and then retrieve it. A timeout of 100 ms is used.
				    camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

				    // Image grabbed successfully?
				    if (ptrGrabResult->GrabSucceeded())
				    {
					
					const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
					
					
					gotit++;
					
					//convert ptrGRab to CV Mat
					CPylonImage target;
					Mat dst;

					    CImageFormatConverter converter;
					    converter.OutputPixelFormat=PixelType_Mono8;
					    //converter.OutputBitAlignment=OutputBitAlignment_MsbAligned;

					    converter.Convert(target,ptrGrabResult);

					    Mat src_gray(target.GetHeight(),target.GetWidth(),CV_8UC1,target.GetBuffer(),Mat::AUTO_STEP);
					  

					    if(src_gray.empty())
					    {
						cout << "Nope" << endl;
						return -1;
					    }
					    

					//just use the center of the image
					Rect roi(2236, 1324, 1000, 1000);
					Mat cropped(src_gray, roi);
					
					//compute focus lap var
					int kernel_size = 11;
					int scale = 1;
					int delta = 0;
					Laplacian(cropped, dst, CV_64F,kernel_size,scale, delta);
					Scalar mu, sigma;
					meanStdDev(dst, mu, sigma);
					double focusMeasure = sigma.val[0] * sigma.val[0];
					cout << u8"\033[2J\033[1;1H"; 
					cout << "focus lapVar = :" << focusMeasure << endl;
					return (focusMeasure);
					

				    }
				    else
				    {
					cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
					return -1;
				    }
					
				}//end while
			    }
			    catch (const GenericException &e)
			    {
				// Error handling.
				cerr << "An exception occurred." << endl
				<< e.GetDescription() << endl;
				return -1;
				
			    }

	}//end getfocusmeasure


	long focusCamera(void){

		int zdir = NEGATIVE;
		vector <double> focusvalues;
		vector <long> zvals; 
		vector <FocusReading> foci;
		vector <FocusReading> ffoci;
		int zstepsize = STARTSTEPS;
		int zwindow = WINDOWSIZE;
		long zstart = zval - ((zwindow * zstepsize)/2); 


		//gotowell goes to current Z value, grab images and measure focus around current Z
		//set to max defocus the scan through previous focal point

		stringstream cmd("");
		cmd << "M" << xval << "," << yval << "," << zstart;
		sendCommand(cmd.str());
		cmd.str("");
		
		//coarse focus
		for (int i=0; i < zwindow; i++){
		
			long znext = zstart + (i * zstepsize);
			cmd << "M" << xval << "," << yval << "," << znext;
			sendCommand(cmd.str());
			cmd.str("");
			FocusReading thispoint;
			thispoint.z = znext;
			thispoint.focus= getFocusMeasure();
			foci.push_back(thispoint);
			

			cout << thispoint.z << "," << thispoint.focus << endl;
			
		

		}//end for each image


		zstepsize=16; //0.5 of actual microstep size 32usteps
		//double maxZ = *max_element(vector.begin(), vector.end());

		sort(foci.begin(), foci.end(), &FocusSort);
		long bestz= foci.back().z;
		zstart = bestz - ( zwindow/2 * zstepsize);
		cout << "best z:" << bestz << endl;

		//fine focus
		for (int i=0; i < zwindow; i++){
			
			//cout << "zval:" << foci[i].z << " focus:" << foci[i].focus << endl;
			long znext = zstart + (i * zstepsize);
			cmd << "M" << xval << "," << yval << "," << znext;
			sendCommand(cmd.str());
			cmd.str("");
			FocusReading thispoint;
			thispoint.z = znext;
			thispoint.focus= getFocusMeasure();
			ffoci.push_back(thispoint);
			

			cout << thispoint.z << "," << thispoint.focus << endl;
			
		

		}//end for each image focus Z

		//set current z to best

		sort(ffoci.begin(), ffoci.end(), &FocusSort);
		zval= ffoci.back().z;

		cout << "updated Z =" << zval << endl; 

		

		for (int i=0; i < zwindow; i++){
			
			cout << "zval:" << ffoci[i].z << " focus:" << ffoci[i].focus << endl;
		}

		 



	}//end focus camera


	int capture_pylon_video(Timer* limitTimer, int channel) {

		// The maximum number of images to be grabbed.
		static const uint32_t c_countOfImagesToGrab = 300;
		// When this amount of image data has been written, the grabbing is stopped.
		static const int64_t c_maxImageDataBytesThreshold = 500 * 1024 * 1024;


		
		gotoWell();
		setLamp(255);		
		
		 try
		    {
			// Check if CVideoWriter is supported and all DLLs are available.
			if (!CVideoWriter::IsSupported())
			{
			    cout << "VideoWriter is not supported at the moment. Please install the pylon Supplementary Package for MPEG-4 which is available on the Basler website." << endl;
			    // Releases all pylon resources.
			   // PylonTerminate();
			    // Return with error code 1.
			    return 1;
			}

			// Create a video writer object.
			CVideoWriter videoWriter;

			// The frame rate used for playing the video (playback frame rate).
			const int cFramesPerSecond = 5;
			// The quality used for compressing the video.
			const uint32_t cQuality = 90;

			// Create an instant camera object with the first camera device found.
			//CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice() );
			CBaslerUniversalInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice() );

			// Print the model name of the camera.
			cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

			//inserted
			INodeMap& nodemap = camera.GetNodeMap();
			camera.Open();
		

		       // Get the integer nodes describing the AOI.
			CIntegerParameter offsetX( nodemap, "OffsetX" );
			CIntegerParameter offsetY( nodemap, "OffsetY" );
			CIntegerParameter width( nodemap, "Width" );
			CIntegerParameter height( nodemap, "Height" );

			
			cout << "help ffs:" << camera.LightSourcePreset.GetValue() << endl;
			camera.LightSourcePreset.SetValue("Off"); 
			cout << "woeked?:" << camera.LightSourcePreset.GetValue() << endl;
			//LightSourcePresetEnums e = camera.LightSourcePreset.GetValue();

			
			CFloatParameter exposure(nodemap, "ExposureTime");
			CFloatParameter gamma(nodemap, "Gamma");
			CFloatParameter gain(nodemap, "Gain");

			
			exposure.SetValue(getExposure(channel));
			gain.SetValue(getGain(channel));
			//lightsource.SetValue("Off");
			
			//gamma.SetValue(0.55);
		       

			cout << "gain          : " << gain.GetValue() << endl;
			cout << " exposure       : " << exposure.GetValue() << endl;
			//end inserted
			// Get the required camera settings.
			CEnumParameter pixelFormat( camera.GetNodeMap(), "PixelFormat" );

			// Optional: Depending on your camera or computer, you may not be able to save
			// a video without losing frames. Therefore, we limit the resolution:
			//width.TrySetValue( 1920, IntegerValueCorrection_Nearest );
			//nmheight.TrySetValue( 1080, IntegerValueCorrection_Nearest );

			// Map the pixelType
			CPixelTypeMapper pixelTypeMapper( &pixelFormat );
			EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue( pixelFormat.GetIntValue() );

			// Set parameters before opening the video writer.
			videoWriter.SetParameter(
			(uint32_t) width.GetValue(),
			    (uint32_t) height.GetValue(),
			    pixelType,
			    cFramesPerSecond,
			    cQuality );

			// Open the video writer.
			videoWriter.Open( "/wormbot/_TestVideo.mp4" );

			// Start the grabbing of c_countOfImagesToGrab images.
			// The camera device is parameterized with a default configuration which
			// sets up free running continuous acquisition.
			camera.StartGrabbing( c_countOfImagesToGrab, GrabStrategy_LatestImages );


			cout << "Please wait. Images are being grabbed." << endl;

			// This smart pointer will receive the grab result data.
			CGrabResultPtr ptrGrabResult;

			// Camera.StopGrabbing() is called automatically by the RetrieveResult() method
			// when c_countOfImagesToGrab images have been retrieved.
			while (camera.IsGrabbing())
			{
			    // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
			    camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException );

			    // Image grabbed successfully?
			    if (ptrGrabResult->GrabSucceeded())
			    {
				// Access the image data.
				cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
				cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
				const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
				cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

		    #ifdef PYLON_WIN_BUILD
				// Display the grabbed image.
				Pylon::DisplayImage( 1, ptrGrabResult );
		    #endif

				// If required, the grabbed image is converted to the correct format and is then added to the video file.
				// If the orientation of the image does not mach the orientation required for video compression, the
				// image will be flipped automatically to ImageOrientation_TopDown, unless the input pixel type is Yuv420p.
				videoWriter.Add( ptrGrabResult );

				// If images are skipped, writing video frames takes too much processing time.
				cout << "Images Skipped = " << ptrGrabResult->GetNumberOfSkippedImages() << boolalpha
				    << "; Image has been converted = " << !videoWriter.CanAddWithoutConversion( ptrGrabResult )
				    << endl;

				// Check whether the image data size limit has been reached to avoid the video file becoming too large.
				if (c_maxImageDataBytesThreshold < videoWriter.BytesWritten.GetValue())
				{
				    cout << "The image data size limit has been reached." << endl;
				    break;
				}
			    }
			    else
			    {
				cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << endl;
			    }
			}
		    }
		    catch (const GenericException& e)
		    {
			// Error handling.
			cerr << "An exception occurred." << endl
			    << e.GetDescription() << endl;
			
		    }


	}//end capture pylon video


	int capture_pylon(int doaligner, int channel){

		//cout << "cap pylon" << endl;

		// Number of images to be grabbed.
		static const uint32_t c_countOfImagesToGrab = 1;
		int gotit = 0;

		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //(CV_IMWRITE_PXM_BINARY);
		compression_params.push_back(0);

		  try
		    {
					cout << "try pylon" << endl;
			// Create an instant camera object with the camera device found first.
			
			CBaslerUniversalInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice() );
			INodeMap& nodemap = camera.GetNodeMap();
			camera.Open();
					/*		
				// Get camera device information.
							cout << "Camera Device Information" << endl
							    << "=========================" << endl;
							cout << "Vendor           : "
							    << CStringParameter( nodemap, "DeviceVendorName" ).GetValue() << endl;
							cout << "Model            : "
							    << CStringParameter( nodemap, "DeviceModelName" ).GetValue() << endl;
							cout << "Firmware version : "
							    << CStringParameter( nodemap, "DeviceFirmwareVersion" ).GetValue() << endl << endl;

							  // Camera settings.
					cout << "Camera Device Settings" << endl
					    << "======================" << endl;

				*/
       // Set the AOI:

       // Get the integer nodes describing the AOI.
        CIntegerParameter offsetX( nodemap, "OffsetX" );
        CIntegerParameter offsetY( nodemap, "OffsetY" );
        CIntegerParameter width( nodemap, "Width" );
        CIntegerParameter height( nodemap, "Height" );

	
	cout << "help ffs:" << camera.LightSourcePreset.GetValue() << endl;
	camera.LightSourcePreset.SetValue("Off"); 
	cout << "woeked?:" << camera.LightSourcePreset.GetValue() << endl;
	//LightSourcePresetEnums e = camera.LightSourcePreset.GetValue();

	
	CFloatParameter exposure(nodemap, "ExposureTime");
	CFloatParameter gamma(nodemap, "Gamma");
	CFloatParameter gain(nodemap, "Gain");

        
        exposure.SetValue(getExposure(channel));
	gain.SetValue(getGain(channel));
	width.SetValue(2590);
	height.SetValue(1942);
	//lightsource.SetValue("Off");
	
	//gamma.SetValue(0.55);
       

        cout << "gain          : " << gain.GetValue() << endl;
        cout << " exposure       : " << exposure.GetValue() << endl;
       


			
		
			
		
			// The parameter MaxNumBuffer can be used to control the count of buffers
			// allocated for grabbing. The default value of this parameter is 10.
			camera.MaxNumBuffer = 5;

			//camera.ExposureMode.SetValue(ExposureMode_Timed);
			cout << "USB:" << camera.IsUsb() << " GigE:" << camera.IsGigE() << endl;

			


			// Start the grabbing of c_countOfImagesToGrab images.
			// The camera device is parameterized with a default configuration which
			// sets up free-running continuous acquisition.
			camera.StartGrabbing( c_countOfImagesToGrab);

			// This smart pointer will receive the grab result data.
			CGrabResultPtr ptrGrabResult;

			// Camera.StopGrabbing() is called automatically by the RetrieveResult() method
			// when c_countOfImagesToGrab images have been retrieved.

			bool infocus = false;
			int focuscounter = 0;

			while ( !infocus) //!gotit )
			{
				cout << "count:" << gotit++ << endl; 
			    // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
			    camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

			    // Image grabbed successfully?
			    if (ptrGrabResult->GrabSucceeded())
			    {
				// Access the image data.
				//cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
				//cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
				const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
				//cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;
				//camera.StopGrabbing();
				//cout << "Frame#" << gotit << endl;

				
				//save the frame
				stringstream number,filename;
				number << setfill('0') << setw(6) << currentframe;
				recordTemp(number.str());


				//convert pylon to opencv RGB for fluor
				CPylonImage target;
				Mat dst;

			        CImageFormatConverter converter;
			        converter.OutputPixelFormat=PixelType_Mono8;
			  	converter.Convert(target,ptrGrabResult);
				Mat src_gray(target.GetHeight(),target.GetWidth(),CV_8UC1,target.GetBuffer(),Mat::AUTO_STEP);
				Mat allblack(target.GetHeight(),target.GetWidth(),CV_8UC1,Scalar(0));
				Mat colorimage(target.GetHeight(),target.GetWidth(),CV_8UC3,Scalar(0,0,0));
				vector<Mat> channels;

  				

			    


			//opencv merge B G R



				switch(channel) {

					case CAPTURE_BF:
						filename << directory << "frame" << number.str() << ".png";
						//no channels to merge, just save it 
						//CImagePersistence::Save( ImageFileFormat_Png, filename.str().c_str(), ptrGrabResult );
						writeTimestamp(src_gray,filename.str());
						imwrite(filename.str(), src_gray, compression_params);
						break;

					case CAPTURE_GFP:
						filename << directory << "GFP" << number.str() << ".png";
						channels.push_back(allblack);//blue
						channels.push_back(src_gray);//green
						channels.push_back(allblack);//red
			   			merge(channels, colorimage);
						writeTimestamp(colorimage,filename.str());
						imwrite(filename.str(), colorimage, compression_params);
						
						break;	

					case CAPTURE_UV:
						filename << directory << "UV" << number.str() << ".png";
						channels.push_back(src_gray);//blue
						channels.push_back(allblack);//green
						channels.push_back(allblack);//red
			   			merge(channels, colorimage);
						writeTimestamp(colorimage,filename.str());
						imwrite(filename.str(), colorimage, compression_params);

						break;
					case CAPTURE_CHERRY:
						filename << directory << "CHERRY" << number.str() << ".png";
						channels.push_back(allblack);//blue
						channels.push_back(allblack);//green
						channels.push_back(src_gray);//red
			   			merge(channels, colorimage);
						writeTimestamp(colorimage,filename.str());
						imwrite(filename.str(), colorimage, compression_params);

						break;

				}//end switch


				
								
				//gotit++;
				
				//incrementFrame(channel);//success increment the frame counter if needed
				return 1;
				
				

			    }
			    else
			    {
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
			    }
				
			}//end while
		    }
		    catch (const GenericException &e)
		    {
			// Error handling.
			cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
			return 0;
			
		    }
			

	}//end capture_pylon


#endif //end if use basler


	int captureVideo(Timer* limitTimer) {

		gotoWell();
		setLamp(255);

		setBrightfieldParams();

		// open input from camera
		VideoCapture input(cameranum);

		// wait for camera to open
		int attempt = 1;
		while (!input.isOpened()) {
			if (attempt <= 3) sleep(1);
			else {
				cout << "no camera found!" << endl;
				return -1;
			}
			attempt++;
		}

		input.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH);
		input.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT);
		setCameraSaturation(0);	

		// open output
		VideoWriter output;
		stringstream filename;
		filename << directory << "/day" << getCurrAge() << ".avi";
		Size size = Size(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);

		output.open(filename.str().c_str(), VideoWriter::fourcc('M', 'P', '4', '2'),
				input.get(CAP_PROP_FPS), size, true);

		if (!output.isOpened()) {
			cout << "Could not open the output video for write" << endl;
			return -1;
		}

		Timer videoTimer;
		//determine how much time is left before next timelapse scan
		int maxVideoLength = SCAN_PERIOD - limitTimer->getSeconds();
		stringstream vidtimeout;
		vidtimeout << "number of second available for video =" << maxVideoLength << endl;
		writeToLog(vidtimeout.str());

		if (maxVideoLength < 30) { videoTimer.startTimer((long)30);}
		else{		
			if (maxVideoLength < VIDEO_DURATION) videoTimer.startTimer((long)maxVideoLength-20); 
			else videoTimer.startTimer((long)VIDEO_DURATION);
		}
		


		do {
			// track length of recording

			Mat frame;
			input >> frame; // get a new frame from camera
			output << frame; // write current frame to output file

		} while (!videoTimer.checkTimer()); // stop recording at 5 minutes

		setLamp(0);
		return 0;
	}


	bool lockOnTarget(void){
		if (targetx < 0 || targety < 0) return(0); //return if no bead found originally
		int dx,dy=1000;
		stringstream cmd("");

		//set camera to color:

		setCameraSaturation(255);

		
		try {

				vector<int> compression_params;
				compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //(CV_IMWRITE_PXM_BINARY);
				compression_params.push_back(0);

				VideoCapture cap(cameranum); // open the default camera

				long c = 0;
				while (!cap.isOpened()) { // check if we succeeded
					if (c < 3) sleep(1);
					else {
						cout << "  camera could not be opened" << endl;
						return -1;
					}
					c++;
				} //end while not opened

				cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
				cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

				if ((int)cap.get(CV_CAP_PROP_FRAME_WIDTH) != 1920
					|| (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) != 1080)
					cout << "  cannot adjust video capture properties!" << endl;

	


				int jittercount=0;
		
				while (1 ){		
					//capture frame
			

				
						Mat frame;

						// capture frame
						cap >> frame;
						int i = 0;
						while (frame.empty() && i < 3) {
							sleep(1);
							cap >> frame;
							i++;
						}

						if (i == 3) { // no frame captured
							cout << "  unable to capture lock frame! (expID: " << expID << ")" << endl;
							return 0;
						}

				
			





					//get centroid of pink pixels
					Mat imgHSV;

					  cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
					 
					  Mat imgThresholded;

					  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

					 //morphological opening (remove small objects from the foreground)
					  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
					  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

					//morphological closing (fill small holes in the foreground)
					  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
					  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

					Moments m = moments(imgThresholded, true);
					int pinkcenterx= m.m10/m.m00;
					int pinkcentery= m.m01/m.m00;


		
					dx= pinkcenterx - targetx;
					dy= pinkcentery - targety;
					if (abs(dx) > 1920 || abs(dy) > 1080) {
						if (jittercount++ > JITTER_WAIT) break;
						continue;  //didn't see a dot try to capture a frame again
					}

					cout << "Well:" << wellname << " targetx:" << targetx << " targety:" << targety << " pinkcenterx:" << pinkcenterx << " pinkcentery:" << pinkcentery << endl; 


					///move the camera to lock in
					if (dx <-10 ) for (int i=0; i < log(abs(dx)); i++ ){sendCommand("W");cout << "w:" << i << "dx:" << dx << endl;}
					if (dx > 10 ) for (int i=0; i < log(abs(dx)); i++ ){sendCommand("S");cout << "s:" << i << "dx:" << dx << endl;}
					if (dy < -10) for (int i=0; i < log(abs(dy)); i++ ){sendCommand("A");cout << "a:" << i << "dy:" << dy << endl;}
					if (dy > 10 ) for (int i=0; i < log(abs(dy)); i++ ){sendCommand("D");cout << "d:" << i << "dy:" << dy << endl;} 
					if (dx <-3 ) sendCommand("W");	
					if (dx > 3 ) sendCommand("S");
					if (dy < -3) sendCommand("A");
					if (dy > 3 ) sendCommand("D");
				/*
					int x1=xval + dx;
					int y1=yval + dy;
					cmd << "M" << x1 << "," << y1;
					sendCommand(cmd.str());
					Timer locktimer;
					locktimer.startTimer((long) TARGET_WAIT_PERIOD);		
					while (!locktime.checkTimer());
				*/

				        if (jittercount++ > JITTER_WAIT || (abs(dx) < ACCEPTABLE_JITTER && abs(dy) < ACCEPTABLE_JITTER)) {
						cout << "final dx,dy:"<< dx << "," << dy << endl;
						if (abs(dx) > ACCEPTABLE_JITTER || abs(dy) > ACCEPTABLE_JITTER){
							string jitterlogname = datapath + string("jitterlog.dat");							
							ofstream jitterlog(jitterlogname.c_str(), fstream::app);
							jitterlog << "Well:" << wellname << ",dx:" << dx << ",dy:" << dy << endl;
							jitterlog.close();
						}//end log bad jitter						
						break;
					}		
				}//end while not locked

		} catch (cv::Exception ex) {

			}//end if caught exception

		//set camera to monochrome
		setCameraSaturation(0);
		
			
		
	}//end lockOnTarget

	int gotoWell(void) {
		stringstream cmd("");
		Timer waittimer;

		cmd << "M" << xval << "," << yval;
		sendCommand(cmd.str());

		cmd.str("");
		
		waittimer.startTimer((long) WELL_WAIT_PERIOD);
		while (!waittimer.checkTimer());
		if (doDotFollow) lockOnTarget();
		return 0; // should change to show success?
	} //end gotoWell

};//end class well


// all wells used in current experimentation
vector <Well*> wells;

// Holds pointers to wells which have been
// requested by experimenter to be video recorded.
Well* monitorSlots[NUM_WELLS];


// Used to sort wells to ensure snake-like pattern for taking pictures
struct rank_sort
{
    inline bool operator() (Well*& struct1, Well*& struct2)
    {
    	int rank1 = struct1->rank;
    	int rank2 = struct2->rank;
        return (rank1 < rank2);
    }
};


void raiseBeep(int pulses){
	for(int j=0; j <pulses; j++){
		for(int i=0; i < 6000; i+=60){
			stringstream beeper;
			beeper << "beep -l 3 -f " << 1000+i << endl;
			system(beeper.str().c_str());
		}
	}//end for j
}//end raise beep

void setLamp(int intensity){
	stringstream ls;
	ls << "IL" << intensity;
	sendCommand(ls.str());

}//end set lamp

void setGFP(int intensity){
	stringstream ls;
	ls << "GL" << intensity;
	sendCommand(ls.str());

}//end set lamp

void setCherry(int intensity){
	stringstream ls;
	ls << "CL" << intensity;
	sendCommand(ls.str());

}//end set lamp

void setUV(int intensity){
	stringstream ls;
	ls << "UL" << intensity;
	sendCommand(ls.str());

}//end set lamp

void chordBeep(double octave){
	stringstream beeper;
	cout << (int)((double)370 * octave);
	beeper << "beep  -l 3333 -f " << (int)((double)370 * octave);  //f#
	system(beeper.str().c_str());
	beeper.str("");
	beeper << "beep  -l 3333 -f " << (int)((double)466 * octave);  //a#
	system(beeper.str().c_str());
	beeper.str("");
	beeper << "beep  -l 3333 -f " << (int)((double)554 * octave);  //c#
	system(beeper.str().c_str());
}//end chord beep

#ifdef USE_BASLER
	int setupPylonCamera(void){
		try {

			CGrabResultPtr ptrGrabResult; //grabptr

			CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice()); //the camera device
			camera.StartGrabbing( 10);
			camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
			
			

		    // Image grabbed successfully?
		    if (ptrGrabResult->GrabSucceeded())
		    {
			cout << "fuck yeah" << endl;
			}
		} catch (const GenericException &e) {
			

		}//end exception caught
	}//end setupPylonCamera

#endif

string getMachineName(){
	stringstream mfilename;
	mfilename << datapath << "machinename";
	ifstream nf(mfilename.str().c_str());
	string thename;
	getline(nf,thename);
	return(thename);
}//end getMachinename

void sendEmail(string emailaddress, string mailsubject, string messagebody){
	stringstream cmdline;
	cmdline << "echo \"" << messagebody << "\" | mail -s '" << mailsubject << "' -aFrom:wormbot@wormbot.org " << emailaddress << endl;
	cout << cmdline.str() << endl;
	system(cmdline.str().c_str());
 
}//end send 



void sendExperimentStatus(void){

	//build email list

	vector <string> emaillist;
	vector <UserStatus> userlist;

	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++) {
			
			Well* thisWell = *citer;
			if (thisWell->status == WELL_STATE_ACTIVE ) {
				emaillist.push_back(thisWell->email);
			}//end if active
	}//end for each

	//condense email list
	std::sort(emaillist.begin(), emaillist.end());
        emaillist.erase(std::unique(emaillist.begin(), emaillist.end()), emaillist.end());

	stringstream expStatusList;

	for (vector<string>::iterator citer = emaillist.begin(); citer != emaillist.end(); citer++) {
		UserStatus thisuser;
		thisuser.addEmail((*citer));
		userlist.push_back(thisuser);
		
	}//end for each email address

	
	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++) {
			
			Well* thisWell = *citer;
			if (thisWell->status == WELL_STATE_ACTIVE ) {
				for (vector<UserStatus>::iterator liter = userlist.begin(); liter != userlist.end(); liter++) {
					if ((*liter).email == thisWell->email) {
						(*liter).addExperiment(thisWell->expID,thisWell->title,thisWell->currentframe);

					}//end if belongs to user add it
				}//end for each user
			}//end if active
	}//end for each well

	

	stringstream emailmessage;

	emailmessage << "Good day.\n This is:" << getMachineName() << "\nI'm functioning properly and I would like to give you an update on the experiments you have running on my system.  Your experiments are listed below.\n Sincerely yours,\n WormBot" << endl;  
       
	for (vector<UserStatus>::iterator citer = userlist.begin(); citer != userlist.end(); citer++) {
		cout << "email:" << (*citer).email << "list:" << (*citer).getExperiments() <<endl;
	        sendEmail((*citer).email, "WormBot Daily Status Update", emailmessage.str() + (*citer).getExperiments());
	}//end for each emaillist


}//end send experiment status

void statusUpdate(void){
	if (sendstatus++ > STATUS_UPDATE_FREQ) {
		sendExperimentStatus();
		sendstatus = 0; //reset the counter
	}//end if time to send
	
}//end statusUpdate


void sendDiskWarning(int pfull, int maxfull){

	vector <string> emaillist;
	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++) {
			
			Well* thisWell = *citer;
			if (thisWell->status == WELL_STATE_ACTIVE ) {
				emaillist.push_back(thisWell->email);
			}//end if active
	}//end for each

	std::sort(emaillist.begin(), emaillist.end());
        emaillist.erase(std::unique(emaillist.begin(), emaillist.end()), emaillist.end());

	stringstream emailmessage;

	emailmessage << "Good day.\n This is:" << getMachineName() <<  "\nI'm sorry to trouble you but this message is to notify you that there has been a fault in your WormBot system.  The hard drive holding your WormBot data is " << pfull << "% full. If the system reaches " << maxfull << "% full, your WormBot system will shutdown until you increase the free space on the system. I hope you have a pleasent day.\n Sincerely yours,\n WormBot" << endl;  
	
	for (vector<string>::iterator citer = emaillist.begin(); citer != emaillist.end(); citer++) {
		cout << "email:" << *citer <<endl;
	        sendEmail(*citer, "ALERT: YOUR WORMBOT IS NEARLY FULL", emailmessage.str());
	}//end for each emaillist
				


	
}//end sendDiskWarning

void sendDiskFull(int pfull, int maxfull){

	vector <string> emaillist;
	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++) {
			
			Well* thisWell = *citer;
			if (thisWell->status == WELL_STATE_ACTIVE ) {
				emaillist.push_back(thisWell->email);
			}//end if active
	}//end for each

	std::sort(emaillist.begin(), emaillist.end());
        emaillist.erase(std::unique(emaillist.begin(), emaillist.end()), emaillist.end());

	stringstream emailmessage;

	emailmessage << "Good day.\n  This is:" << getMachineName() <<  "\nI'm sorry to trouble you but this message is to notify you that there has been a fault in your WormBot system.  The hard drive holding your WormBot data is " << pfull << "% full. This is greater than the " << maxfull << "% threshold to insure your system runs without error. Your WormBot system has now paused data collection until you increase the free space on the system. I hope you have a pleasent day.\n Sincerely yours,\n WormBot" << endl;  
	
	for (vector<string>::iterator citer = emaillist.begin(); citer != emaillist.end(); citer++) {
		cout << "email:" << *citer <<endl;
	        sendEmail(*citer, "ALERT: YOUR WORMBOT IS FULL, EXPERIMENTS PAUSED", emailmessage.str());
	}//end for each emaillist


}//end sendDIskFull


int checkDiskFull(void){

	   struct statvfs mystats;
	   

	   statvfs(datapath.c_str(), &mystats);

	   unsigned long long totalblocks = mystats.f_blocks * mystats.f_bsize;
	   unsigned long long freeblocks = mystats.f_bavail * mystats.f_bsize;
	   
	    	
	   float pfree = ((mystats.f_blocks - mystats.f_bavail) / (double)(mystats.f_blocks) * 100.0);
	   cout << "total:" << totalblocks << " free:" << freeblocks << " pfree:" << pfree << endl; 
	   
		
	   string msg = "Drive space remaining:";
	   cout << msg << (int)pfree << endl; 

	   int percentfree = pfree;
	 	
	   if (percentfree >= DISK_USAGE_WARNING_THRESHOLD) {
		if (sendwarning ==0) sendDiskWarning(percentfree, MAX_DISK_USAGE);
		if (sendwarning++ > DISK_WARNING_FREQ) sendwarning=0;  
	   }//send email warning periodically

	   if (percentfree >  MAX_DISK_USAGE) {
		sendDiskFull(percentfree, MAX_DISK_USAGE);
		return 1; 
		}else return 0;



}//end check disk full

// Scan through experiments and capture pictures
void scanExperiments(void) {
	stringstream cmd("");
	int align = 0;  //toggle alignment
	//int count = 0;

	// sort wells
	//std::sort(wells.begin(), wells.end(), rank_sort());

	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++) {
		//int captured = 0;
		//cout << "count:" << count++ << endl;
		cmd.str("");
		bool firstframe=true;
		Well* thisWell = *citer;

		takesecond: //goto take a second image

		


				if (thisWell->status == WELL_STATE_ACTIVE && thisWell->timelapseActive) {
					thisWell->gotoWell();
					setLamp(255);
					int captured = 0;
					#ifdef USE_BASLER
						if (ZAXIS) thisWell->focusCamera();
					#endif
					thisWell->gotoWell(); // goto best focus
					while (captured != 1) {
						#ifdef USE_BASLER
							captured = thisWell->capture_pylon(align, CAPTURE_BF);
						#else
							captured = thisWell->capture_frame(align);
						#endif
						
					}
					setLamp(0);
				}//end if timelapse active

				if (thisWell->status == WELL_STATE_ACTIVE && thisWell->activeUV > 0) {
					thisWell->gotoWell();
					
					setUV(255);
					int captured = 0;
					while (captured != 1) {
						#ifdef USE_BASLER
							captured = thisWell->capture_pylon(align, CAPTURE_UV);
						#else
							captured = thisWell->capture_frame(align);
						#endif
						
					}
					setUV(0);
				}//end if uv active
				
				if (thisWell->status == WELL_STATE_ACTIVE && thisWell->activeGFP >0) {
					thisWell->gotoWell();
					
					setGFP(255);
					int captured = 0;
					while (captured != 1) {
						#ifdef USE_BASLER
							captured = thisWell->capture_pylon(align, CAPTURE_GFP);
						#else
							captured = thisWell->capture_frame(align);
						#endif
						
					}
					setGFP(0);
				}//end if gfp active

				if (thisWell->status == WELL_STATE_ACTIVE && thisWell->activeCherry > 0) {
					thisWell->gotoWell();
					
					setCherry(255);
					int captured = 0;
					while (captured != 1) {
						#ifdef USE_BASLER
							captured = thisWell->capture_pylon(align, CAPTURE_CHERRY);
						#else
							captured = thisWell->capture_frame(align);
						#endif
						
					}
					setCherry(0);
				}//end if cherry active
				thisWell->incrementFrame(BLIND);
				
				if (DOUBLEFRAMES && firstframe){
					Timer doubletimer((long)0);
					firstframe=false;
					doubletimer.startTimer((long)DOUBLESPEED);
					while(!doubletimer.checkTimer()){} //wait until timer expires
					goto takesecond;

				}//end if need double
					

				
	}//end for each
}//end scanExperiments


bool addMonitorJob(Well* well) {
	if (well->monitorSlot == MONITOR_STATE_START) {
		// find open monitor slot starting from the current time
		int i = calcCurrSlot();
		while (i < currMonitorSlot + NUM_WELLS) {
			int thisSlot = i % NUM_WELLS;
			if (monitorSlots[thisSlot] == NULL) {
				monitorSlots[thisSlot] = well;
				well->monitorSlot = thisSlot;
				return true;
			}//end if slot was empty
			i++;
		}//end while slots to scan through
	} else { // slot was already assigned in joblist 
		int slot = well->monitorSlot;
		if (monitorSlots[slot] != NULL)
			cout << "Monitor slot " << slot << " overwritten: two experiments assigned same slot" << endl;
		monitorSlots[slot] = well;
		return true;
	}

	// If this point is reached, all monitor slots must be full
	cout << "No open monitor slot found for experiment " << well->expID << endl;
	return false;
}


void removeMonitorJob(Well* well) {
	monitorSlots[well->monitorSlot] = NULL;
	well->monitorSlot = MONITOR_STATE_OFF;
}

void printCurrExperiments(void){
	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++){
		//(*citer).printSpecs();
		(*citer)->printSpecs();
	}//end for each well
}//end printCurrExperiments


void setVfl2(string setting, string cameradev) {
	stringstream camerasettings;
	camerasettings << "v4l2-ctl -d " << cameradev <<" -c " << setting << endl;
	system(camerasettings.str().c_str());
	cout << setting <<endl;
}


string setupCamera(string configFilename) {
	string camfile(datapath + configFilename);
	ifstream inputfile(camfile.c_str());
	string configline;
	string cameradevice;
	getline(inputfile,cameradevice);
	while (getline(inputfile, configline)){
		setVfl2(configline,cameradevice);
	}
	return cameradevice;
}


string fdGetLine(int fd) {
	stringstream ss;
	char c[1];
	while(1){
		if (read(fd,c,1)==0) break;
		ss << c[0];
		if (c[0] == '\n' ) break;
	}
	return ss.str();
}


string fdGetFile(int fd) {
	stringstream ss;
	char c[1];
	while(1){
		if (read(fd,c,1)==0) break;
		ss << c[0];
	}
	return ss.str();
}


int checkJoblistUpdate(void) {

	string filename = datapath + "RRRjoblist.csv";
	cout << "  opening " << filename.c_str() << endl;
	int fd = open (filename.c_str(), O_RDONLY);

	// exit if file not found
	if (fd == -1) {
		cout << "  failed to find joblist\n";  exit(EXIT_FAILURE);
	}

	/* Initialize the flock structure. */
	struct flock lock;
    memset (&lock, 0, sizeof(lock));
	lock.l_type = F_RDLCK;

	/* Place a read lock on the file. Blocks if can't get lock */
	//fcntl (fd, F_SETLKW , &lock);
	//cout <<"locked...checkjoblistupdate\n ";

	//size_t len = 10; //read in first 10 bytes
	 
	string fileheader = fdGetLine(fd);

	lock.l_type = F_UNLCK;
	//fcntl(fd,F_SETLK, &lock);
	close(fd);

	if (fileheader.find("UPDATE") == string::npos)
		return (0);
	else
		return (1);
}



// Load experiments from the joblist
// @param init indicates we've just turned on the robot with true
void syncWithJoblist(bool init = false) {

	string filename = datapath + "RRRjoblist.csv";
	cout << filename << endl;
	int fd = open(filename.c_str(), O_RDONLY);

	if (fd == -1) {
		cout << "  failed to find joblist" << endl;
		exit(EXIT_FAILURE);
	} //exit if file gone

	/* Initialize the flock structure. */
	struct flock lock;
	memset(&lock, 0, sizeof(lock));
	lock.l_type = F_RDLCK;

	/* Place a read lock on the file. Blocks if can't get lock */
	// fcntl (fd, F_SETLKW, &lock);

	const string file = fdGetFile(fd);

	stringstream ss(file); //dump joblist into stringstream

	// remove the header
	string fileheader;
	getline(ss, fileheader);
	bool updated = false;
	if (fileheader.find("UPDATE") != string::npos) updated = true;

	stringstream oss; // stream to output as joblist
	oss << string(BLANKUPDATE) << endl; // append blank header

	string experimentline;
	while (getline(ss, experimentline)) {

		Well* thisWell = new Well(experimentline);

		if (thisWell->status == WELL_STATE_STOP) {
			if (init) continue;
			// find experiment and remove
			for (vector<Well*>::iterator citer = wells.begin();
					citer != wells.end(); citer++) {
				if (thisWell->expID == (*citer)->expID) {
					(*citer)->printDescriptionFile();
					if ((*citer)->monitorSlot >= 0)
						removeMonitorJob(*citer);
					wells.erase(citer);
					break;
				}
			}
		}

		else if (thisWell->status == WELL_STATE_START) {

			if (thisWell->monitorSlot != MONITOR_STATE_OFF)
				addMonitorJob(thisWell);

			thisWell->setStatus(WELL_STATE_ACTIVE);
			wells.push_back(thisWell);
		}

		else if (init) { // well is active but uninitialized
			wells.push_back(thisWell);

			if (thisWell->monitorSlot != MONITOR_STATE_OFF)
				addMonitorJob(thisWell);
		}
	}

	// iterate over active wells and update joblist and description files
	for (vector<Well*>::iterator citer = wells.begin(); citer != wells.end(); citer++) {
		oss << (*citer)->printWell();
		(*citer)->printDescriptionFile();
	}

	// unlock joblist for read-only
	lock.l_type = F_UNLCK;
	// fcntl (fd, F_SETLKW, &lock);
	close(fd);

	// Open joblist for write-only and lock
	fd = creat(filename.c_str(), O_WRONLY);
	lock.l_type = F_WRLCK;
	// fcntl (fd, F_SETLKW, &lock);

	if (updated) {
		cout << "\n*********UPDATED JOB LIST*********" << endl;
		cout << oss.str();
		cout << "**********************************\n" << endl;
	}

	FILE *stream;
	if ((stream = fdopen(fd, "w")) == NULL) {
		cout << "  joblist not found" << endl;
		close(fd);
		return;
	}
	fputs(oss.str().c_str(), stream); // output to joblist
	fclose(stream);

	// unlock joblist for write-only
	lock.l_type = F_UNLCK;
	// fcntl (fd, F_SETLKW, &lock);
	close(fd);
}

const string getCurrTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

void writeToLog(string logline){
	string fn = datapath + "/runlog";
	ofstream ofile(fn.c_str(), std::ofstream::app);
	ofile << getCurrTime() << " " << logline << endl;
	ofile.close();
}



void eraseLog(void){
	string fn = datapath + "/runlog";
	ofstream ofile(fn.c_str());
	ofile << "start log" <<endl;
	ofile.close();
}



int main(int argc, char** argv) {

	if (argc >1) {
		
		for (int i=0; i < argc; i++){
			string args(argv[i]);
			//cout << "args:" << args << endl;
			if (args.find("-df") != string::npos){
				 doDotFollow = true; //do dot following if -df flag is present
				cout << "tracking pink dots" << endl;
				}
			if (args.find("-daemon") != string::npos){
				 
			
				cout << "starting as daemon" << endl;
				cout.rdbuf(logfile.rdbuf()); //redirect std::cout to out.txt!
				daemon(0,1);
				}
		}//end for each argument
	} else cout << "not tracking dots, no daemon" << endl;

	//datapath = "/var/www/wormbot/experiments";

//read in path from /usr/lib/cgi-bin/data_path
	ifstream pathfile("/usr/lib/cgi-bin/data_path");
	getline (pathfile,datapath);
	pathfile.close();
		

 
	// testing
	//syncWithJoblist(true);

	ifstream t("var/root_dir");
	root_dir << t.rdbuf();

	bool skipIntro = true;

	if (!skipIntro) {
		raiseBeep(10);
		chordBeep(1);
		string msg = "cd " + datapath + "; ./play.sh mario.song";
		system(msg.c_str());
	}

	//

	string read;
	string camera;
	string machineZero("ZZ");
	string machineMax("LL");
	string machineCalibrate("CC");
	string machineQCCal("QC");
	string machineRestX("MX150");
	string machineRestY("MY150");
	string machineRest("M5000,5000");
	
	cout << "OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;
	string arduinoport(PORT);
	//eraseLog();

	cout << "********************************************************************" << endl;
	cout << "Kaeberlein Robot controller " << VERSION << endl
		 << "Jason N Pitt and Nolan Strait : Kaeberlein Lab : http://wormbot.org" << endl;
	cout << "********************************************************************" << endl;

	cout << "setting camera parameters \n";
	string camconfigFilename("camera.config");
	camera = setupCamera(camconfigFilename);
	cameranum = boost::lexical_cast<int>(camera[camera.length() - 1]);
	cout << "camera number:" << cameranum << " " << camera << endl;

	#ifdef USE_BASLER
		cout << "Init Pylon runtime" << endl;
		PylonInitialize();
		cout << "Setup Pylon Camera" << endl;
		//setupPylonCamera();
	#endif 
	

	int portnum = 0;

	int robotfound = 0;

	

	

	ScanPort: while (!robotfound) {

		ardu.Open(arduinoport);

		ardu.SetBaudRate(SerialStreamBuf::BAUD_9600);
		ardu.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
		ardu.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_HARD);

		cout << "Set baud rate 9600 and char size 8 bits\n Waiting for Robot to be ready." << endl;

		sleep(20); //wait for robot to align itself
		
		sendCommand("M5000,5000");		
		sendCommand("ZZ");
		//sendCommand("CC");
		sendCommand("M5000,5000");
		
		//turn off lamps
		sendCommand("IL0");
		sendCommand("GL0");
		sendCommand("CL0");
		
		robotfound=true;

		/* wait for scanner
		Timer startupwait;
		startupwait.startTimer(long(120));
		while (read.find("RR") == string::npos) {
			getline(ardu, read);
			if (startupwait.checkTimer()) {
				portnum = (int) arduinoport[11];
				portnum++;
				arduinoport[11] = portnum;

				cout << "Gave up scanning port: " << arduinoport << endl;
				ardu.Close();
				goto ScanPort;
			} // end if waited too long
		}
		robotfound = 1;
		*/

	} 


	// initialize record slots to null
	for (int i = 0; i < NUM_WELLS; i++) monitorSlots[i] = NULL;

	currMonitorSlot = calcCurrSlot();
	writeToLog("startup currMonitorSlot=");
	writeToLog(boost::lexical_cast<string>(currMonitorSlot));

	
	string msg;
	msg = "Loading experiments from joblist...";
	cout << msg << endl;
	writeToLog(msg);

	syncWithJoblist(true);

	checkDiskFull();
	sendExperimentStatus();


	// ***ROBOT STATE MACHINE***

	Timer loadTimer((long)0);
	Timer scanTimer((long)0);
	Timer fullTimer((long)0);
	int robotstate = ROBOT_STATE_SCANNING;
	bool updated = false;

	while (true) { // no kill state exists

		switch (robotstate) {


		// Cycle through wells and take pictures
		case ROBOT_STATE_SCANNING:
		   {

			
			raiseBeep(3);

			scanTimer.startTimer((long) SCAN_PERIOD);			

			if (calibration_counter++ > CALIBRATE_FREQ) {
				msg = "Calibrating...";
				cout << msg << endl;
				writeToLog(msg);
				sendCommand(String("CC")); //run axis calibration in firmware
				calibration_counter=0; //reset the counter


			}//end if need to calibrate

			

			msg = "Scanning...";
			cout << msg << endl;
			writeToLog(msg);

			
			

			// iterate over experiments
			scanExperiments();

			// zero the wormbot
			//sendCommand(machineRest);
			//sendCommand(String("CC")); //just calibrate it instead
			

	
			stringstream pdebg;
			pdebg << "precondition currMonitorSlot=" << currMonitorSlot << " calcurrslot()=" << calcCurrSlot() << " \n";
			writeToLog(pdebg.str());

			
				
			// check monitor slot
				//currMonitorSlot = calcCurrSlot();
				Well* currWell = monitorSlots[currMonitorSlot];
				stringstream debg;
				debg <<	"True : currMonitorSlot=" << currMonitorSlot << " calcurrslot()=" << calcCurrSlot() << " \n";						
				writeToLog(debg.str());
				if (currWell != NULL) {
					cout << "  capturing video for monitor slot " << currMonitorSlot
						 << " (expID: " << currWell->expID << ")" << endl;
					#ifdef USE_BASLER					
						currWell->capture_pylon_video(&scanTimer,CAPTURE_BF);
					#else
						currWell->captureVideo(&scanTimer);
					#endif

					// start video analysis
					/*
					stringstream cmd;
					cmd << "sudo /usr/lib/cgi-bin/wormtracker " // location of wormtracker
						<< currWell->directory << "/day" << currWell->getCurrAge()
						<< ".avi" << " " // video file
						<< currWell->directory // dir to put analysis data
						<< " &"; // run process in background
					system(cmd.str().c_str());*/

				} else {
					cout << "  skip video for monitor slot " << currMonitorSlot
						 << " (no well found)" << endl;
				}
				if (++currMonitorSlot >= NUM_WELLS) currMonitorSlot=0; 
				debg <<	" post capture currMonitorSlot=" << currMonitorSlot << " \n";
			

			// zero the plotter
			sendCommand(machineZero);

			robotstate = ROBOT_STATE_WAIT;

			break; //end state scanning
		  }


		// robot is in between scans
		case ROBOT_STATE_WAIT:

			

			msg = "Syncing with joblist...";
			cout << msg << endl;
			writeToLog(msg);

			updated = checkJoblistUpdate();

			syncWithJoblist();

			statusUpdate();
			if (checkDiskFull()){
				msg= "WARNING DRIVE FULL";
				cout << msg << endl;
				writeToLog(msg);
				robotstate = ROBOT_STATE_DISK_FULL;
				break; 
			}//end checkDisk

			// Check joblist for updates
			if (updated) {
				
				robotstate = ROBOT_STATE_LOAD;
			} else {
				
				cout << "Waiting..." << endl;
				while (!scanTimer.checkTimer()) {}
				robotstate = ROBOT_STATE_SCANNING;
			}

			break; //end state wait

		//if drive is full
		case ROBOT_STATE_DISK_FULL:
			while(checkDiskFull()){
				fullTimer.startTimer((long)SCAN_PERIOD);

				while(!fullTimer.checkTimer()){} //wait until timer expires
			}//end while full
			robotstate = ROBOT_STATE_SCANNING;
		
			break;
			//end if full

		// Wait for experimenter to load/unload plates
		case ROBOT_STATE_LOAD:

			

			msg = "Preparing to load/unload plates...";
			cout << msg << endl;
			writeToLog(msg);

			sendCommand("ZZ");

			msg = "Please load/unload plates";
			cout << msg << endl;

			loadTimer.startTimer((long) LOAD_WAIT_PERIOD);
			while (!loadTimer.checkTimer()) {}

			for (int j = 1; j < 4; j++) chordBeep(double(j));

			msg = "Ending load period... Keep clear of machine";
			cout << msg << endl;

			raiseBeep(10);

			// sort wells
			std::sort(wells.begin(), wells.end(), rank_sort());

			sendCommand(machineZero);

			robotstate = ROBOT_STATE_SCANNING;

			currMonitorSlot = calcCurrSlot();

			cout << "Waiting..." << endl;
			while (!scanTimer.checkTimer()) {}

			break; //end state load

		}

	} //end while robot not killed

	cout << "shutdown" << endl;

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

