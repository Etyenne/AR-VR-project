#include "vrpn_Tracker.h"

#include <shellapi.h>
#include <tchar.h>
#include <iostream>
#include <SFML/Graphics.hpp>

using namespace std;

sf::RenderWindow* windowPtr = NULL;
sf::CircleShape* shapePtr = NULL;
sf::RenderWindow* windowPtr2 = NULL;
sf::CircleShape* shapePtr2 = NULL;


struct libgestures3D {
	int leftHandHeighState;
	int rightHandHeighState;
	int leftHandDepthState;
	int rightHandDepthState;
	int clapHandsState;
};

struct joint3d {
	double x = -100.0;
	double y = -100.0;
	double z = -100.0;
};

struct squelette3d {
	joint3d head;
	joint3d neck;
	joint3d ankleLeft;
	joint3d ankleRight;
	joint3d elbowRight;
	joint3d elbowLeft;
	joint3d footLeft;
	joint3d footRight;
	joint3d handLeft;
	joint3d handRight;
	joint3d handTipLeft;
	joint3d handTipRight;
	joint3d hipLeft;
	joint3d hipRight;
	joint3d kneeLeft;
	joint3d kneeRight;
	joint3d shoulderLeft;
	joint3d shoulderRight;
	joint3d spineBase;
	joint3d spineMid;
	joint3d spineShoulder;
	joint3d thumbLeft;
	joint3d thumbRight;
	joint3d wristLeft;
	joint3d wristRight;
};

squelette3d kinectDetect;
libgestures3D kinectGestures;
libgestures3D kinectPastGestures;


void affectSquelette3d(squelette3d* squelette, joint3d joint, int tracker) {

	// inversion of head and spine base
	// inversion of neck and spine mid
	//1 Traking du squelette complet
	switch (tracker)
	{
	case 3:
		squelette->spineBase = joint;
		//cout << "spineBase";
			break;
	case 2:
		squelette->spineMid = joint;
		//cout << "spineMid";
			break;
	case 1:
		squelette->neck = joint;
		//cout << "neck";
		break;
	case 0:
		squelette->head = joint;
		//cout << "head";
		break;
	case 4:
		squelette->shoulderLeft = joint;
		//cout << "shoulderLeft";
		break;
	case 5:
		squelette->elbowLeft = joint;
		//cout << "elbowLeft";
		break;
	case 6:
		squelette->wristLeft = joint;
		//cout << "wristLeft";
		break;
	case 7:
		//cout << "handLeft";
		squelette->handLeft = joint;
		break;
	case 8:
		squelette->shoulderRight = joint;
		//cout << "shoulderRight";
		break;
	case 9:
		squelette->elbowRight = joint;
		//cout << "elbowRight";
		break;
	case 10:
		squelette->wristRight = joint;
		//cout << "wristRight";
		break;
	case 11:
		squelette->handRight = joint;
		//cout << "handRight";
		break;
	case 12:
		squelette->hipLeft = joint;
		//cout << "hipLeft";
		break;
	case 13:
		squelette->kneeLeft = joint;
		//cout << "kneeLeft";
		break;
	case 14:
		squelette->ankleLeft = joint;
		//cout << "ankleLeft";
		break;
	case 15:
		squelette->footLeft = joint;
		//cout << "footLeft";
		break;
	case 16:
		squelette->hipRight = joint;
		//cout << "hipRight";
		break;
	case 17:
		squelette->kneeRight = joint;
		//cout << "kneeRight";
		break;
	case 18:
		squelette->ankleRight = joint;
		//cout << "ankleRight";
		break;
	case 19:
		squelette->footRight = joint;
		//cout << "footRight";
		break;
	case 20:
		squelette->spineShoulder = joint;
		//cout << "spineShoulder";
		break;
	case 21:
		squelette->handTipLeft = joint;
		//cout << "handTipLeft";
		break;
	case 22:
		squelette->thumbLeft = joint;
		//cout << "thumbLeft";
		break;
	case 23:
		squelette->handTipRight = joint;
		//cout << "handTipRight";
		break;
	case 24:
		squelette->thumbRight = joint;
		//cout << "thumbRight";
		break;
	default:
		//cout << "Error, tracker joint not defined" << " " << "\n";
		//return "Error, tracker joint not defined";
		break;
	}
}

void detectGesture3d(squelette3d *squelette, libgestures3D *kinectGestures) {
	// 2 interpretation : detection de geste // condition que le squelette soit au complet

	// Left hand heigh state
	if (squelette->handLeft.y > squelette->head.y) {
		kinectGestures->leftHandHeighState = 2;
		//cout << " left hand up " << "\n";
	}
	else if (squelette->handLeft.y > squelette->hipLeft.y) {
		kinectGestures->leftHandHeighState = 1;
		//cout << " left hand center " << "\n";
	}
	else {
		kinectGestures->leftHandHeighState = 0;
		//cout << " left hand down " << "\n";
	}

	// Left hand depth state
	if (squelette->handLeft.z < (squelette->head.z*1.1)) {
		kinectGestures->leftHandDepthState = 1;
		//cout << " left hand  " << "\n";
	}
	else {
		kinectGestures->leftHandDepthState = 0;
		//cout << " left hand  " << "\n";
	}

	// Right hand heigh state
	if (squelette->handRight.y > squelette->head.y) {
		kinectGestures->rightHandHeighState = 2;
		//cout << " right hand up " << "\n";
	}
	else if (squelette->handRight.y > squelette->hipLeft.y) {
		kinectGestures->rightHandHeighState = 1;
		//cout << " left hand center " << "\n";
	}
	else {
		kinectGestures->rightHandHeighState = 0;
		//cout << " right hand down " << "\n";
	}

	// right hand depth state
	if (squelette->handRight.z < squelette->head.z ) {
		kinectGestures->rightHandDepthState = 1;
		//cout << " left hand  " << "\n";
	}
	else {
		kinectGestures->rightHandDepthState = 0;
		//cout << " left hand  " << "\n";
	}
}

int calibrateX(double x) {
	x = x * 1980 * 1;
	return x;
}
int calibrateY(double y) {
	y = (y - 0.2) * -1 * 1080 * 1;
	return y;
}

void actionByGesture3d(libgestures3D *kinectGestures, libgestures3D *kinectPastGestures) {
	// cout << " left h " << "\n";
	// cout << " Tracker " << "\n";
	// 3 et 4 APP
	double xpaint = calibrateX(kinectDetect.handRight.x);
	double ypaint = calibrateY(kinectDetect.handRight.y);

	if (kinectGestures->leftHandHeighState != kinectPastGestures->leftHandHeighState) {
		
		if (kinectGestures->leftHandHeighState == 2) {
			cout << " left hand up  "  << "\n";

			if (windowPtr->isOpen()) {

				windowPtr->close();
			}
			else {
				
				windowPtr->create(sf::VideoMode(220, 220), "SFML works!");
				windowPtr->setPosition(sf::Vector2i(10, 50));
				shapePtr->setRadius(100.f);
				shapePtr->setPosition(10.f, 10.f);
				shapePtr->setFillColor(sf::Color::Green);
				windowPtr->clear();
				windowPtr->draw(*shapePtr);
				windowPtr->display();
			}
		}
		if (kinectGestures->leftHandHeighState == 1) {
			if (windowPtr->isOpen()) {

				shapePtr->setFillColor(sf::Color::Red);

				windowPtr->clear();
				windowPtr->draw(*shapePtr);
				windowPtr->display();
			}
			cout << " left hand center " << "\n";
		}
		if (kinectGestures->leftHandHeighState == 0) {
			if (windowPtr->isOpen()) {
				shapePtr->setFillColor(sf::Color::Blue);

				windowPtr->clear();
				windowPtr->draw(*shapePtr);
				windowPtr->display();
				
			}
			cout << " left hand down " << "\n";
		}
		kinectPastGestures->leftHandHeighState = kinectGestures->leftHandHeighState;
	}

	if (kinectGestures->rightHandHeighState != kinectPastGestures->rightHandHeighState) {
		if (kinectGestures->rightHandHeighState == 2) {
			if (windowPtr2->isOpen()) {

				windowPtr2->close();
			}
			else {
				
				windowPtr2->create(sf::VideoMode(220, 220), "SFML works!");
				windowPtr2->setPosition(sf::Vector2i(240, 50));
				shapePtr2->setRadius(100.f);
				shapePtr2->setPosition(10.f, 10.f);
				shapePtr2->setFillColor(sf::Color::Green);
				windowPtr2->clear();
				windowPtr2->draw(*shapePtr2);
				windowPtr2->display();
			}

		cout << " right hand up " << "\n";
		}
		else if (kinectGestures->rightHandHeighState == 1) {
			if (windowPtr2->isOpen()) {

				shapePtr2->setFillColor(sf::Color::Red);
				
				windowPtr2->clear();
				windowPtr2->draw(*shapePtr2);
				windowPtr2->display();
			}

			cout << " right hand up " << "\n";
		}
		else {
			if (windowPtr2->isOpen()) {
				shapePtr2->setFillColor(sf::Color::Blue);

				windowPtr2->clear();
				windowPtr2->draw(*shapePtr2);
				windowPtr2->display();
				
			}
			cout << " right hand down " << "\n";
		}
		kinectPastGestures->rightHandHeighState = kinectGestures->rightHandHeighState;
	}



	if (kinectGestures->rightHandDepthState != kinectPastGestures->rightHandDepthState) {
		if (kinectGestures->rightHandHeighState == 1) {
			
			cout << " right hand forward " << "\n";
		}
		else {
			cout << " right hand backward " << "\n";
		}
		kinectPastGestures->rightHandDepthState = kinectGestures->rightHandDepthState;
	}

	if (kinectGestures->leftHandDepthState != kinectPastGestures->leftHandDepthState) {
		if (kinectGestures->leftHandHeighState == 1) {
			cout << " left hand forward " << "\n";
		}
		else {
			cout << " left hand backward " << "\n";
		}
		kinectPastGestures->leftHandDepthState = kinectGestures->leftHandDepthState;
	}
}

int firstSensor = NULL;

void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB b)
{
	if (firstSensor == NULL) {
		firstSensor = b.sensor;
	}
	joint3d joint = { b.pos[0] , b.pos[1] , b.pos[2] };
	affectSquelette3d(&kinectDetect, joint, b.sensor);

	if (b.sensor == firstSensor) {
		detectGesture3d(&kinectDetect, &kinectGestures);
		actionByGesture3d(&kinectGestures, &kinectPastGestures);
	}
}


int main(int argc, char* argv[])
{
	// 192.168.246.10
	sf::RenderWindow window;
	windowPtr = &window;
	sf::CircleShape shape;
	shapePtr = &shape;

	sf::RenderWindow window2;
	windowPtr2 = &window2;
	sf::CircleShape shape2;
	shapePtr2 = &shape2;

	vrpn_Tracker_Remote* vrpnTracker = new vrpn_Tracker_Remote("Tracker0@localhost:3883");//Changer en local

	vrpnTracker->register_change_handler(0, handle_tracker);
	while(1)
	{
		vrpnTracker->mainloop();
		Sleep(75);
	}

	return 0;
}
