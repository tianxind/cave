#include <iostream>
#include <GLUT/glut.h>
#include <Eigen/Dense>
#include <cmath>


using namespace std;
using namespace Eigen;

double numSubT = 10; //this is the number of pieces each spline segment will be broken into
Eigen::Matrix4d m;
Eigen::Matrix4d b;
Eigen::Matrix4d firstM;
Eigen::Matrix4d lastM;
int TOLERANCE = 0.00001f;

Eigen::Vector3d getPoint(double u, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4) {
	Eigen::Vector3d point;
	Eigen::Vector4d c;
	double usquared = u*u;
	double ucubed = usquared*u;	
	//calculate x
	Eigen::Vector4d xDim;
	xDim << p1.x(), p2.x(), p3.x(), p4.x();
	// if (first) {
	// 	c = firstM * xDim;
	// } else if (last) {
	// 	c = lastM * xDim;
	// } else {
	// 	c = m * xDim;
	// }
	c = b * xDim;
	c /= 6;
	//point.x() = c[0] + c[1]*u + c[2]*usquared + c[3]*ucubed;
	point.x() = c[3] + c[2]*u + c[1]*usquared + c[0]*ucubed;
	//calculate y
	Eigen::Vector4d yDim;
	yDim << p1.y(), p2.y(), p3.y(), p4.y();
	// if (first) {
	// 	c = firstM * yDim;
	// } else if (last) {
	// 	c = lastM * yDim;
	// } else {
	// 	c = m * yDim;
	// }
	c = b * yDim;
	c /= 6;
	//point.y() = c[0] + c[1]*u + c[2]*usquared + c[3]*ucubed;
	point.y() = c[3] + c[2]*u + c[1]*usquared + c[0]*ucubed;
	//calculate z
	Eigen::Vector4d zDim;
	zDim << p1.z(), p2.z(), p3.z(), p4.z();
	// if (first) {
	// 	c = firstM * zDim;
	// } else if (last) {
	// 	c = lastM * zDim;
	// } else {
	// 	c = m * zDim;
	// }
	c = b * zDim;
	c /= 6;
	//point.z() = c[0] + c[1]*u + c[2]*usquared + c[3]*ucubed;
	point.z() = c[3] + c[2]*u + c[1]*usquared + c[0]*ucubed;

	return point;
}

void drawSpline(vector<Vector3d> frames){
	if (frames.size() == 0) return;
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < frames.size(); i++){
		 glVertex3f(frames[i].x(), frames[i].y(), frames[i].z());
	}
	glEnd();
}

vector<Vector3d> readControlPts(string filename) {
	m << 0.0, 1.0, 0.0, 0.0,
	-0.5, 0.0, 0.5, 0.0,
	1.0, -2.5, 2, -0.5,
	-0.5, 1.5, -1.5, 0.5;

	b << -1.0, 3.0, -3.0, 1.0,
	3.0, -6.0, 3.0, 0.0,
	-3.0, 0, 3.0, 0.0,
	1.0, 4.0, 1.0, 0.0;
	// firstM << 0.0, 1.0, 0.0, 0.0,
	// 0.0 , -1.0, 1.0, 0.0,
	// 0.0, -0.5, 1.0, -0.5,
	// 0.0, 0.5, -1.0, 0.5;

	// lastM << 0.0, 1.0, 0.0, 0.0,
	// -0.5, 0.0, 0.5, 0.0,
	// 1.0, -2.0, 1.0, 0.0,
	// -0.5, 1.0, -0.5, 0.0;

	vector<Vector3d> kframes;
	vector<Vector3d> iframes;
	ifstream data;
	string line;
    data.open(filename.c_str());
    if (data.is_open()) {
        while (data.good() && !data.eof()) {
        	double x, y, z;
            getline(data, line);
            sscanf(line.c_str(), "v%lf v%lf v%lf", &x, &y, &z);
            Eigen::Vector3d ctrlPt;
            ctrlPt.x() = x; ctrlPt.y() = y; ctrlPt.z() = z;
            kframes.push_back(ctrlPt);
        }
    } else {
    	cout << "Error reading file." << endl;
    	vector<Vector3d> empty;
    	return empty;
    }

    //Do interpolation
    Eigen::Vector3d iPoint;
    //Quat q;
    bool head;
    bool tail;
    int numPts = kframes.size();

	for (int i = 0; i < numPts; i++){ //i is index of keyframe beginning segment
		for (double j = 0; j < 1; j += 1/numSubT){
			// (i == 0) ? head = true : head = false;//first segment
			// (i == kframes.size() - 2) ? tail = true : tail = false;//last segment
			iPoint = getPoint(j, kframes[(i-1 + numPts)%numPts], kframes[i], kframes[(i+1)%numPts], kframes[(i+2)%numPts]);
			iframes.push_back(iPoint);
		}
	}
	iframes.push_back(iframes[0]);

    return iframes;
}