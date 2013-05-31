#include <iostream>
#include <GLUT/glut.h>
#include <Eigen/Dense>
#include <cmath>
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>

using namespace std;
using namespace Eigen;
using namespace OpenMesh;

double numSubT = 3; //this is the number of pieces each wall segment will be broken into
Eigen::Matrix4d m;
Eigen::Matrix4d b;
// Eigen::Matrix4d firstM;
// Eigen::Matrix4d lastM;
int TOLERANCE = 0.00001f;

Vec3f getPoint(double u, Vec3f p1, Vec3f p2, Vec3f p3, Vec3f p4) {
	Vec3f point;
	Vector4d c;
	double usquared = u*u;
	double ucubed = usquared*u;	
	//calculate x
	Eigen::Vector4d xDim;
	xDim << p1[0], p2[0], p3[0], p4[0];
	// if (first) {
	// 	c = firstM * xDim;
	// } else if (last) {
	// 	c = lastM * xDim;
	// } else {
	// 	c = m * xDim;
	// }
	c = m * xDim;
	//c /= 6;
	point[0] = c[0] + c[1]*u + c[2]*usquared + c[3]*ucubed;
	//point[0] = c[3] + c[2]*u + c[1]*usquared + c[0]*ucubed;
	//calculate y
	Eigen::Vector4d yDim;
	yDim << p1[1], p2[1], p3[1], p4[1];
	// if (first) {
	// 	c = firstM * yDim;
	// } else if (last) {
	// 	c = lastM * yDim;
	// } else {
	// 	c = m * yDim;
	// }
	c = m * yDim;
	//c /= 6;
	point[1] = c[0] + c[1]*u + c[2]*usquared + c[3]*ucubed;
	//point[1] = c[3] + c[2]*u + c[1]*usquared + c[0]*ucubed;
	//calculate z
	Eigen::Vector4d zDim;
	zDim << p1[2], p2[2], p3[2], p4[2];
	// if (first) {
	// 	c = firstM * zDim;
	// } else if (last) {
	// 	c = lastM * zDim;
	// } else {
	// 	c = m * zDim;
	// }
	c = m * zDim;
	//c /= 6;
	point[2] = c[0] + c[1]*u + c[2]*usquared + c[3]*ucubed;
	//point[2] = c[3] + c[2]*u + c[1]*usquared + c[0]*ucubed;

	return point;
}

void drawSpline(vector<Vec3f> frames){
	if (frames.size() == 0) return;
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < frames.size(); i++){
		 glVertex3f(frames[i][0], frames[i][1], frames[i][2]);
	}
	glEnd();
}

vector<vector<Vec3f> > readControlPts(string filename) {
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

	vector<vector<Vec3f> > kframes;
	vector<vector<Vec3f> > iframes;
	ifstream data;
	string line;
	int perRow;
    data.open(filename.c_str());
    if (data.is_open()) {
    	getline(data, line);
    	sscanf(line.c_str(),"%d", &perRow);
        while (data.good() && !data.eof()) {
        	vector<Vec3f> pts;
        	// for (int i = 0; i < perRow; i++) {
	        // 	double x, y, z;
	        //     getline(data, line);
	        //     int c = sscanf(line.c_str(), "v%lf v%lf v%lf", &x, &y, &z);
	        //     if (c > 0){
		       //      Eigen::Vec3f ctrlPt;
		       //      ctrlPt[0] = x; ctrlPt[1] = y; ctrlPt[2] = z;
		       //      pts.push_back(ctrlPt);
	        // 	} else {
	        // 		i--;
	        // 	}
	        // }
	        while (data.good() && !data.eof()) {

	        	double x, y, z;
	            getline(data, line);
	            int c = sscanf(line.c_str(), "v%lf v%lf v%lf", &x, &y, &z);
	            if (c > 0){
		            Vec3f ctrlPt;
		            ctrlPt[0] = x; ctrlPt[1] = y; ctrlPt[2] = z;
		            pts.push_back(ctrlPt);
		            // cout << x << ", " << y << ", " << z << endl;
	        	} else {
	        		break;
	        	}
	        }
	        kframes.push_back(pts);
        }
    } else {
    	cout << "Error reading file." << endl;
    	vector<vector<Vec3f> > empty;
    	return empty;
    }

    //Do interpolation
    Vec3f iPoint;
    // bool head;
    // bool tail;
    //kframes.size() should == 4
    for (int k = 0; k < 4; k++){
    	vector<Vec3f> pts;
		for (int i = 0; i < perRow; i++){ //i is index of keyframe beginning segment
			for (double j = 0; j < 1; j += 1/numSubT){
				// (i == 0) ? head = true : head = false;//first segment
				// (i == kframes.size() - 2) ? tail = true : tail = false;//last segment
				iPoint = getPoint(j, kframes[k][(i-1 + perRow)%perRow], kframes[k][i], kframes[k][(i+1)%perRow], kframes[k][(i+2)%perRow]);
				pts.push_back(iPoint);
			}
		}
		pts.push_back(pts[0]);
		iframes.push_back(pts);
	}
    return iframes;
}