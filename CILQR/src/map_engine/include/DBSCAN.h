#if !defined(_DBSCAN_H_INCLUDED_)
#define _DBSCAN_H_INCLUDED_


#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <map>

using namespace std;

namespace dbscan
{

enum
{
	pointType_UNDO,
	pointType_NOISE,
	pointType_BORDER,
	pointType_CORE
};

class point {
public:
	float x;
	float y;
	int cluster;
	int pointType;  // 1 noise 2 border 3 core
	int pts;        // points quantity around one point
	int corePointID;
	vector<int> corepts;
	int  visited;
	void init();
	point();
	point(float a, float b, int c=0) {
		x = a;
		y = b;
		cluster = c;
		pointType = pointType_UNDO;
		pts = 0;
		visited = 0;
		corePointID = -1;
	};
};

float squareDistance(point a, point b);
vector< pair<float, float> > DBSCAN(vector< pair<float, float> > positions, float Eps, int MinPts);

}

#endif