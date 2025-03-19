#include "DBSCAN.h"

int clusterID = 0;

namespace dbscan
{


float squareDistance(point a, point b) {
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

vector< pair<float, float> > DBSCAN(vector< pair<float, float> > positions, float Eps, int MinPts) {
	int len = positions.size();
	vector<point> dataset;

	for (int i = 0; i < positions.size(); i++)
	{
		point p(positions[i].first, positions[i].second);
		dataset.push_back(p);
	}
	vector<vector <float>> distP2P(len);
	//vector<vector <float>> distP2P(vector <float>(len));

	//calculate pts计算每个点在eps范围内有几个点
	// cout << "calculate pts" << endl;
	for (int i = 0; i < len; i++) {
		for (int j = 0; j < len; j++) {
			float distance = squareDistanceVect(dataset[i], dataset[j]);//squareDistanceVect squareDistance 
			distP2P[i].push_back(distance);//disp for debug
			if (distance <= Eps) {
				dataset[i].pts++;
			}
		}
	}
	//core point 核心点，pts大于minPts的时候，该点为核心点
	// cout << "core point " << endl;
	vector<point> corePoint;
	for (int i = 0; i < len; i++) {
		int tempPts = dataset[i].pts;
		if (tempPts >= MinPts) {
			dataset[i].pointType = pointType_CORE;
			dataset[i].corePointID = i;
			corePoint.push_back(dataset[i]);
		}
	}

	// cout << "joint core point" << endl;
	//joint core point
	int numCorePoint = corePoint.size(); //core point number核心点的数量
	for (int i = 0; i < numCorePoint; i++) {
		for (int j = 0; j < numCorePoint; j++) {
			float distTemp = distP2P[corePoint[i].corePointID][corePoint[j].corePointID];//display for debug  distTemp相当于二维数组，distTemp[i][j]即核心点i和j之间的距离
			if (distTemp <= Eps) {//squareDistance(corePoint[i],corePoint[j])
				corePoint[i].corepts.push_back(j);//other point orderID link to core point
			}//把每一个在核心点领域的核心点放到一起
		}
	}
	for (int i = 0; i < numCorePoint; i++) {//遍历所有的核心点
		stack<point*> ps;//临时存储核心点
		if (corePoint[i].visited == 1) continue;
		clusterID++;
		corePoint[i].cluster = clusterID;//create a new cluster
		ps.push(&corePoint[i]);
		point* v;
		while (!ps.empty()) {
			v = ps.top();
			v->visited = 1;
			ps.pop();
			for (int j = 0; j < v->corepts.size(); j++) {//最开始归类的一簇进行遍历
				if (corePoint[v->corepts[j]].visited == 1) continue;
				corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
				//dataset[v->corepts[j]].cluster= corePoint[i].cluster;
				corePoint[v->corepts[j]].visited = 1;
				ps.push(&corePoint[v->corepts[j]]);
			}
		}
	}

	// cout << "border point,joint border point to core point" << endl;
	//border point,joint border point to core point
	int k = 0;//k用来在dataset中统计是第几个核心点
	for (int i = 0; i < len; i++) {
		if (dataset[i].pointType == pointType_CORE)//如果该点是核心点，在上面已经访问过了，就不再访问，因为核心点不可能是边界点，没必要再访问一次
		{
			dataset[i].cluster = corePoint[k++].cluster;//遍历到第k个核心点时，把属于的簇id给原来的dataset
			continue;
		}
		for (int j = 0; j < numCorePoint; j++) {
			float distTemp = distP2P[i][corePoint[j].corePointID];
			if (distTemp <= Eps) {
				dataset[i].pointType = pointType_BORDER;
				dataset[i].cluster = corePoint[j].cluster;
				break;
			}
		}
	}
	// cout << "output" << endl;
	//output
	//display  
	//save in .txt format named clustering.txt
	// fstream clustering;//save .txt
	// clustering.open("clustering.txt", ios::out);//save .txt
	char dispInfo[20000];
	int dataDim = dataset[0].xn.size();//data dimension
	for (int i = 0; i < len; i++) {
		//%11.4lf,%11.4lf,%11.4lf,%11.4lfDBSCAN
		sprintf(dispInfo, "第%3d个数据：", i + 1);
		for (int j = 0; j < dataDim; j++DBSCAN) {
			char dataSrc[30];
			if (j == 0)
				sprintf(dataSrc, "%11.4lf", dataset[i].xn[j]);
			else
				sprintf(dataSrc, ",%11.4lf", dataset[i].xn[j]);
			strcat(dispInfo, dataSrc);
		}
		char dataClust[30];
		sprintf(dataClust, ",%4d", dataset[i].cluster);
		strcat(dispInfo, dataClust);
		char datasrc1[10];
		sprintf(datasrc1, "    %d\n", dataset[i].pointType);
		strcat(dispInfo, datasrc1);
		cout << dispInfo;      //display in cmd window
		// clustering << dispInfo;//save the results in .txt format named clustering.txt

	}
	// clustering.close();//save .txt

}

}