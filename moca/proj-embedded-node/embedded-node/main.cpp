#pragma once
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <iostream>

#include "ModelLoader.h"
#include "MocaDeformationGraph.h"
using namespace std;

Moca::DeformationGraph g_graph;


#define USE_TET_FILE 0

void main(int argc, char **argv) {
	// moca
	// test moca data
	std::vector<float> mocaPoints;
	std::vector<float> mocaNormals;
	Moca::LoadPlyPointCloud("../../sample-data/point_cloud_01.ply", mocaPoints);
	printf("num of points = %d\n", mocaPoints.size() / 4);

	std::vector<float> mocaNodes;
#if USE_TET_FILE
	Moca::LoadNodesFromTet("../../sample-data/moca_01.tet", mocaNodes);
#else
	Moca::GenNodes(mocaPoints, mocaNodes, 0.02f);
#endif 

	// build graph
	const int nodenodeNN = 7;	 // +1 for excluding itself later (actual valid neighbor is nodenodeNN - 1)
	const int pointnodeNN = 4;
	const int markernodeNN = 4;
	const float nndistThres = 0.1f;
	const float vndistThres = 0.1f;
	const float mndistThres = 0.1f;

	g_graph.SetParamters(nndistThres, vndistThres, mndistThres, nodenodeNN, pointnodeNN, markernodeNN);
	g_graph.SetBuffers(mocaNodes, mocaPoints);
	g_graph.BuildGraph();


	printf(" finish build graph ... \n");

}


