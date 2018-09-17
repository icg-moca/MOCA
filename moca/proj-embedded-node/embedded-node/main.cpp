#pragma once
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <iostream>

#include "ModelLoader.h"
#include "MocaDeformationGraph.h"
#include "MocaTopologyFileWriter.h"

using namespace std;

Moca::DeformationGraph g_graph;


#define USE_TET_FILE 0
#define LOAD_GRAPH 1

void main(int argc, char **argv) {
	// moca
	// test moca data
	const int POINT_DIM = 4;
	std::vector<float> mocaPoints;
	std::vector<float> mocaNormals;
	Moca::LoadPlyPointCloud("../../sample-data/volume_shoes_mesh_clamp_fillhole.ply", mocaPoints);
	printf("num of points = %d\n", mocaPoints.size() / POINT_DIM);

	std::vector<float> mocaNodes;
#if USE_TET_FILE
	Moca::LoadNodesFromTet("../../sample-data/moca_01.tet", mocaNodes);
#else
	Moca::GenNodes(mocaPoints, mocaNodes, 0.02f);
#endif 


#if LOAD_GRAPH 
	// start build deformation graph
	{
		// build graph
		const int nodenodeNN = 7;	 // +1 for excluding itself later (actual valid neighbor is 5)
		const int pointnodeNN = 5;
		const int nodepointNN = 7;
		const int markernodeNN = 8;
		const int markermarkerNN = 9;
		const float nndistThres = 0.1f;
		const float vndistThres = 0.03f;
		const float nvdistThres = 0.03f;
		const float mndistThres = 0.1f;
		const float mmdistThres = 38; // num of voxel
		g_graph.SetParamters(nndistThres, vndistThres, nvdistThres, mndistThres, mmdistThres, nodenodeNN, pointnodeNN, nodepointNN, markernodeNN, markermarkerNN);
		g_graph.SetBuffers(g_graph.m_nodeRef, mocaPoints);

		Moca::TopologyFileIO loader;
		loader.LoadDeformationGrpahFromFile("testTop.txt", g_graph, Moca::TopologyFileIO::ASCII);
		printf("done loading .. \n");

		g_graph.Print();
	}
#else

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
	
#endif
	printf("press any key to exit... \n");
	getchar();
}


