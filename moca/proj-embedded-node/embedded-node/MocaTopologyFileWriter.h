#pragma once
#ifndef _MOCA_TOPOLOGY_FILE_H
#define _MOCA_TOPOLOGY_FILE_H
#include <vector>
#include <fstream>
#include <iostream>
#include "MocaDeformationGraph.h"


namespace Moca {
	class TopologyFileIO {
	public:
		enum FileType {
			ASCII,
			BINARY
		};

		TopologyFileIO() {

		}

		void SaveDeformationGraphToFile(const char* filename, DeformationGraph &graph, FileType type);
		void LoadDeformationGrpahFromFile(const char* filename, DeformationGraph &graph, FileType type);

	private:
		// write numElement, then buffer;  buffer.size() = stride * numElement
		void SavePointDataBuffer(std::ofstream &os, int numElement , const std::vector<float> &buffer, const int stride) {
			os << numElement << std::endl;
			for (int i = 0; i < numElement; ++i) {
				os << buffer[stride * i] << " " << buffer[stride * i + 1] << " " << buffer[stride * i + 2] << std::endl;
			}
		}

		// write numElement, then buffer;  buffer.size() = stride * numElement
		void SaveIndexDataBuffer(std::ofstream &os, int numElement, const int knn, const std::vector<int> &buffer) {
			os << numElement << std::endl;
			for (int i = 0; i < numElement; ++i) {
				for (int j = 0; j < knn; ++j) {
					os << buffer[knn * i + j] << " ";
				}
				os << std::endl;
			}
		}

		// write weight only, no numElements
		void SaveWeightDataBuffer(std::ofstream &os, int numElement, const int knn, const std::vector<float> &buffer) {
			for (int i = 0; i < numElement; ++i) {
				for (int j = 0; j < knn; ++j) {
					os << buffer[knn * i + j] << " ";
				}
				os << std::endl;
			}
		}

		void LoadPointDataBuffer(std::ifstream &is, int &numElement, std::vector<float> &buffer, const int stride) {
			is >> numElement;
			buffer.clear();
			buffer.resize(numElement * stride);
			for (int i = 0; i < numElement; ++i) {
				is >> buffer[stride * i] >> buffer[stride * i + 1] >> buffer[stride * i + 2];
			}
		}

		// write numElement, then buffer;  buffer.size() = stride * numElement
		void LoadIndexDataBuffer(std::ifstream &is, int &numElement, const int knn, std::vector<int> &buffer) {
			is >> numElement;
			buffer.clear();
			buffer.resize(numElement * knn, -1);
			for (int i = 0; i < numElement; ++i) {
				for (int j = 0; j < knn; ++j) {
					is >> buffer[knn * i + j];
				}
			}
		}

		void LoadWeightDataBuffer(std::ifstream &is, int &numElement, const int knn, std::vector<float> &buffer) {
			//os << numElement << std::endl;
			//for (int i = 0; i < numElement; ++i) {
			//	for (int j = 0; j < knn; ++j) {
			//		os << buffer[knn * i + j] << " ";
			//	}
			//	os << std::endl;
			//}
		}

		void SaveTopology(std::ofstream &os, Topology &topology) {
			// save knn
			os << topology.m_knn << std::endl;
			// save num + index
			SaveIndexDataBuffer(os, topology.m_num, topology.m_knn, topology.m_index);
			// save weights
			//SaveWeightDataBuffer(os, topology.m_num, topology.m_knn, topology.m_weight);
		}


		void LoadTopology(std::ifstream &is, Topology &topology) {
			// load knn
			is >> topology.m_knn;
			// load num + index
			LoadIndexDataBuffer(is, topology.m_num, topology.m_knn, topology.m_index);
			// load weight
		}

	};

	inline void Moca::TopologyFileIO::SaveDeformationGraphToFile(const char* filename, DeformationGraph &graph, FileType type) {
		std::ofstream os;
		if (type == ASCII) {
			os = std::ofstream(filename, std::ios::out);
		}
		else if (type == BINARY) {
			os = std::ofstream(filename, std::ios::out | std::ios::binary);
		}
		else {
			printf("invalid output data type...\n");
			return;
		}
		if (!os.is_open()) {
			printf("failed to create file to write...\n");
		}

		// save graph node
		const int vertDim = 4; // VERTEX_DIM
		SavePointDataBuffer(os, graph.m_node_node.m_num, graph.m_nodeRef, vertDim);
		// save marker_marker
		SaveTopology(os, graph.m_marker_marker);
		// save marker_node
		SaveTopology(os, graph.m_marker_node);
		// save node_node
		SaveTopology(os, graph.m_node_node);
		// save node_point
		SaveTopology(os, graph.m_node_point);
		// save point_node
		SaveTopology(os, graph.m_point_node);

		os.clear();
		os.close();
	}

	inline void TopologyFileIO::LoadDeformationGrpahFromFile(const char * filename, DeformationGraph & graph, FileType type) {
		std::ifstream is;
		if (type == ASCII) {
			is = std::ifstream(filename, std::ios::in);
		}
		else if (type == BINARY) {
			is = std::ifstream(filename, std::ios::in | std::ios::binary);
		}
		else {
			printf("invalid input file type...\n");
			return;
		}
		if (!is.is_open()) {
			printf("failed to open file to read...\n");
		}

		// load graph node
		const int vertDim = 4;
		int numGraphNode = 0;
		LoadPointDataBuffer(is, numGraphNode, graph.m_nodeRef, vertDim);
		
		// load marker_marker
		LoadTopology(is, graph.m_marker_marker);
		// load marker_node
		LoadTopology(is, graph.m_marker_node);
		// load node_node
		LoadTopology(is, graph.m_node_node);
		// load node_point
		LoadTopology(is, graph.m_node_point);
		// load point_node
		LoadTopology(is, graph.m_point_node);


		is.clear();
		is.close();

		if (!graph.m_nodeRef.empty() && 0) {
			for (int i = 0; i < numGraphNode; ++i) {
				printf("node %d: ( %f, %f, %f )\n", i, graph.m_nodeRef[4 * i], graph.m_nodeRef[4 * i + 1], graph.m_nodeRef[4 * i + 2]);
			}
		}
		if (!graph.m_marker_marker.m_index.empty() && 0) {
			for (int i = 0; i < graph.m_marker_marker.m_num; ++i) {
				printf("m_index of %d: ", i);
				for (int j = 0; j < graph.m_marker_marker.m_knn; ++j) {
					printf(" %d,", graph.m_marker_marker.m_index[graph.m_marker_marker.m_knn * i + j]);
				}
				printf("\n");
			}
		}
	}
}

#endif // !_MOCA_TOPOLOGY_FILE_H
