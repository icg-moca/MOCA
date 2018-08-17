#pragma once
#ifndef _MOCA_DEFORMATION_GRAPH_H
#define _MOCA_DEFORMATION_GRAPH_H
#include <iostream>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <flann\flann.hpp>

namespace Moca{
	const int VERTEX_DIM = 4;

	// uniform sampling
	// in: points , out: nodes
	void GenNodes(const std::vector<float> &points, std::vector<float> &nodes, const float radius) {
		const int VERTEX_DIM = 4;
		const int numPoints = points.size() / VERTEX_DIM;

		// Compute point cloud spatial bound
		Eigen::Vector3f bbLower(FLT_MAX, FLT_MAX, FLT_MAX);
		Eigen::Vector3f bbUpper(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (int i = 0; i < numPoints; ++i)
		{
			bbLower(0) = min(bbLower(0), points[VERTEX_DIM * i]);
			bbLower(1) = min(bbLower(1), points[VERTEX_DIM * i + 1]);
			bbLower(2) = min(bbLower(2), points[VERTEX_DIM * i + 2]);
			bbUpper(0) = max(bbUpper(0), points[VERTEX_DIM * i]);
			bbUpper(1) = max(bbUpper(1), points[VERTEX_DIM * i + 1]);
			bbUpper(2) = max(bbUpper(2), points[VERTEX_DIM * i + 2]);
		}
		Eigen::Vector3f extent = bbUpper - bbLower;
		float searchRad = max(extent.x(), max(extent.y(), extent.z())) * radius; // radius here is ratio
		printf("search radius is : %f\n", searchRad);

		Eigen::Vector4f leafSize = Eigen::Vector4f(searchRad, searchRad, searchRad, 1);
		Eigen::Vector4f invLeafSize = Eigen::Array4f::Ones() / leafSize.array();

		// Compute the minimum and maximum bounding box values
		Eigen::Vector4i min_b, max_b;
		min_b[0] = static_cast<int> (floor(bbLower[0] * invLeafSize[0]));
		max_b[0] = static_cast<int> (floor(bbUpper[0] * invLeafSize[0]));
		min_b[1] = static_cast<int> (floor(bbLower[1] * invLeafSize[1]));
		max_b[1] = static_cast<int> (floor(bbUpper[1] * invLeafSize[1]));
		min_b[2] = static_cast<int> (floor(bbLower[2] * invLeafSize[2]));
		max_b[2] = static_cast<int> (floor(bbUpper[2] * invLeafSize[2]));

		// Compute the number of divisions needed along all axis
		Eigen::Vector4i div_b = max_b - min_b + Eigen::Vector4i::Ones();
		div_b[3] = 0;

		// stores the nD centroid in a leaf
		struct Leaf {
			Leaf() :idx(-1) {}
			int idx;
		};

		// 3d grid leaves
		std::unordered_map<size_t, Leaf> mLeaves;
		// setup division multiplier
		Eigen::Vector4i divb_mul = Eigen::Vector4i(1, div_b[0], div_b[0] * div_b[1], 0);

		// 1st pass: build set of leaves with point index closet to the leaf cube center
		for (int i = 0; i < numPoints; ++i) {
			Eigen::Vector4i ijk = Eigen::Vector4i::Zero();
			ijk[0] = static_cast<int>(floor(points[VERTEX_DIM * i] * invLeafSize[0]));
			ijk[1] = static_cast<int>(floor(points[VERTEX_DIM * i + 1] * invLeafSize[1]));
			ijk[2] = static_cast<int>(floor(points[VERTEX_DIM * i + 2] * invLeafSize[2]));

			// compute leaf idx
			int idx = (ijk - min_b).dot(divb_mul);
			Leaf& leaf = mLeaves[idx];
			if (leaf.idx == -1) {
				leaf.idx = i;
				continue;
			}

			Eigen::Vector3f p;
			p << points[VERTEX_DIM * i], points[VERTEX_DIM * i + 1], points[VERTEX_DIM * i + 2];
			Eigen::Vector3f pleaf;
			pleaf << points[VERTEX_DIM * leaf.idx], points[VERTEX_DIM * leaf.idx + 1], points[VERTEX_DIM * leaf.idx + 2];

			// check if this point is closer than the current record of leaf
			float diff_cur = (p - ijk.head<3>().cast<float>()).squaredNorm();
			float diff_prev = (pleaf - ijk.head<3>().cast<float>()).squaredNorm();
			if (diff_cur < diff_prev) {
				leaf.idx = i;
			}
		}

		// 2nd pass: go over all the leaves and copy data
		nodes.reserve(mLeaves.size() * VERTEX_DIM);
		for (std::unordered_map<size_t, Leaf>::const_iterator it = mLeaves.begin(); it != mLeaves.end(); ++it) {
			nodes.push_back(points[VERTEX_DIM * it->second.idx]);
			nodes.push_back(points[VERTEX_DIM * it->second.idx + 1]);
			nodes.push_back(points[VERTEX_DIM * it->second.idx + 2]);
			nodes.push_back(1.0f);
		}
		printf("[Uniform Sampling] :: total sampled on point cloud node location: %d\n", nodes.size() / VERTEX_DIM);
	}

	// load control points from tet file
	// in: tet file, out: nodes
	void LoadNodesFromTet(const char* filename, std::vector<float> &nodes) {	
		FILE *fp = fopen(filename, "r");
		if (!fp) {
			printf("can't open tet file... \n");
			return;
		}

		char tetflag[4];
		fread(tetflag, sizeof(char), 3, fp);

		int numOfNodes = 0;
		fscanf(fp, "%d ", &numOfNodes);
		int numOfTet = 0;
		fscanf(fp, "%d ", &numOfTet);

		printf("%s, numNode = %d, numTet = %d\n", tetflag, numOfNodes, numOfTet);

		const int POINT_DIM = 4;
		nodes.resize(numOfNodes * POINT_DIM, 0.0f);
		for (int i = 0; i < numOfNodes; ++i) {
			fscanf(fp, "%f ", &nodes[POINT_DIM * i]);
			fscanf(fp, "%f ", &nodes[POINT_DIM * i + 1]);
			fscanf(fp, "%f ", &nodes[POINT_DIM * i + 2]);
			nodes[POINT_DIM * i + 3] = 1.0f;
		}

		//for (int i = 0; i < numOfNodes; ++i) {
		//	printf("%f, %f, %f \n", nodes[4 * i], nodes[4 * i + 1], nodes[4 * i + 2]);
		//}

		// todo tet indices
		//int numIndices = numOfTet * 4;
		//std::vector<int> indices(numIndices, -1);
		//for (int i = 0; i < numIndices; ++i) {
		//	fscanf(fp, "%d ", &indices[i]);
		//}

		//for (int i = 0; i < numOfTet; ++i) {
		//	printf("%d, %d, %d, %d \n", indices[4 * i], indices[4 * i + 1], indices[4 * i + 2], indices[4 * i + 3]);
		//}

		fclose(fp);

	}

	// load point cloud from ply file with pack4 points
	void LoadPlyPointCloud(const char* filename, std::vector<float> &points) {
		PLYModelLoader loader;
		loader.LoadModel(filename);
		loader.CopyPointsWithPack4(points);
		loader.Clear();
	}

	// load point cloud from ply file with pack4 points + normals
	void LoadPlyPointCloud(const char* filename, std::vector<float> &points, std::vector<float> &normals) {
		PLYModelLoader loader;
		loader.LoadModel(filename);
		loader.CopyPointsWithPack4(points);
		loader.CopyNormalsWithPack4(normals);
		loader.Clear();
	}

	class Topology {
	public:
		int											m_num;						// num of topology point
		int											m_knn;						// num nearest neighbor
		float										m_maxDistance;				// max distance between skeleton
		std::vector<int>							m_index;					// vert -> node
		std::vector<float>							m_distance;					// vert -> node dist
		std::vector<float>							m_weight;					// vert -> node weight

	public:
		Topology() : m_num(-1), m_knn(-1), m_maxDistance(-1.0f) {

		}

		void Clear(void) {
			m_num = -1;
			m_knn = -1;
			m_index.clear();
			m_distance.clear();
			m_weight.clear();
		}
	};
	// Generate deformation graph topology
	void GenTopology(
		std::vector<float> &nodes, std::vector<float> &query,
		Topology &topology,
		int k
	) {
		topology.Clear();

		topology.m_num = static_cast<int>(query.size() / VERTEX_DIM);
		topology.m_knn = k;		

		topology.m_index.resize(query.size() * topology.m_knn, -1);
		topology.m_distance.resize(query.size() * topology.m_knn, -1.0f);

		// build kd tree (usually eigen is column major, flann is row major)
		flann::Matrix<float> flann_dataset(&nodes[0], nodes.size()/VERTEX_DIM, VERTEX_DIM);
		flann::Index< flann::L2<float> > flann_index(flann_dataset, flann::KDTreeIndexParams(1));
		flann_index.buildIndex();

		// build search buffer
		flann::Matrix<float> query_data(&query[0], topology.m_num, VERTEX_DIM);		// num query * dim			
		flann::Matrix<int>	 neighbors(&topology.m_index[0], topology.m_num, topology.m_knn);
		flann::Matrix<float>	 distances(&topology.m_distance[0], topology.m_num, topology.m_knn);
		// query by the nodes to find nearest k neighbor (k+1 for exclude itself)
		flann_index.knnSearch(query_data, neighbors, distances, topology.m_knn, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

		for (std::vector<float>::iterator it = topology.m_distance.begin(); it != topology.m_distance.end(); ++it) {
			if (topology.m_maxDistance < *it) {
				topology.m_maxDistance = *it;
			}
		}
	}

	// filter the invalid linked nodes
	void CheckNeighbors(Topology &topology, float threshold) {
		for (int i = 0; i < topology.m_distance.size(); ++i) {
			if (topology.m_distance[i] >= threshold || topology.m_distance[i] == 0.0f) {
				topology.m_index[i] = -1;
				topology.m_distance[i] = -1.0f;
			}
		}
	}

	// Compute topology link weight
	void CalcWeights(Topology &topology) {
		topology.m_weight.clear();
		topology.m_weight.resize(topology.m_distance.size(), 0.0f);
		
		for (int i = 0; i < topology.m_num; ++i) {
			float accum = 0.0f;
			std::vector<float> w(topology.m_knn, 0.0f);
			for (int j = 0; j < topology.m_knn; j++) {
				int id = topology.m_knn * i + j;
				if (topology.m_distance[id] < 0) {
					continue;
				}
				
				// compute weight
				w[j] = (1.0f - topology.m_distance[id]);
				w[j] *= w[j];
				accum += w[j];
			}

			// normalization
			for (int j = 0; j < topology.m_knn; ++j) {
				int id = topology.m_knn * i + j;
				if (topology.m_distance[id] < 0) {
					continue;
				}

				topology.m_weight[id] = accum / w[j];
			}	
		}
	}

	// Transform points based on current topology and transformation
	void TransformPoints(
		const std::vector<float> &nodes, const Topology &topology,
		const std::vector<float> &transforms,
		const std::vector<float> &src_p, const std::vector<float> &src_n,
		std::vector<float> &dst_p, std::vector<float> &dst_n
	) {
		dst_p.resize(src_p.size(), 0);
		dst_n.resize(src_n.size(), 0);
		for (int i = 0; i < src_p.size(); ++i) {
			// for each graph node that this vert connected
			for (int j = 0; j < topology.m_knn; ++j) {
				int idx = j + i * topology.m_knn; // retrieve the idx of topology buffer
				int nid = topology.m_index[idx];
				
				// retrieve weight term
				float w = topology.m_weight[idx];

				// composite transformation
				const int numParams = 12;
				Eigen::Matrix3f nodeR;
				nodeR << transforms[numParams * nid], transforms[numParams * nid + 1], transforms[numParams * nid + 2],
					transforms[numParams * nid + 3], transforms[numParams * nid + 4], transforms[numParams * nid + 5],
					transforms[numParams * nid + 6], transforms[numParams * nid + 7], transforms[numParams * nid + 8];
				Eigen::Vector3f nodeT;
				nodeT << transforms[numParams * nid + 9], transforms[numParams * nid + 10], transforms[numParams * nid + 11];

				int nodesBufferID = VERTEX_DIM * nid;
				Eigen::Vector3f node;
				node << nodes[nodesBufferID], nodes[nodesBufferID + 1], nodes[nodesBufferID + 2];

				int pointBufferID = VERTEX_DIM * i;
				Eigen::Vector3f oldP, oldN;
				oldP << src_p[pointBufferID], src_p[pointBufferID + 1], src_p[pointBufferID + 2];
				oldN << src_n[pointBufferID], src_n[pointBufferID + 1], src_n[pointBufferID + 2];
			
				// For position:  
				Eigen::Vector3f newV;
				newV = (nodeR * (oldP - node)) + node + nodeT;

				// For normal: (create inv transpose buffer?)
				Eigen::Matrix3f invtR = nodeR.inverse();
				invtR.transposeInPlace();
				Eigen::Vector3f newN;
				newN = invtR * oldN;

				// make contribution
				dst_p[pointBufferID] += w * newV(0);
				dst_p[pointBufferID + 1] += w * newV(1);
				dst_p[pointBufferID + 2] += w * newV(2);

				dst_n[pointBufferID] += w * newN(0);
				dst_n[pointBufferID + 1] += w * newN(1);
				dst_n[pointBufferID + 2] += w * newN(2);
			}
		}
	}

	// Compute RME
	float RME(const std::vector<float> &p0, const std::vector<float> &p1) {
		float rme = 0.0f;
		if (p0.size() != p1.size()) {
			printf("RME buffer size not equal ... \n");
			return rme;
		}
		int numElems = p0.size();
		for (int i = 0; i < numElems; ++i) {
			float diff = p0[i] - p1[i];
			rme += (diff * diff);
		}
		rme /= numElems;
		rme *= 4; //? dim is necessary?
		rme = sqrt(rme);
		return rme;
	}

	class DeformationGraph {
	public:
		DeformationGraph() {

		}

		void SetParamters( const float nodenodedist, const float vertnodedist, const float nodepointdist,
						   const int nodeNodeNN, const int pointNodeNN, const int nodePointNN) {
			m_nodeNodeDistThres = nodenodedist;
			m_pointNodeDistThres = vertnodedist;
			m_nodePointDistThres = nodepointdist;

			m_node_node.m_knn = nodeNodeNN;
			m_point_node.m_knn = pointNodeNN;
			m_node_point.m_knn = nodePointNN;
		}

		void SetBuffers(std::vector<float> &nodes, std::vector<float> &points) {
			m_nodeRef = nodes;
			m_pointRef = points;
		}

		void BuildGraph(void) {
			if (m_nodeRef.empty() || m_pointRef.empty()) {
				printf("Deformation Graph :: call SetBuffers() before build... \n");
			}

			GenTopology(m_nodeRef, m_nodeRef, m_node_node, m_node_node.m_knn);
			GenTopology(m_nodeRef, m_pointRef, m_point_node, m_point_node.m_knn);
			GenTopology(m_pointRef, m_nodeRef, m_node_point, m_node_point.m_knn);

			CheckNeighbors(m_node_node, m_nodeNodeDistThres);
			CheckNeighbors(m_point_node, m_pointNodeDistThres);
			CheckNeighbors(m_node_point, m_nodePointDistThres);

			CalcWeights(m_node_node);
		}

	public:
		// setting
		float m_nodeNodeDistThres;
		float m_pointNodeDistThres;
		float m_nodePointDistThres;

		// graph info
		std::vector<float>							m_nodeRef;
		Topology									m_node_node;				// node -> node links
		Topology									m_node_point;				// node -> point links

		std::vector<float>							m_pointRef;
		Topology									m_point_node;				// point -> node links

		std::vector<float>							m_transform;
	};
};

#endif // !_MOCA_DEFORMATION_GRAPH_H

