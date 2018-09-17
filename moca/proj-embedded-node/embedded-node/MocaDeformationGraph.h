#pragma once
#ifndef _MOCA_DEFORMATION_GRAPH_H
#define _MOCA_DEFORMATION_GRAPH_H
#include <iostream>
#include <vector>
#include <unordered_map>
#include <set>
#include <Eigen/Dense>
#include <flann\flann.hpp>


namespace Moca {
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

	template< class T >
	static void UniformSampling(std::vector<float> &samples, const int pdim, const std::vector<T>& volume, const int vres,
		const Eigen::Matrix4f &vol2mesh, float spacing = 0.0f) {
		samples.clear();
		samples.reserve(vres*vres*vres);
		if (spacing <= 0.0f) {
			spacing = 1.0f;
		}
		// tweak spacing to avoid edge cases for particles laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		float spacingEps = spacing * (1.0f - 1e-4f);

		if (pdim > 4) {
			printf("3D point dim max comp is 4...\n");
			return;
		}

		// scale to normalized space
		float unitVoxel = vol2mesh(0, 0) / vres;

		Eigen::Vector3f meshLower;
		meshLower << vol2mesh(0, 3), vol2mesh(1, 3), vol2mesh(2, 3);

		// sample interior
		for (int x = 0; x < vres; x += spacingEps)
		{
			for (int y = 0; y < vres; y += spacingEps)
			{
				for (int z = 0; z < vres; z += spacingEps)
				{
					const int index = z * vres*vres + y * vres + x;
					if (volume[index])
					{
						// volume coordinate space
						Eigen::Vector3f v = meshLower;
						v += unitVoxel * Eigen::Vector3f(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f);

						Eigen::Vector4f p;
						p << v(0), v(1), v(2), 1.0f;

						for (int i = 0; i < pdim; ++i) {
							samples.push_back(p(i));
						}
					}
				}
			}
		}
	}

	// load control points from tet file
	// in: tet file, out: nodes
	void LoadNodesFromTet(const char* filename, std::vector<float> &nodes, std::vector<int> &indices) {
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
		int numIndices = numOfTet * 4;
		indices.resize(numIndices, -1);
		for (int i = 0; i < numIndices; ++i) {
			fscanf(fp, "%d ", &indices[i]);
		}

		//for (int i = 0; i < numOfTet; ++i) {
		//	printf("%d, %d, %d, %d \n", indices[4 * i], indices[4 * i + 1], indices[4 * i + 2], indices[4 * i + 3]);
		//}

		fclose(fp);

	}

	// load point cloud from ply file
	void LoadPlyPointCloud(const char* filename, std::vector<float> &points) {
		PLYModelLoader loader;
		loader.LoadModel(filename);
		loader.CopyPointsWithPack4(points);
		loader.Clear();
	}

	// load point cloud from ply file
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

		struct TopologyPoint {
			int m_idx;
			float m_dist;

			TopologyPoint() :m_idx(-1), m_dist(1000000.0f) {}
			TopologyPoint(int idx, float dist) :m_idx(idx), m_dist(dist) {}
		};
		struct TopologyPointCompare_ {
		public:
			bool operator() (const TopologyPoint &a, const TopologyPoint &b) const {
				return a.m_dist > b.m_dist;
			}
		};


		void Print(void) {
			printf("num of points in top: %d \n", this->m_num);
			printf("knn of top: %d \n", this->m_knn);
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
		flann::Matrix<float> flann_dataset(&nodes[0], nodes.size() / VERTEX_DIM, VERTEX_DIM);
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

	// Generate Topology from mesh connectivity 
	void GenTopology(std::vector<float> &points, const int pdim, std::vector<int> &indices, const bool isMesh, Topology &topology) {
		int numPoints = points.size() / pdim;

		int vertPerFace = 0;
		if (isMesh) {
			vertPerFace = 3;
		}
		else {
			vertPerFace = 4;
		}


		// check points number match indices max indice
		//int maxindice = -1;
		//for (int i = 0; i < indices.size(); ++i) {
		//	if (maxindice < indices[i]) {
		//		maxindice = indices[i];
		//	}
		//}
		//printf("max idx = %d, numpoints = %d \n", maxindice, numPoints);

		typedef std::unordered_multimap<int, int>::const_iterator umit;
		std::unordered_multimap<int, int>  connectivity;

		int numFaces = indices.size() / vertPerFace;
		for (int i = 0; i < numFaces; ++i) {
			for (int j = 0; j < vertPerFace; ++j) {
				int idx = vertPerFace * i + j;
				int vid = indices[idx];

				// if this is first time meet this vertice
				for (int jj = 0; jj < vertPerFace; ++jj) {
					int vnidx = vertPerFace * i + jj;		// be careful...
					int vnid = indices[vnidx];
					if (vnid == vid) {
						continue;
					}
					// check whether this will be duplicated entry			!!!!!!!!!!!!!!!!
					std::pair <std::unordered_multimap<int, int>::iterator, std::unordered_multimap<int, int>::iterator> ret;
					ret = connectivity.equal_range(vid);
					bool duplicate = false;
					for (umit it = ret.first; it != ret.second; ++it) {
						if (it->second == vnid) {
							duplicate = true;
							break;
						}
					}
					if (!duplicate) {
						connectivity.insert(std::pair<int, int>(vid, vnid));
					}
				}
			}
		}

		// extract all the points that connect to this vertex
		int maxNeighbor = 0;
		int maxIdx = -1;
		for (int i = 0; i < numPoints; ++i) {
			int numEntry = connectivity.count(i);
			if (maxNeighbor < numEntry) {
				maxNeighbor = numEntry;
				maxIdx = i;
			}
		}
		//printf("max neighbor = %d\n", maxNeighbor);

		topology.m_num = numPoints;
		topology.m_knn = maxNeighbor;
		topology.m_index.resize(topology.m_knn * numPoints, -1);
		topology.m_distance.resize(topology.m_knn * numPoints, 0.0f);

		std::vector<int> countForPoints(numPoints, 0);
		for (umit it = connectivity.begin(); it != connectivity.end(); ++it) {
			int srcPid = it->first;
			int curCount = countForPoints[srcPid];
			int dstPid = it->second;
			topology.m_index[topology.m_knn * srcPid + curCount] = dstPid;
			Eigen::Vector3f srcp, dstp;
			srcp << points[pdim * srcPid], points[pdim * srcPid + 1], points[pdim * srcPid + 2];
			dstp << points[pdim * dstPid], points[pdim * dstPid + 1], points[pdim * dstPid + 2];
			topology.m_distance[topology.m_knn * srcPid + curCount] = (dstp - srcp).norm();
			countForPoints[srcPid]++;
		}

		// examine max 
		//for (int i = 0; i < maxNeighbor; ++i) {
		//	int id = topology.m_knn * maxIdx + i;
		//	printf("index %d, dist %f \n", topology.m_index[id], topology.m_distance[id]);
		//}
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

		void SetParamters(const float nodenodedist, const float vertnodedist, const float nodepointdist,
			const int nodeNodeNN, const int pointNodeNN, const int nodePointNN) {
			m_nodeNodeDistThres = nodenodedist;
			m_pointNodeDistThres = vertnodedist;
			m_nodePointDistThres = nodepointdist;

			m_node_node.m_knn = nodeNodeNN;
			m_point_node.m_knn = pointNodeNN;
			m_node_point.m_knn = nodePointNN;
		}

		void SetParamters(const float nodenodedist, const float vertnodedist, const float nodepointdist, const float markernodedist, const float markermarkerdist,
			const int nodeNodeNN, const int pointNodeNN, const int nodePointNN, const int markerNodeNN, const int markermarkerNN) {
			m_nodeNodeDistThres = nodenodedist;
			m_pointNodeDistThres = vertnodedist;
			m_nodePointDistThres = nodepointdist;
			m_markerNodeDistThres = markernodedist;
			m_markerMarkerDistThres = markermarkerdist;

			m_node_node.m_knn = nodeNodeNN;
			m_point_node.m_knn = pointNodeNN;
			m_node_point.m_knn = nodePointNN;
			m_marker_node.m_knn = markerNodeNN;
			m_marker_marker.m_knn = markermarkerNN;
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
			CalcWeights(m_point_node);
			CalcWeights(m_node_point);
		}

		void BuildGraphWithMarker(void) {
			if (m_nodeRef.empty() || m_pointRef.empty() || m_canonicalMarkers.empty()) {
				printf("Deformation Graph :: call SetBuffers() before build... \n");
			}

			GenTopology(m_nodeRef, m_nodeRef, m_node_node, m_node_node.m_knn);
			GenTopology(m_nodeRef, m_pointRef, m_point_node, m_point_node.m_knn);
			GenTopology(m_pointRef, m_nodeRef, m_node_point, m_node_point.m_knn);
			GenTopology(m_nodeRef, m_canonicalMarkers, m_marker_node, m_marker_node.m_knn);

			CheckNeighbors(m_node_node, m_nodeNodeDistThres);
			CheckNeighbors(m_point_node, m_pointNodeDistThres);
			CheckNeighbors(m_node_point, m_nodePointDistThres);
			CheckNeighbors(m_marker_node, m_markerNodeDistThres);

			CalcWeights(m_marker_marker);
			CalcWeights(m_marker_node);
			CalcWeights(m_node_node);
			CalcWeights(m_node_point);
			CalcWeights(m_point_node);
		}

		void Print(void) {
			printf("\n");
			printf("marker_marker: m_num %d, m_knn %d, m_index size = %d, m_distance size = %d, m_weight size = %d \n",
				m_marker_marker.m_num, m_marker_marker.m_knn, m_marker_marker.m_index.size(), m_marker_marker.m_distance.size(), m_marker_marker.m_weight.size());
			printf("marker_node: m_num %d, m_knn %d, m_index size = %d, m_distance size = %d, m_weight size = %d \n",
				m_marker_node.m_num, m_marker_node.m_knn, m_marker_node.m_index.size(), m_marker_node.m_distance.size(), m_marker_node.m_weight.size());
			printf("node_node: m_num %d, m_knn %d, m_index size = %d, m_distance size = %d, m_weight size = %d \n",
				m_node_node.m_num, m_node_node.m_knn, m_node_node.m_index.size(), m_node_node.m_distance.size(), m_node_node.m_weight.size());
			printf("node_point: m_num %d, m_knn %d, m_index size = %d, m_distance size = %d, m_weight size = %d \n",
				m_node_point.m_num, m_node_point.m_knn, m_node_point.m_index.size(), m_node_point.m_distance.size(), m_node_point.m_weight.size());
			printf("point_node: m_num %d, m_knn %d, m_index size = %d, m_distance size = %d, m_weight size = %d \n",
				m_point_node.m_num, m_point_node.m_knn, m_point_node.m_index.size(), m_point_node.m_distance.size(), m_point_node.m_weight.size());
			printf("\n");
		}

	public:
		// setting
		float m_nodeNodeDistThres;
		float m_pointNodeDistThres;
		float m_nodePointDistThres;
		float m_markerNodeDistThres;
		float m_markerMarkerDistThres;

		// graph info
		std::vector<float>							m_canonicalMarkers;
		Topology									m_marker_marker;
		Topology									m_marker_node;

		std::vector<float>							m_nodeRef;
		Topology									m_node_node;				// node -> node links
		Topology									m_node_point;				// node -> point links

		std::vector<float>							m_pointRef;
		Topology									m_point_node;				// point -> node links

		std::vector<float>							m_transform;
	};


	void FindCloestNodeToMarker(Topology &marker_node, std::vector<float> &graph_nodes, const std::vector<Eigen::Vector4f> markers) {
		marker_node.Clear();

		std::vector<float> markerarr(4 * markers.size());
		for (int i = 0; i < markers.size(); ++i) {
			markerarr[4 * i] = markers[i].x();
			markerarr[4 * i + 1] = markers[i].y();
			markerarr[4 * i + 2] = markers[i].z();
			markerarr[4 * i + 3] = 1.0f;
		}
		GenTopology(graph_nodes, markerarr, marker_node, 1);
	}
	void FindCloestNodeToMarker(Topology &marker_node, std::vector<float> &graph_nodes, std::vector<float> markers) {
		marker_node.Clear();
		GenTopology(graph_nodes, markers, marker_node, 1);
		printf("max_distance = %f\n", marker_node.m_maxDistance);
	}

	// Dijkstra algorithm set method O(ElogV)
	// points = (point cloud)
	// topology = (point_point_topology)
	void ComputeGeodesicDistance(float &geodesic, std::vector<int> &parent, const Moca::Topology &topology, const int src, const int dst) {
		if (!topology.m_num) {
			return;
		}

		// distance and status
		int numPoints = topology.m_num;
		const float fardist = 10000.0f;
		std::vector<float> distance(numPoints, fardist);
		std::vector<int>   status(numPoints, 0);
		distance[src] = 0.0f;

		// path
		parent.clear();
		parent.resize(numPoints);
		parent[src] = -1;

		// dist, indx
		std::set<std::pair<float, int>>  setCandis;
		setCandis.insert(std::make_pair(distance[src], src));

		while (!setCandis.empty()) {
			// get the min dist point
			std::pair<float, int> top = *(setCandis.begin());
			setCandis.erase(setCandis.begin());
			int id = top.second;

			// 'i' is used to get all adjacent vertices of a vertex  ( start from 1 to skip itself )
			for (int i = 1; i < topology.m_knn; ++i) {
				int bidx = topology.m_knn * id + i; // buffer index
				int nid = topology.m_index[bidx];
				if (nid < 0) {
					continue;
				}
				float dist = topology.m_distance[bidx];

				//  If there is shorter path to neighbor through current id.
				if (distance[nid] > distance[id] + dist)
				{
					if (distance[nid] != fardist) {
						setCandis.erase(setCandis.find(std::make_pair(distance[nid], nid)));
					}

					// Updating distance of v
					distance[nid] = distance[id] + dist;
					if (nid == dst) {
						//printf("id = %d, nid = %d, prev dist nid = %f, dist id = %f, dist = %f \n", id, nid, distance[nid], distance[id], dist);
					}
					parent[nid] = id;
					setCandis.insert(std::make_pair(distance[nid], nid));
				}
			}
		}

		geodesic = distance[dst];
	}

	typedef Topology GeodesicTopology;
	/*
	Generate topology of marker based on the surface geometry or inner geometry only...
	*/
	void GenTopologyForMarkers(GeodesicTopology &marker_marker, std::vector<std::vector<int>> &paths,
		const Topology &marker_node, const Topology &node_node,
		const float maxDist, const int knn) {
		marker_marker.Clear();

		if (!marker_node.m_num || !node_node.m_num || marker_node.m_knn != 1) {
			return;
		}

		// set shortest debug buffer
		paths.resize(marker_node.m_num);

		// copy setting
		marker_marker.m_knn = knn;
		marker_marker.m_num = marker_node.m_num;

		// storing geodesic distances between each projected marker
		const float fardist = 1000000.0f;
		Eigen::MatrixXf adjacentDistMat;
		adjacentDistMat.setConstant(marker_node.m_num, marker_node.m_num, fardist);

		for (int i = 0; i < marker_node.m_num; ++i) {
			// index in node array
			int srcIdx = marker_node.m_index[i];
			// compute geodesic distance among all other projected marker locations
			for (int j = 0; j < marker_node.m_num; ++j) {
				int dstIdx = marker_node.m_index[j];
				if (srcIdx == dstIdx) {
					adjacentDistMat(i, j) = 0.0f; // diagonal
					continue;
				}
				float geodesic = -1.0f;
				Moca::ComputeGeodesicDistance(geodesic, paths[i], node_node, srcIdx, dstIdx);
				//printf("geodesic: %f\n", geodesic);
				if (geodesic < adjacentDistMat(i, j)) {
					adjacentDistMat(i, j) = geodesic;
				}
			}
		}

		// make marker_point index translator <point id, marker id>
		std::unordered_map<int, int> translator;
		for (int i = 0; i < marker_node.m_num; ++i) {
			translator.insert(std::make_pair(marker_node.m_index[i], i));
		}

		// construct topology from adjacent projected point dist matrix
		marker_marker.m_distance.resize(knn * marker_node.m_num, fardist);
		marker_marker.m_index.resize(knn * marker_node.m_num, -1);
		for (int i = 0; i < marker_marker.m_num; ++i) {
			// buffer for min heap to find points that increasing in distance
			std::vector<Topology::TopologyPoint> candidates(marker_marker.m_num);
			for (int j = 0; j < marker_node.m_num; ++j) {
				Topology::TopologyPoint p;
				p.m_idx = marker_node.m_index[j];
				if (p.m_idx < 0) continue;

				p.m_dist = adjacentDistMat(i, j);
				candidates[j] = p;
			}
			std::make_heap(candidates.begin(), candidates.end(), Topology::TopologyPointCompare_());

			// loop to find the most closest knn neighbor points, then translate to marker index
			marker_marker.m_maxDistance = 0.0f;
			for (int j = 0; j < marker_marker.m_knn; ++j) {
				std::pop_heap(candidates.begin(), candidates.end(), Topology::TopologyPointCompare_());
				Topology::TopologyPoint min = candidates.back();
				candidates.pop_back();

				if (min.m_dist > maxDist) {
					continue;
				}
				if (marker_marker.m_maxDistance < min.m_dist && min.m_dist != fardist) {
					marker_marker.m_maxDistance = min.m_dist;
				}

				int id = marker_marker.m_knn * i + j;
				marker_marker.m_index[id] = translator.find(min.m_idx)->second;
				marker_marker.m_distance[id] = min.m_dist;	// semi geodesic distance
			}
		}
	}

	/*
	Generate topology of markers from both inner and surface geometry
	*/
	void GenTopologyForMarkers(GeodesicTopology &marker_marker, std::vector<std::vector<int>> &paths,
		const Topology &marker_surface, const Topology &marker_inner, const Topology &surface_top, const Topology &inner_top,
		const float maxDist, const int knn) {
		GeodesicTopology marker_marker_surface;
		GeodesicTopology marker_marker_inner;

		std::vector<std::vector<int>> surface_path;
		std::vector<std::vector<int>> inner_path;

		// generate marker connectivity from two data sets
		GenTopologyForMarkers(marker_marker_surface, surface_path, marker_surface, surface_top, maxDist, knn);
		GenTopologyForMarkers(marker_marker_inner, inner_path, marker_inner, inner_top, maxDist, knn);
		if (marker_marker_surface.m_num != marker_marker_inner.m_num) {
			printf("inner marker number is not equal with surface marker...\n");
		}

		// compute final topology
		int numOfMarkers = marker_marker_surface.m_num;
		marker_marker.m_num = numOfMarkers;
		marker_marker.m_knn = knn;
		marker_marker.m_maxDistance = max(marker_marker_surface.m_maxDistance, marker_marker_inner.m_maxDistance);

		// compute final path
		paths.clear();
		paths.resize(marker_marker.m_num);

		marker_marker.m_distance.resize(marker_marker.m_knn * marker_marker.m_num, 1000000.0f);
		marker_marker.m_index.resize(marker_marker.m_knn * marker_marker.m_num, -1);
		//printf("markm %d, marker sur %d, marker inr %d\n", marker_marker.m_distance.size(), marker_marker_surface.m_distance.size(), marker_marker_inner.m_distance.size());
		for (int i = 0; i < marker_marker.m_num; ++i) {
			std::vector<float> dists(marker_marker.m_knn * 2, 1000000.0f);
			std::vector<int>   idxs(marker_marker.m_knn * 2, -1);
			for (int j = 0; j < marker_marker.m_knn; ++j) {
				int id = marker_marker.m_knn * i + j;
				dists[2 * j] = marker_marker_surface.m_distance[id];
				idxs[2 * j] = marker_marker_surface.m_index[id];
				dists[2 * j + 1] = marker_marker_inner.m_distance[id];
				idxs[2 * j + 1] = marker_marker_inner.m_index[id];
			}

			// sort O(n^2)		
			int ii, jj, min_idx;
			// One by one move boundary of unsorted subarray
			for (ii = 0; ii < dists.size() - 1; ii++) {
				// Find the minimum element in unsorted array
				min_idx = ii;
				for (jj = ii + 1; jj < dists.size(); jj++)
					if (dists[jj] < dists[min_idx])
						min_idx = jj;

				// Swap the found minimum element with the first element
				float tmpd = dists[min_idx];
				int tmpi = idxs[min_idx];
				dists[min_idx] = dists[ii];
				idxs[min_idx] = idxs[ii];
				dists[ii] = tmpd;
				idxs[ii] = tmpi;
			}

			int count = 0;
			for (int j = 0; j < dists.size(); ++j) {
				if (count == marker_marker.m_knn - 1) {
					break;
				}
				int id = marker_marker.m_knn * i + count;
				if (j > 0 && marker_marker.m_index[id - 1] == idxs[j]) {
					continue;
				}
				//if (idxs[j] == 0) {
				//	printf("marker: %d, with 0, distance %f \n", i, dists[j]);
				//}
				marker_marker.m_distance[id] = dists[j];
				marker_marker.m_index[id] = idxs[j];
				count++;
			}
			paths[i] = surface_path[i];

			// debug
			//for (int j = 0; j < knn; ++j) {
			//	int id = marker_marker.m_knn * i + j;
			//	marker_marker.m_distance[id] = marker_marker_surface.m_distance[id];
			//	marker_marker.m_index[id] = marker_marker_surface.m_index[id];
			//	marker_marker.m_distance[id] = marker_marker_inner.m_distance[id];
			//	marker_marker.m_index[id] = marker_marker_inner.m_index[id]; 
			//	printf("marker %d, link to %d, distance %f\n", i, marker_marker.m_index[id], marker_marker.m_distance[id]);
			//}
			//paths[i] = surface_path[i];
			//paths[i] = inner_path[i];

		}
	}
};

#endif // !_MOCA_DEFORMATION_GRAPH_H

