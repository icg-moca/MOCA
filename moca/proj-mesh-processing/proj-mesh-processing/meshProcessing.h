#pragma once
#include <vector>
#include <iostream>
#include <map>
#include "Eigen\Dense"

void CotagentLaplacianSmooth(std::vector<Eigen::Vector3f> &result,
	const std::vector<Eigen::Vector3f> &verts, const std::vector<int> &indices) {
	if (indices.empty()) return;

	int numTri = indices.size() / 3;
	std::vector<std::vector<int>>			triangleList(verts.size());   // each vertex in
	for (int i = 0; i < indices.size(); i += 3) {
		int ti = indices[i];
		int tj = indices[i + 1];
		int tk = indices[i + 2];

		triangleList[ti].push_back(tj);
		triangleList[ti].push_back(tk);
		triangleList[tj].push_back(ti);
		triangleList[tj].push_back(tk);
		triangleList[tk].push_back(ti);
		triangleList[tk].push_back(tj);
	}

	struct NeighborEdge {
		int j;
		std::vector<int> sides;
		float cotw;
		NeighborEdge() : j(-1), cotw(0.0f) {}
		NeighborEdge(int j) : cotw(0.0f) {
			this->j = j;
		}
		void Print(void) {
			printf("j: %d, ", j);
			for (int i = 0; i < sides.size(); ++i) {std::cout << sides[i] << ", ";}
			std::cout << "\n";
		}
		void Print(int sid) {
			printf("i: %d => j: %d  :: side: ", sid, j);
			for (int i = 0; i < sides.size(); ++i) {std::cout << sides[i] << ", ";}
			std::cout << "\n";
		}
	};

	// store neighbor index and two side vert index
	std::vector<std::vector<NeighborEdge>>  neighborInfo(verts.size());
	typedef std::multimap<int, int>::iterator  sideIter;
	typedef std::pair<int, int>  sidePair;
	typedef std::pair< std::multimap<int, int>::iterator, std::multimap<int, int>::iterator> rangeIter;
	for (int i = 0; i < triangleList.size(); ++i) {
		std::multimap<int, int> sideverts;
		std::vector<int> uniqueKeys; uniqueKeys.reserve(20);
		for (int j = 0; j < triangleList[i].size(); j += 2) {
			int si = triangleList[i][j];
			int sj = triangleList[i][j + 1];
			if (sideverts.find(si) == sideverts.end()) {
				// not find this pair, insert
				sideverts.insert(sidePair(si, sj));
				uniqueKeys.push_back(si);
			}
			else {
				rangeIter range = sideverts.equal_range(si);
				int duplicate = 0;
				for (sideIter it = range.first; it != range.second; ++it) {
					if (it->second == sj) {
						duplicate = 1;
						break;
					}
				}
				if (!duplicate) {
					sideverts.insert(sidePair(si, sj));
				}
			}

			if (sideverts.find(sj) == sideverts.end()) {
				// not find this pair, insert
				sideverts.insert(sidePair(sj, si));
				uniqueKeys.push_back(sj);
			}
			else {
				rangeIter range = sideverts.equal_range(sj);
				int duplicate = 0;
				for (sideIter it = range.first; it != range.second; ++it) {
					if (it->second == si) {
						duplicate = 1;
						break;
					}
				}
				if (!duplicate) {
					sideverts.insert(sidePair(sj, si));
				}
			}
		}// end sideverts

		neighborInfo[i].reserve(triangleList[i].size());  //this size should be twice of neighbor verts for close mesh
		for (int j = 0; j < uniqueKeys.size(); ++j) {
			NeighborEdge nedge;
			int vjid = uniqueKeys[j];	nedge.j = vjid;
			rangeIter range = sideverts.equal_range(vjid);
			int count = 0;
			for (sideIter it = range.first; it != range.second; ++it) {
				nedge.sides.push_back(it->second);
				count++;
			}
			neighborInfo[i].push_back(nedge);
		} // end build side verts
	}

	result.clear();
	result.resize(verts.size());

	const float eps = 1e-6f;
	const float cotan_max = cos(eps) / sin(eps);
	// for each vert compute cotagent weight and blend
	for (int i = 0; i < neighborInfo.size(); ++i) {
		for (int nid = 0; nid < neighborInfo[i].size(); ++nid) {
			NeighborEdge edge = neighborInfo[i][nid];
			//edge.Print(i);
			int jid = edge.j;
			Eigen::Vector3f eij = verts[jid] - verts[i];
			Eigen::Vector3f e0i = verts[i] - verts[edge.sides[0]];
			Eigen::Vector3f e0j = verts[jid] - verts[edge.sides[0]];

			float le0 = eij.norm();
			float le1 = e0i.norm();
			float le2 = e0j.norm();
			float l2e0 = eij.squaredNorm();
			float l2e1 = e0i.squaredNorm();
			float l2e2 = e0j.squaredNorm();

			float dblA0 = 2.0f * 0.25f * sqrtf(
				(le0 + (le1 + le2)) * (le2 - (le0 - le1)) * (le2 + (le0 - le1)) * (le0 + (le1 - le2))
			);
			if (dblA0 != dblA0){ dblA0 = 0.0f; }
			float cot0 = 0.0f;
			if (dblA0 > 1e-6f) cot0 = (l2e1 + l2e2 - l2e0) / dblA0 / 4.0f;

			float cot1 = 0.0f;
			if (edge.sides.size() > 1) {
				Eigen::Vector3f e1i = verts[i] - verts[edge.sides[1]];
				Eigen::Vector3f e1j = verts[jid] - verts[edge.sides[1]];

				float le3 = e1i.norm();
				float le4 = e1j.norm();
				float l2e3 = e1i.squaredNorm();
				float l2e4 = e1j.squaredNorm();

				float dblA1 = 2.0f * 0.25f * sqrtf(
					(le0 + (le3 + le4)) * (le4 - (le0 - le3)) * (le4 + (le0 - le3)) * (le0 + (le3 - le4))
				);
				if (dblA1 != dblA1){ dblA1 = 0.0f; }
				if (dblA1 > 1e-6f) cot1 = (l2e3 + l2e4 - l2e0) / dblA1 / 4.0f;
			}

			float cot = (cot0 + cot1) / 2;
			if (cot > cotan_max) {
				cot = cotan_max;
			}

			if (cot != cot) {
				cot = 0.0f;
			}
			//printf("s0: cot = %f, cot alec = %f     s1: cot = %f, cot alec %f,  final cot = %f\n ",
			//			cots0, cotss0, cots1, cotss1, cot);
			//printf("s0: cos = %f, cot = %f     s1: cos = %f, cot %f,  final cot = %f\n ",
			//	cos0, cot0, cos1, cot1, cot);
			//printf("cot : %f\n", cot);
			neighborInfo[i][nid].cotw = cot;
		}

		Eigen::Vector3f p = Eigen::Vector3f::Zero();
		float sumw = 0.0f;
		for (int j = 0; j < neighborInfo[i].size(); ++j) {
			float w = neighborInfo[i][j].cotw;
			p += (verts[neighborInfo[i][j].j] - verts[i]) * w;
			sumw += w;
		}
		float smooth = 0.5f;
		if (sumw < 1e-6f) sumw = 1.0f;	// avoid divide zero and stop smoothing ( first boundary condition )
		result[i] = (p / sumw) * smooth + verts[i];
	}
}


void Mat3PolarDecomposition(Eigen::Matrix3f &s, Eigen::Matrix3f &q, const Eigen::Matrix3f &m) {
	s.setZero();
	q.setZero();

	Eigen::Matrix3f mtm = m.transpose() * m;
	Eigen::Matrix3f evecs; evecs.setZero();
	Eigen::VectorXf evals; evals.setZero();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(mtm);
	if (eigensolver.info() != Eigen::Success) abort();
	evals = eigensolver.eigenvalues();		 // sorted in increasing order
	evecs = eigensolver.eigenvectors();		 // eigenvectors are normalized and stored as columns

	Eigen::Matrix3f E;
	E.setZero();
	E(0, 0) = sqrt(evals(0));
	E(1, 1) = sqrt(evals(1));
	E(2, 2) = sqrt(evals(2));

	s = evecs * E * evecs.inverse();		// scale + shear
	q = m * s.inverse();					// rot
}