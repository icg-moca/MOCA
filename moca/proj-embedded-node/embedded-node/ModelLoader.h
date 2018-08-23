#pragma once
#ifndef _MODEL_LOADER_H
#define _MODEL_LOADER_H
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <Eigen\Core>
#include <Eigen/Geometry>
#include <Eigen\Dense>
using namespace std;

class PLYModelLoader {
public:
	std::vector<float> points;
	std::vector<float> normals;
	std::vector<int> colors;
	std::vector<int> indices;
	int totalVert;
	int totalFace;
	
public:
	PLYModelLoader():totalFace(0), totalVert(0){}

	~PLYModelLoader() {
		Clear();
	}

	void Clear(void) {
		totalVert = -1;
		totalFace = -1;
		points.clear();
		normals.clear();
		colors.clear();
		indices.clear();
	}

	void LoadModel(const char* filepath) {
		ifstream file;
		file.open(filepath, ios::binary);
		if (!file.is_open()) {
			cerr << "Couldn't open ply file < " << filepath << "> ! \n";
			return;
		}
		std::string line;
		getline(file, line);
		while (line.compare("vertex") != 0) {
			getline(file, line, ' ');
			cout << line << "\n";
		}
		file >> totalVert; // get total vert num
		cout << "model total vert: " << totalVert << "\n";

		// check if it is mesh
		bool hasNormal = false; bool hasColor = false;
		bool isEndHeader = false; bool isMesh = false;
		while (line.compare("face") != 0 && !isEndHeader) {
			getline(file, line);
			cout << line << endl;
			if (line.find("red") != std::string::npos) { hasColor = true; }
			if (line.find("nx") != std::string::npos) { hasNormal = true; }
			if (line.find("end_header") != std::string::npos) { isEndHeader = true; }
		}
		if (!isEndHeader) {
			file >> totalFace; // get total face num
			cout << "total face: " << totalFace << "\n";
			if (totalFace) isMesh = true;
			// read other information until end_header
			while (line.compare("end_header") != 0) {
				getline(file, line);
				cout << line << endl;
			}
		}

		points.reserve(totalVert * 3);
		normals.reserve(totalVert * 3);
		// Read p, n
		if (!hasColor && hasNormal) {
			float v = 0.0f;
			for (int i = 0; i < totalVert; i++) {
				//*
				getline(file, line);
				std::stringstream ss(line);
				float v;
				ss >> v;				points.push_back(v);
				//cout << v << " ";
				ss >> v;				points.push_back(v);
				//cout << v << " ";
				ss >> v;				points.push_back(v);
				//cout << v << " ";

				ss >> v;				normals.push_back(v);
				//cout << v << " ";
				ss >> v;				normals.push_back(v);
				//cout << v << " ";
				ss >> v;				normals.push_back(v);
				//cout << v << " " << "\n";		
				//*/
			}
		}
		if (hasColor && hasNormal) {
			float v = 0.0f;
			for (int i = 0; i < totalVert; i++) {
				//*
				getline(file, line);
				std::stringstream ss(line);
				float v;
				ss >> v;				points.push_back(v);
				//cout << v << " ";
				ss >> v;				points.push_back(v);
				//cout << v << " ";
				ss >> v;				points.push_back(v);
				//cout << v << " ";

				ss >> v;				normals.push_back(v);
				//cout << v << " ";
				ss >> v;				normals.push_back(v);
				//cout << v << " ";
				ss >> v;				normals.push_back(v);
				//cout << v << " " << "\n";		
				//*/
				// rgba
				int c;
				ss >> c;				colors.push_back(c);
				//cout << c << " ";
				ss >> c;				colors.push_back(c);
				//cout << c << " ";
				ss >> c;				colors.push_back(c);
				//cout << c << " ";
				ss >> c;				colors.push_back(c);
				//cout << c << " " << "\n";		
				//*/
			}
		}
		if (isMesh) {
			indices.reserve(totalFace * 3);
			for (int i = 0; i < totalFace; ++i) {
				int id;
				getline(file, line);
				std::stringstream ss(line);

				ss >> id;									// 3
				//cout << id << " ";
				ss >> id;	indices.push_back(id);			// tri_id0
				//cout << id << " ";
				ss >> id;	indices.push_back(id);			// tri_id1
				//cout << id << " ";
				ss >> id;	indices.push_back(id);			// tri_id2
				//cout << id << " " << "\n";
			}
		}

		file.close();
	}

	// copy to float buffer each stride is a vec4 
	void CopyPointsWithPack4(std::vector<float> &points) {
		points.clear();
		points.resize(4 * totalVert, 0.0f);
		for (int i = 0; i < totalVert; ++i) {
			points[4 * i] = this->points[3 * i];
			points[4 * i + 1] = this->points[3 * i + 1];
			points[4 * i + 2] = this->points[3 * i + 2];
			points[4 * i + 3] = 1.0f;
		}
	}

	// copy to float buffer each stride is a vec4 
	void CopyNormalsWithPack4(std::vector<float> &normals) {
		normals.clear();
		normals.resize(4 * totalVert, 0.0f);
		for (int i = 0; i < totalVert; ++i) {
			normals[4 * i] = this->normals[3 * i];
			normals[4 * i + 1] = this->normals[3 * i + 1];
			normals[4 * i + 2] = this->normals[3 * i + 2];
			normals[4 * i + 3] = 0.0f;
		}
	}
};
#endif