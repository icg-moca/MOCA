#pragma once
#ifndef _GL_RENDER_OBJECT_H
#define _GL_RENDER_OBJECT_H
#include <vector>
#include <string>
#include <iostream>


//=========================================================================
//		GL VR object class 
//=========================================================================
class GLVRobject {
	GLuint m_vbo;
	GLuint m_ibo;
	GLuint m_vao;
	GLuint m_tex;
	GLsizei m_vertCount;
	std::string m_modelName;

public:
	GLVRobject(const std::string &modelName) : m_modelName(modelName){
		m_ibo = 0;
		m_tex = 0;
		m_vao = 0;
		m_vbo = 0;
	}

	~GLVRobject() { Cleanup(); }

	bool InitBuffer(const vr::RenderModel_t &vrModel, const vr::RenderModel_TextureMap_t &vrDiffuseTex);
	void Cleanup();
	void Draw();
	const std::string &GetName() const { return m_modelName; }
};

bool GLVRobject::InitBuffer(const vr::RenderModel_t &vrModel, const vr::RenderModel_TextureMap_t &vrDiffuseTex) {
	// create and bind a VAO to hold state for this model
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	// Populate a vertex buffer
	glGenBuffers(1, &m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vr::RenderModel_Vertex_t) * vrModel.unVertexCount, vrModel.rVertexData, GL_STATIC_DRAW);

	// Identify the components in the vertex buffer
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t), (void *)offsetof(vr::RenderModel_Vertex_t, vPosition));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t), (void *)offsetof(vr::RenderModel_Vertex_t, vNormal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t), (void *)offsetof(vr::RenderModel_Vertex_t, rfTextureCoord));

	// Create and populate the index buffer
	glGenBuffers(1, &m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint16_t) * vrModel.unTriangleCount * 3, vrModel.rIndexData, GL_STATIC_DRAW);

	glBindVertexArray(0);

	// create and populate the texture
	glGenTextures(1, &m_tex);
	glBindTexture(GL_TEXTURE_2D, m_tex);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTex.unWidth, vrDiffuseTex.unHeight,
		0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTex.rubTextureMapData);

	// If this renders black ask McJohn what's wrong.
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

	glBindTexture(GL_TEXTURE_2D, 0);

	m_vertCount = vrModel.unTriangleCount * 3;

	return true;
}

void GLVRobject::Cleanup() {
	if (m_vbo)
	{
		glDeleteBuffers(1, &m_ibo);
		glDeleteVertexArrays(1, &m_vao);
		glDeleteBuffers(1, &m_vbo);
		m_ibo = 0;
		m_vao = 0;
		m_vbo = 0;
	}
}
void GLVRobject::Draw() {
	glBindVertexArray(m_vao);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_tex);

	glDrawElements(GL_TRIANGLES, m_vertCount, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}


//=========================================================================
//		GL model object class 
//=========================================================================
class GLobject {
public:
	GLuint m_vbo;
	GLuint m_ibo;
	GLuint m_vao;
	GLuint m_tex;
	GLuint m_ubo;
	GLuint m_nbo;
	GLsizei m_vertCount;
	GLsizei m_indiceCount;
	std::string m_modelName;
public:
	GLobject() {
		m_ibo = 0;
		m_tex = 0;
		m_vao = 0;
		m_vbo = 0;
		m_ubo = 0;
		m_nbo = 0;
		m_modelName = "";
	}

	GLobject(const std::string &modelName) : m_modelName(modelName) {
		m_ibo = 0;
		m_tex = 0;
		m_vao = 0;
		m_vbo = 0;
		m_ubo = 0;
		m_nbo = 0;
	}

	~GLobject() { Cleanup(); }

	void Init(const std::string &modelName);
	void InitBuffer(std::vector<float> vertices, std::vector<float> normals, std::vector<float> uvs, std::vector<unsigned int> indices);
	template <class T>
	void InitBuffer(std::vector<T> vertices, std::vector<T> normals, std::vector<T> colors);
	bool BindTexture();
	void Cleanup();
	void DrawIBO();
	void DrawVBO();
	const std::string &GetName() const { return m_modelName; }
};
void GLobject::Init(const std::string &modelName) {
	m_ibo = 0;
	m_tex = 0;
	m_vao = 0;
	m_vbo = 0;
	m_ubo = 0;
	m_nbo = 0;
	m_modelName = modelName;
}

void GLobject::InitBuffer(std::vector<float> vertices, std::vector<float> normals, std::vector<float> uvs, std::vector<unsigned int> indices) {
	m_vertCount = vertices.size() / 3;
	size_t numNormals = normals.size() / 3;
	size_t numUVs = uvs.size() / 2;
	size_t numIndices = indices.size();

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	/// position
	glEnableVertexAttribArray(0);
	glGenBuffers(1, &m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*m_vertCount * 3, vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo
				
	/// normal
	if (numNormals > 0) {
		glEnableVertexAttribArray(1);
		glGenBuffers(1, &m_nbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_nbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numNormals * 3, normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}
	/// uvs
	if (numUVs > 0) {
		glEnableVertexAttribArray(2);
		glGenBuffers(1, &m_ubo);
		glBindBuffer(GL_ARRAY_BUFFER, m_ubo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numUVs * 2, uvs.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}

	// IBO
	glGenBuffers(1, &m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*numIndices, indices.data(), GL_STATIC_DRAW);
	m_indiceCount = numIndices;
	// Reset State
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

}

template <class T>
void GLobject::InitBuffer(std::vector<T> vertices, std::vector<T> normals, std::vector<T> colors) {
	m_vertCount = vertices.size();
	size_t numNormals = normals.size();
	size_t numColors = colors.size();
	if (sizeof(T) == 4) {
		m_vertCount /= 3;
		numNormals /= 3;
		numColors /= 4;
	}

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	/// position
	glEnableVertexAttribArray(0);
	glGenBuffers(1, &m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*m_vertCount * 3, vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo

																					/// normal
	if (numNormals > 0) {
		glEnableVertexAttribArray(1);
		glGenBuffers(1, &m_nbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_nbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numNormals * 3, normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}
	/// colors
	if (numColors > 0) {
		glEnableVertexAttribArray(2);
		glGenBuffers(1, &m_ubo);
		glBindBuffer(GL_ARRAY_BUFFER, m_ubo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(int)*numColors * 3, colors.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(int), (GLvoid*)0); // bind vao to vbo
	}

	// Reset State
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// TODO
bool GLobject::BindTexture() {
	return false;
}

void GLobject::Cleanup() {
	if (m_vbo)
	{
		glDeleteVertexArrays(1, &m_vao);
		glDeleteBuffers(1, &m_vbo);
		m_vao = 0;
		m_vbo = 0;
	}
	if (m_nbo) {
		glDeleteBuffers(1, &m_nbo);
		m_nbo = 0;
	}
	if (m_ibo) {
		glDeleteBuffers(1, &m_ibo);
		m_ibo = 0;
	}
	if (m_ubo) {
		glDeleteBuffers(1, &m_ubo);
		m_ubo = 0;
	}
}
void GLobject::DrawIBO() {
	glBindVertexArray(m_vao);

	if (m_tex) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_tex);
	}

	glDrawElements(GL_TRIANGLES, m_vertCount, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}

void GLobject::DrawVBO() {
	glBindVertexArray(m_vao);
	if (m_tex) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_tex);
	}
	glDrawArrays(GL_TRIANGLES, 0, m_vertCount);
	glBindVertexArray(0);
}


//=========================================================================
//		GL Framebuffer class 
//=========================================================================
typedef struct GLfbo {
	GLuint m_depthbuf;
	GLuint m_renderTex;
	GLuint m_framebuffer;
	GLuint m_resolveTex;
	GLuint m_resolveFramebuffer;

}GLfbo;

bool CreateFBO(GLfbo &obj, int width, int height) {
	// framebuffer
	glGenFramebuffers(1, &obj.m_framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, obj.m_framebuffer);
#define USE_MULTISAMPLE
#ifdef USE_MULTISAMPLE
	glGenRenderbuffers(1, &obj.m_depthbuf);	// bind depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, obj.m_depthbuf);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, obj.m_depthbuf);

	glGenTextures(1, &obj.m_renderTex);
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, obj.m_renderTex);
	glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, width, height, true);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, obj.m_renderTex, 0);
#else
	glGenRenderbuffers(1, &obj.m_depthbuf);	// bind depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, obj.m_depthbuf);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, obj.m_depthbuf);

	glGenTextures(1, &obj.m_renderTex);
	glBindTexture(GL_TEXTURE_2D, obj.m_renderTex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, obj.m_renderTex, 0);
#endif 
	// resovleFramebuffer
	glGenFramebuffers(1, &obj.m_resolveFramebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, obj.m_resolveFramebuffer);

	glGenTextures(1, &obj.m_resolveTex);
	glBindTexture(GL_TEXTURE_2D, obj.m_resolveTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, obj.m_resolveTex, 0);

	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		return false;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return true;
}

void CleanFBO(GLfbo &obj) {
	glDeleteRenderbuffers(1, &obj.m_depthbuf);
	glDeleteTextures(1, &obj.m_renderTex);
	glDeleteFramebuffers(1, &obj.m_framebuffer);
	glDeleteTextures(1, &obj.m_resolveTex);
	glDeleteFramebuffers(1, &obj.m_resolveFramebuffer);
}
#endif // !_GL_RENDER_OBJECT_H