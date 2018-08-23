#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 normals;
layout(location = 2) in vec2 uvs;

uniform mat4 matrix;

out vec2 UVs;

void main(){
    UVs = uvs;
    gl_Position = matrix * vec4(position.xyz, 1.0);
}