#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 color;

uniform mat4 matrix;

out vec4 inColor;

void main(){
    inColor.xyz = color;
    inColor.w = 1.0;
    gl_Position = matrix * position;
}