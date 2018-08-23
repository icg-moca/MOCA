#version 330 core

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;

uniform mat4 model_to_vive;
uniform mat4 vive_to_eye;

out vec3 inColor;
out vec3 inNormal;

void main(){
    vec4 world = model_to_vive * vec4(pos, 1.0);
    gl_Position = vive_to_eye * world;
    inColor = color;
    gl_PointSize = 3.0;
}