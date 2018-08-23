#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec2 uvs;

noperspective out vec2 UVs;

vec4 Distort(vec4 p)
{
    vec2 v = p.xy / p.w;
    // Convert to polar coords:
    float radius = length(v);
    if (radius > 0)
    {
      float theta = atan(v.y,v.x);
      
      // Distort:
      radius = pow(radius, 0.87);

      // Convert back to Cartesian:
      v.x = radius * cos(theta);
      v.y = radius * sin(theta);
      p.xy = v.xy * p.w;
    }
    return p;
}


void main(){
    UVs = uvs;
	//vec4 distortedPos = Distort(position);
    gl_Position = position;
	//gl_Position = distortedPos;
}