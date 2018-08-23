#version 330 core

noperspective in vec2 UVs;
out vec4 outColor;

uniform sampler2D tex;

vec2 Distort(vec2 p)
{
    p = 2.0 * p - 1.0;
    float theta  = atan(p.y, p.x);
    float radius = length(p);
    radius = pow(radius, 1.2);
    p.x = radius * cos(theta);
    p.y = radius * sin(theta);
    return 0.5 * (p + 1.0);
}
//https://forums.oculusvr.com/developer/discussion/88/opengl-full-example-and-shader
void main(){
    outColor = vec4(0.0, 0.0, 0.0, 1.0);
    vec2 uv = Distort(UVs);
    if (!any(bvec2(clamp(uv, vec2(0.03), vec2(0.98))-uv))) {
        outColor = texture(tex, uv);
    }
    //outColor = texture(tex, uv);
    //outColor = texture(tex, UVs);
    //outColor = vec4(UVs.x, UVs.y, 0.0, 1.0);
}