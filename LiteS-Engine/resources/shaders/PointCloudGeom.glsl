#version 330 core
layout (points) in;
layout (line_strip,max_vertices = 2) out;

in VS_OUT {
    vec3 Normal;
    vec3 Color;  
} gs_in[];

uniform bool isRenderNormal;

out vec3 fColor;

const float MAGNITUDE = 1.0f;

void GenerateLine(int index)
{
    gl_Position = gl_in[index].gl_Position;
    EmitVertex();
    gl_Position = gl_in[index].gl_Position + vec4(gs_in[index].Normal, 0.0f) * MAGNITUDE;
    EmitVertex();
    EndPrimitive();
}

void main()
{
    GenerateLine(0); // First vertex normal
    //GenerateLine(1); // Second vertex normal
    //GenerateLine(2); // Third vertex normal
    fColor=gs_in[0].Color;    
}