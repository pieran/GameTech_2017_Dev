#version 150 core

uniform sampler2D fontTex;

in Vertex	{
	vec4 colour;
	vec2 texCoords;
} IN;

out vec4 OutFrag;

void main(void)	{
	OutFrag = texture(fontTex, IN.texCoords) * IN.colour;
	
	if (OutFrag.w < 0.1f)
	{
		discard;
	}
	//OutFrag.w = 1.0f;
}