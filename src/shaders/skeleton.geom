R"zzz(#version 330 core
layout (lines) in;
layout (line_strip, max_vertices = 2) out;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
void main() {
	int n = 0;
	for (n = 0; n < gl_in.length() - 1; n += 2) {
        gl_Position = gl_in[n].gl_Position.xyz;
        EmitVertex();
        gl_Position = gl_in[n+1].gl_Position.xyz;
        EmitVertex();
	    EndPrimitive();
	}
}
)zzz"
