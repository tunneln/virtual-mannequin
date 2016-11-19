R"zzz(
#version 330 core
in vec4 vertex_position;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
void main() {
	gl_Position = projection * view * model * vertex_position;
}
)zzz"
