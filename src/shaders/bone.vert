R"zzz(
#version 330 core
in vec4 vertex_position;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
uniform mat4 radius;
void main() {
	vec4 wrapped_position = vec4(0.0f, 0.0f, 0.0f, 1.0f);
	float pi = 3.14159265;
	wrapped_position.x = cos(2 * pi * vertex_position.y) * radius;
	wrapped_position.y = sin(2 * pi * vertex_position.y) * radius;
	wrapped_position.z = vertex_position.z;

	gl_Position = projection * view * model * vertex_position;
}
)zzz"
