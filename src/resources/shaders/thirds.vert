#version 330

layout(location = 0) in vec2 a_pos;

void main() {
	gl_Position = vec4(a_pos * 2 - vec2(1), 0, 1);
}