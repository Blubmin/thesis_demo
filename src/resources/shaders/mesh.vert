#version 330

// Vertex Attributes
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;

// Uniforms
uniform mat4 u_perspective;
uniform mat4 u_view;
uniform mat4 u_model;

// Output
out vec3 v_normal;
out vec3 v_pos;

void main() {
    gl_Position = u_perspective * u_view * u_model * vec4(a_pos, 1);
    v_normal = (u_model * vec4(a_normal, 0)).xyz;
    v_pos = (u_model * vec4(a_pos, 1)).xyz;
}