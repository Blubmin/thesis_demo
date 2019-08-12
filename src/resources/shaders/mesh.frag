#version 330

// Input
in vec3 v_normal;
in vec3 v_pos;

// Uniforms
struct PointLight {
    vec3 pos;
    vec3 color;
    float linear_attenuation;
    float quadratic_attenuation;
};
uniform vec3 u_camera_pos;
uniform int u_num_point_lights;
uniform PointLight[4] u_point_lights;

// Output
out vec3 f_color;

// Globals
vec3 normal_vec;
vec3 view_vec;


vec3 compute_point_light(PointLight light) {
    vec3 light_vec = normalize(light.pos - v_pos);
    vec3 half_vec = normalize(light_vec + view_vec);
    float diffuse = max(dot(half_vec, normal_vec), 0);

    float dist = length(light.pos - v_pos);
    float attenuation = 1 / (1 + light.linear_attenuation * dist + light.quadratic_attenuation * dist * dist);

    return light.color * diffuse * attenuation;
}

void main() {
    normal_vec = normalize(v_normal);
    view_vec = normalize(u_camera_pos - v_pos);
    f_color = vec3(0, 0, 0);
    for (int i = 0; i < u_num_point_lights; ++i) {
        f_color += compute_point_light(u_point_lights[i]);
    }
}