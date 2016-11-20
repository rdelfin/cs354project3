R"zzz(
#version 330 core
#define MAX_JOINT_COUNT 128
#define MAX_VERTICES 10000
uniform vec4 light_position;
uniform vec3 camera_position;
uniform mat4 skeleton_mat[MAX_JOINT_COUNT];
uniform float weights[MAX_JOINT_COUNT*MAX_VERTICES];
in vec4 vertex_position;
in vec4 normal;
in vec2 uv;
out vec4 vs_light_direction;
out vec4 vs_normal;
out vec2 vs_uv;
out vec4 vs_camera_direction;
void main() {
    int m = 0;
    mat4 totalTransform = mat4(0.0f);
    for (m = 0; m < MAX_JOINT_COUNT; m++) {
        totalTransform = totalTransform + weights[gl_VertexID*MAX_JOINT_COUNT + m]*skeleton_mat[m];
    }

    gl_Position = totalTransform * vertex_position;
    vs_light_direction = light_position - gl_Position;
    vs_camera_direction = vec4(camera_position, 1.0) - gl_Position;
    vs_normal = normal;
    vs_uv = uv;
}
)zzz"
