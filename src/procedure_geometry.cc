#include "procedure_geometry.h"
#include "bone_geometry.h"
#include "config.h"

void create_floor(std::vector<glm::vec4>& floor_vertices, std::vector<glm::uvec3>& floor_faces)
{
    floor_vertices.push_back(glm::vec4(kFloorXMin, kFloorY, kFloorZMax, 1.0f));
    floor_vertices.push_back(glm::vec4(kFloorXMax, kFloorY, kFloorZMax, 1.0f));
    floor_vertices.push_back(glm::vec4(kFloorXMax, kFloorY, kFloorZMin, 1.0f));
    floor_vertices.push_back(glm::vec4(kFloorXMin, kFloorY, kFloorZMin, 1.0f));
    floor_faces.push_back(glm::uvec3(0, 1, 2));
    floor_faces.push_back(glm::uvec3(2, 3, 0));
}

void create_bone_mesh(Skeleton* skeleton) {

}

// FIXME: create cylinders and lines for the bones
// Hints: Generate a lattice in [-0.5, 0, 0] x [0.5, 1, 0] We wrap this
// around in the vertex shader to produce a very smooth cylinder.  We only
// need to send a small number of points.  Controlling the grid size gives a
// nice wireframe.

/**
 * Generates a latice in [0, 0, 0] x [1, 1, 0] with `detail` vertices per side
 */
void create_lattice(std::vector<glm::vec4> &vertices, std::vector<glm::vec4> normals, std::vector<glm::uvec3> &faces,
                    size_t detail) {
    size_t idx = vertices.size();
    float stepsize = 1.0f / (float)(detail-1);

    for(size_t i = 0; i < detail; i++) {
        for(size_t j = 0; j < detail; j++) {
            vertices.push_back(glm::vec4(j*stepsize - 0.5f, i*stepsize, 0.0f, 1));
            normals.push_back(glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f));
        }
    }

    for(size_t i = 0; i < detail - 1; i++) {
        for(size_t j = 0; j < detail - 1; j++) {
            size_t ll = idx + j + i*detail,
                    ul = idx + j + (i+1)*detail,
                    lr = idx + (j + 1) + i*detail,
                    ur = idx + (j + 1) + (i + 1)*detail;


            faces.push_back(glm::uvec3(ll, lr, ul));
            faces.push_back(glm::uvec3(lr, ur, ul));
        }
    }
}

void create_lattice_lines(std::vector<glm::vec4> &vertices, std::vector<glm::uvec2> &lines, size_t detail) {
    size_t offset_idx = vertices.size();
    float stepsize = 1.0f / (float)detail;

    for(size_t i = 0; i < detail; i++) {
        for(size_t j = 0; j < detail; j++) {
            vertices.push_back(glm::vec4(j*stepsize, i*stepsize, 0.0f, 1));

            size_t current_idx = offset_idx + j + i*detail;

            // Not last vertex on Y index
            if(i < detail - 1)
                lines.push_back(glm::uvec2(current_idx, current_idx + detail));

            // Not last vertex on X index
            if(j < detail - 1)
                lines.push_back(glm::uvec2(current_idx, current_idx + 1));
        }
    }
}
