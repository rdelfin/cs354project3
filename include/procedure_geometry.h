#ifndef PROCEDURE_GEOMETRY_H
#define PROCEDURE_GEOMETRY_H

#include <vector>
#include <glm/glm.hpp>
#include <skeleton.h>

void create_floor(std::vector<glm::vec4>& floor_vertices, std::vector<glm::uvec3>& floor_faces);

void create_bone_mesh(Skeleton* skeleton);

void create_lattice(std::vector<glm::vec4> &vertices, std::vector<glm::vec4> normals, std::vector<glm::uvec3> &faces,
                    size_t detail = 20);

void create_lattice_lines(std::vector<glm::vec4> &vertices, std::vector<glm::uvec2> &lines,
                    size_t detail = 20);

#endif
