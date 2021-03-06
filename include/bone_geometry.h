#ifndef BONE_GEOMETRY_H
#define BONE_GEOMETRY_H

#include <ostream>
#include <vector>
#include <map>
#include <limits>
#include <glm/glm.hpp>
#include <mmdadapter.h>
#include <skeleton.h>

struct BoundingBox {
    BoundingBox()
            : min(glm::vec3(-std::numeric_limits<float>::max())),
              max(glm::vec3(std::numeric_limits<float>::max())) {}
    glm::vec3 min;
    glm::vec3 max;
};


struct Mesh {
    Mesh();
    ~Mesh();
    std::vector<glm::vec4> vertices;
    std::vector<glm::vec4> animated_vertices;
    std::vector<glm::uvec3> faces;
    std::vector<glm::vec4> vertex_normals;
    std::vector<glm::vec4> face_normals;
    std::vector<glm::vec2> uv_coordinates;
    std::vector<Material> materials;
    BoundingBox bounds;
    Skeleton* skeleton;

    void loadpmd(const std::string& fn);
    void updateAnimation();
    int getNumberOfBones() const
    {
        return skeleton->getNumberOfBones();
    }
    glm::vec3 getCenter() const { return 0.5f * glm::vec3(bounds.min + bounds.max); }
private:
    void computeBounds();
    void computeNormals();
};

#endif
