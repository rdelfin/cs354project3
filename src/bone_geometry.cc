#include "config.h"
#include "bone_geometry.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <glm/gtx/io.hpp>
#include <glm/gtx/transform.hpp>

/*
 * For debugging purpose.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
    size_t count = std::min(v.size(), static_cast<size_t>(10));
    for (size_t i = 0; i < count; ++i) os << i << " " << v[i] << "\n";
    os << "size = " << v.size() << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const BoundingBox& bounds)
{
    os << "min = " << bounds.min << " max = " << bounds.max;
    return os;
}



// FIXME: Implement bone animation.


Mesh::Mesh()
{
}

Mesh::~Mesh()
{
    delete skeleton;
}

void Mesh::loadpmd(const std::string& fn)
{
    MMDReader mr;
    mr.open(fn);
    mr.getMesh(vertices, faces, vertex_normals, uv_coordinates);
    computeBounds();
    mr.getMaterial(materials);


    // Fetch all joints and create a skeleton based off of that
    std::vector<glm::vec3> offsets;
    std::vector<int> parents;

    glm::vec3 vec;
    int parent;
    for(int i = 0; mr.getJoint(i, vec, parent); i++) {
        offsets.push_back(vec);
        parents.push_back(parent);
    }

    std::vector<SparseTuple> weights;
    mr.getJointWeights(weights);

    skeleton = new Skeleton(offsets, parents, weights);



}

void Mesh::updateAnimation()
{
    animated_vertices = vertices;
    // FIXME: blend the vertices to animated_vertices, rather than copy
    //        the data directly.
}


void Mesh::computeBounds()
{
    bounds.min = glm::vec3(std::numeric_limits<float>::max());
    bounds.max = glm::vec3(-std::numeric_limits<float>::max());
    for (const auto& vert : vertices) {
        bounds.min = glm::min(glm::vec3(vert), bounds.min);
        bounds.max = glm::max(glm::vec3(vert), bounds.max);
    }
}

