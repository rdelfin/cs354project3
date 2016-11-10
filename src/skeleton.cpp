#include <glm/gtx/transform.hpp>
#include <unordered_map>
#include "skeleton.h"


Joint::Joint(glm::vec3 offset, Joint* parent, glm::mat4 translation, glm::mat4 rotation)
    : offset(glm::vec4(offset, 1.0f)), rotation(rotation), translation(translation), parent(parent)
{

}

std::vector<Joint*> Joint::pathTo(Joint *joint) {
    if(this == joint)
        return { this };

    for(auto it = children.begin(); it != children.end(); ++it) {
        std::vector<Joint*> childPath = (*it)->pathTo(joint);
        if(childPath.size() != 0) {
            childPath.push_back(this);
            return childPath;
        }
    }

    return std::vector<Joint*>();

}

void Joint::addChild(Joint* child) {
    if(child != nullptr)
        children.push_back(child);
}

size_t Joint::getAllChildCount() {
    size_t count = children.size();
    for(auto it = children.begin(); it != children.end(); ++it) {
        count += (*it)->getAllChildCount();
    }

    return count;
}

glm::mat4 Joint::transform() {
    return translation * rotation * glm::translate(glm::vec3(offset));
}

Joint::~Joint() {
    for(auto it = children.begin(); it != children.end(); ++it) {
        delete *it;
    }
}



Skeleton::Skeleton() {

}

Skeleton::Skeleton(Joint* root)
    : root(root) {

}

Skeleton::Skeleton(std::vector<glm::vec3> offset, std::vector<int> parent) {
    std::unordered_map<int, Joint*> indexMap;
    root = nullptr;

    size_t minSize = std::min(offset.size(), parent.size());
    for(int i = 0; i < minSize; i++) {
        int p = parent[i];
        Joint *joint;

        if(p >= 0) {
            // Obtain parent pointer if it exists
            Joint* pPointer = indexMap[p];
            // Create pointer based on
            joint = new Joint(offset[i], pPointer);
            // Added child
            pPointer->addChild(joint);
        } else {
            // Create joint without parent
            joint = new Joint(offset[i], nullptr);
            // Therefore, this must be the root
            root = joint;
        }
        indexMap.insert({i, joint});
    }

    // Delete entire tree if no root was defined to avoid memory leaks
    if(root == nullptr) {
        for (auto it = indexMap.begin(); it != indexMap.end(); ++it) {
            delete it->second;
        }
    }
}


glm::mat4 Skeleton::transform(Joint* joint) {
    std::vector<Joint*> path = pathTo(joint);

    glm::mat4 transform(1.0f);

    // Iterate backwards through the path and compute the total transform
    for(auto it = path.rbegin(); it != path.rend(); ++it) {
        transform = (*it)->transform() * transform;
    }

    return transform;
}

glm::vec4 Skeleton::transform(glm::vec4 point, Joint* joint) {
    return transform(joint) * point;
}

std::vector<Joint*> Skeleton::pathTo(Joint* joint) {
    return root->pathTo(joint);
}

void Skeleton::compute_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines) {
    int idx = points.size();


    //points.push_back()
}

void Skeleton::update_joints(std::vector<glm::vec4>& points) {

}

void Skeleton::recomputeBoneCount() {
    if(root == nullptr)
        num_bones_cache = 0;
    else
        num_bones_cache = root->getAllChildCount();
}

Skeleton::~Skeleton() {
    if(root != nullptr)
        delete root;
}