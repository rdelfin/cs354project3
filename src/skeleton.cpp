#include <glm/gtx/transform.hpp>
#include <unordered_map>
#include <queue>
#include <stack>
#include <iostream>
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

Skeleton::Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent) {
    std::unordered_map<int, Joint*> indexMap;
    root = nullptr;

    size_t minSize = std::min(offset.size(), parent.size());
    for(int i = 0; i < minSize; i++) {
        int p = parent[i];
        Joint *joint;

        //std::cout << "ADDED JOINT(" << i << ") WITH PARENT " << p << " AND OFFSET (" << offset[i].x << ", " << offset[i].y << ", " << offset[i].z << ")" << std::endl;

        if(p >= 0) {
            // Create pointer with no parent. It will be determined afterwards
            joint = new Joint(offset[i], nullptr);
            //std::cout << "Parent children length: " << pPointer->children.size() << std::endl;
        } else {
            // Create joint without parent
            joint = new Joint(offset[i], nullptr);
            // Therefore, this must be the root
            root = joint;

            std::cout << "Root pointer: " << root << std::endl;
        }
        indexMap.insert({i, joint});
    }

    std::cout << "Index map has " << indexMap.size() << " elements. Offset has " << offset.size() << " elements" << std::endl;

    for(auto it = indexMap.begin(); it != indexMap.end(); ++it) {
        int parentIdx = parent[it->first];
        if(parentIdx >= 0) {
            Joint* pPointer = indexMap[parentIdx];
            pPointer->addChild(it->second);
            if(pPointer->children.size() != 0)
                std::cout << "CHILD SIZE " << pPointer->children.size() << " for index " << it->first << std::endl;
            it->second->parent = pPointer;
        }
    }

    for(auto it = indexMap.begin(); it != indexMap.end(); ++it) {
        std::cout << "[" << it->first << "] has " << it->second->children.size() << " children" << std::endl;
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

    return pathTransform(path);
}

glm::vec4 Skeleton::transform(glm::vec4 point, Joint* joint) {
    return transform(joint) * point;
}

std::vector<Joint*> Skeleton::pathTo(Joint* joint) {
    return root->pathTo(joint);
}

glm::mat4 Skeleton::pathTransform(const std::vector<Joint*>& path) {
    glm::mat4 transform(1.0f);
    // Iterate backwards through the path and compute the total transform
    for(auto it = path.rbegin(); it != path.rend(); ++it) {
        transform = (*it)->transform() * transform;
    }

    return transform;
}

void Skeleton::compute_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines) {
    std::queue<Joint*> jointQueue;
    std::vector<Joint*> jointPath;

    std::cout << "Root pointer (pt 2): " << root << std::endl;

    // Push back joint
    if(root != nullptr)
        jointQueue.push(root);

    while(jointQueue.size() > 0) {
        // Pop out element to process
        Joint* parent = jointQueue.front();
        jointQueue.pop();

        // Transform for this path
        glm::mat4 tmat = pathTransform(jointPath);

        // Index of parent point
        int pIdx = points.size();

        // Add parent to joint for transform
        points.push_back(tmat * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

        std::cout << "Parent children length: " << parent->children.size() << std::endl;
        std::cout << "Parent pointer: " << parent << std::endl;

        for (auto it = parent->children.begin(); it != parent->children.end(); ++it) {
            Joint* child = *it;

            lines.push_back(glm::uvec2(pIdx, points.size()));
            points.push_back(tmat * child->transform() * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));


            // Add child to work queue
            jointQueue.push(child);
        }

        // If the next element is the parent of the current element, the joint path should go "up" by popping out
        if(jointQueue.front() == parent->parent)
            jointPath.pop_back();
    }

}

void Skeleton::update_joints(std::vector<glm::vec4>& points) {
    std::vector<glm::uvec2> lines;
    compute_joints(points, lines);
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