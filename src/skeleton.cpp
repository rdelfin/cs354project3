#include <glm/gtx/transform.hpp>
#include <unordered_map>
#include <queue>
#include <stack>
#include <iostream>
#include "skeleton.h"

/*======================================================================
 *============================ JOINT ===================================
 *======================================================================*/
Joint::Joint(glm::vec3 offset, int parent)
    : offset(glm::vec4(offset, 1.0f)), parent(parent) { }

Joint::~Joint() { }



/*======================================================================
 *============================== BONE ==================================
 *======================================================================*/

Bone::Bone(Joint* start, Joint* end, Bone* parent)
    : startJoint(start), endJoint(end), parent(parent), rot(1.0f), trans(1.0f) {
    length = glm::length(endJoint->offset);

    // t: vector between both endpoints
    t = glm::normalize(endJoint->offset);

    // n: first, set to t and replace smallest element with t. Then n = cross(n,t)
    n = t;
    if(n.x < n.y && n.x < n.z) n.x = 1;
    else if(n.y < n.x && n.y < n.z) n.y = 1;
    else n.z = 1;
    n = glm::normalize(glm::cross(t, n));

    b = glm::cross(t, n);

    rot[0] = glm::vec4(t, 0.0f);
    rot[1] = glm::vec4(n, 0.0f);
    rot[2] = glm::vec4(b, 0.0f);
    rot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    rot = glm::transpose(rot);

    trans = glm::translate(glm::vec3(length, 0.0f, 0.0f));
}

void Bone::addChild(Bone* child) {
    children.push_back(child);
}

void Bone::addChildren(std::vector<Bone*> child) {
    children.insert(children.end(), child.begin(), child.end());
}

Bone::~Bone() {
    for(auto it = children.begin(); it != children.end(); ++it)
        delete (*it);

    if(children.size() == 0)
        delete endJoint;
    delete startJoint;
}



/*======================================================================
 *========================== SKELETON ==================================
 *======================================================================*/

Skeleton::Skeleton() {

}

Skeleton::Skeleton(Bone* root)
    : root(root) {

}

Skeleton::Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent) {
    std::vector<Joint*> joints;
    size_t rootJointIdx;
    root = nullptr;


    size_t maxSize = std::max(offset.size(), parent.size());
    for(size_t i = 0; i < maxSize; i++) {
        joints.push_back(new Joint(offset[i], parent[i]));
        if(parent[i] == -1) {
            rootJointIdx = joints.size() - 1;
        }
    }

    root = initializeBone(joints, rootJointIdx, nullptr)[0];
}

std::vector<Bone*> Skeleton::initializeBone(std::vector<Joint*> joints, int rootJointIdx, Bone* rootBone) {
    Joint* currJoint = joints[rootJointIdx];
    Joint* nextJoint = nullptr;

    std::vector<Bone*> result;

    for(size_t i = 0; i < joints.size(); i++) {
        Joint* j = joints[i];
        if(j->parent == currJoint->parent) {
            nextJoint = j;

            Bone* currBone = new Bone(currJoint, nextJoint, rootBone);

            // Create the child bone before creating this one
            currBone->addChildren(initializeBone(joints, i, currBone));
        }

        i++;
    }

    return result;
}


void Skeleton::compute_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines) {

}

void Skeleton::update_joints(std::vector<glm::vec4>& points) {
    points.clear();
    std::vector<glm::uvec2> lines;
    compute_joints(points, lines);
}

Skeleton::~Skeleton() {
    delete root;
}