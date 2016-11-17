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
    if(std::abs(n.x) <= std::abs(n.y) && std::abs(n.x) <= std::abs(n.z)) n = glm::vec3(1.0f, 0.0f, 0.0f);
    else if(std::abs(n.y) <= std::abs(n.x) && std::abs(n.y) <= std::abs(n.z)) n = glm::vec3(0.0f, 1.0f, 0.0f);
    else n = glm::vec3(0.0f, 0.0f, 1.0f);

    n = glm::normalize(glm::cross(t, n));

    b = glm::normalize(glm::cross(t, n));

    rot[0] = glm::vec4(t, 0.0f);
    rot[1] = glm::vec4(n , 0.0f);
    rot[2] = glm::vec4(b, 0.0f);
    rot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    //rot = glm::transpose(rot);

    trans = glm::translate(glm::vec3(glm::length(startJoint->offset), 0.0f, 0.0f));
}

void Bone::addChild(Bone* child) {
    children.push_back(child);
}

void Bone::addChildren(std::vector<Bone*> child) {
    children.insert(children.end(), child.begin(), child.end());
}

void Bone::compute_joints_r(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines, glm::mat4 parentTransform) {
    glm::vec4 startPoint = parentTransform * trans * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    glm::vec4 endPoint = parentTransform * trans * rot * glm::vec4(length, 0.0f, 0.0f, 1.0f);

    lines.push_back(glm::uvec2(points.size(), points.size() + 1));
    points.push_back(startPoint);
    points.push_back(endPoint);

    for(auto it = children.begin(); it != children.end(); ++it) {
        Bone* child = *it;
        child->compute_joints_r(points, lines, parentTransform * trans * rot);
    }
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

Skeleton::Skeleton()
    : root(nullptr) {
    std::cout << "Constructor" << std::endl;
}

Skeleton::Skeleton(Bone* root)
    : root(root) {
    std::cout << "Constructor" << std::endl;
}

Skeleton::Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent) {
    std::vector<Joint*> joints;
    size_t rootJointIdx = 0;
    root = nullptr;


    size_t maxSize = std::max(offset.size(), parent.size());
    for(size_t i = 0; i < maxSize; i++) {
        joints.push_back(new Joint(offset[i], parent[i]));
        if(parent[i] == -1) {
            rootJointIdx = joints.size() - 1;
        }
    }

    // Inserts special joint at the origin
    joints.push_back(new Joint(glm::vec3(0.0f, 0.0f, 0.0f), -1));

    root = new Bone(joints[joints.size() - 1], joints[rootJointIdx], nullptr);
    std::vector<Bone*> rootBones = initializeBone(joints, rootJointIdx, root);
    root->addChildren(rootBones);
}

std::vector<Bone*> Skeleton::initializeBone(std::vector<Joint*> joints, int rootJointIdx, Bone* rootBone) {
    Joint* currJoint = joints[rootJointIdx];
    Joint* nextJoint = nullptr;

    std::vector<Bone*> result;

    for(size_t i = 0; i < joints.size(); i++) {
        Joint* j = joints[i];
        if(rootJointIdx == j->parent) {
            nextJoint = j;

            Bone* currBone = new Bone(currJoint, nextJoint, rootBone);

            // Create the child bone before creating this one
            currBone->addChildren(initializeBone(joints, i, currBone));

            result.push_back(currBone);
        }
    }

    return result;
}


void Skeleton::compute_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines) {
    root->compute_joints_r(points, lines, glm::mat4(1.0f));
}

void Skeleton::update_joints(std::vector<glm::vec4>& points) {
    points.clear();
    std::vector<glm::uvec2> lines;
    compute_joints(points, lines);
}

Skeleton::~Skeleton() {
    if(root != nullptr)
        delete root;
}