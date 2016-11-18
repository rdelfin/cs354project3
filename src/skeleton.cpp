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

Bone::Bone(Joint* start, Joint* end, int identifier, Bone* parent)
    : startJoint(start), endJoint(end),
      parent(parent), rot(1.0f), trans(1.0f),
      length(glm::length(endJoint->offset)),
      id(identifier) {
    // Rotation and translation matrix calculation
    updateBasis();
}

void Bone::updateBasis() {
    glm::mat4 parentM(1.0f);
    if(parent != nullptr)
        parentM = parent->transform();

    glm::mat4 parentR(1.0f);
    if(parent != nullptr)
        parentR = parent->totalRotate();

    glm::mat4 parentMInv = glm::inverse(parentM);

    glm::vec4 qVec = parentMInv*startWorld();
    trans = glm::translate(glm::vec3(qVec));

    t = glm::normalize(glm::vec3(glm::transpose(parentR)*glm::vec4(endJoint->offset, 0.0f)));
    // n: first, set to t and replace smallest element with t. Then n = cross(n,t)
    n = t;
    if(std::abs(n.x) <= std::abs(n.y) && std::abs(n.x) <= std::abs(n.z)) n = glm::vec3(1.0f, 0.0f, 0.0f);
    else if(std::abs(n.y) <= std::abs(n.x) && std::abs(n.y) <= std::abs(n.z)) n = glm::vec3(0.0f, 1.0f, 0.0f);
    else n = glm::vec3(0.0f, 0.0f, 1.0f);
    n = glm::normalize(glm::cross(t, n));

    b = glm::normalize(glm::cross(t, n));

    rot[0] = glm::vec4(b, 0.0f);
    rot[1] = glm::vec4(n, 0.0f);
    rot[2] = glm::vec4(t, 0.0f);
    rot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void Bone::addChild(Bone* child) {
    children.push_back(child);
}

void Bone::addChildren(std::vector<Bone*> child) {
    children.insert(children.end(), child.begin(), child.end());
}

glm::mat4 Bone::transform() {
    glm::mat4 result(1.0f);

    if(parent != nullptr)
        result  = parent->transform() * result;

    return result * trans * rot;
}


glm::mat4 Bone::totalRotate() {
    glm::mat4 result(1.0f);
    
    if(parent != nullptr)
        result = parent->totalRotate() * result;

    return result * rot;
}

glm::mat4 Bone::totalTranslate() {
    glm::mat4 result(1.0f);
    
    if(parent != nullptr)
        result = parent->totalTranslate() * result;

    return result * trans;
}


glm::vec4 Bone::startWorld() {
    glm::vec3 result(0.0f, 0.0f, 0.0f);
    if(parent != nullptr)
        result += glm::vec3(parent->startWorld());
    return glm::vec4(result + startJoint->offset, 1.0f);
}

glm::vec4 Bone::endWorld() {
    return glm::vec4(glm::vec3(startWorld()) + endJoint->offset, 1.0f);
}

bool Bone::intestects(glm::vec3 s, glm::vec3 dir, float r, float& t) {
    glm::vec3 boneDir = endJoint->offset;
    glm::vec3 boneStart = glm::vec3(startWorld());

    glm::vec3 c = boneStart - s;

    float denominator = glm::dot(dir, dir)*glm::dot(boneDir, boneDir) - glm::dot(dir, boneDir)*glm::dot(dir, boneDir);
    float ta = (-glm::dot(dir, boneDir)*glm::dot(boneDir, c) + glm::dot(boneDir, c)*glm::dot(dir, dir)) / denominator;
    float tb = (glm::dot(dir, boneDir)*glm::dot(boneDir, c) - glm::dot(boneDir, c)*glm::dot(dir, dir)) / denominator;

    float distance = glm::length(s + dir*ta - boneStart - boneDir*ta);

    if(ta >= 0 && tb >= 0 && tb <= 1 && distance < r) {
        t = ta;
        return true;
    }

    return false;
}

void Bone::compute_joints_r(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines, glm::mat4 parentTransform) {
    glm::vec4 startPoint = parentTransform * trans * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    glm::vec4 endPoint = parentTransform * trans * rot * glm::vec4(0.0f, 0.0f, length, 1.0f);

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
}

Skeleton::Skeleton(Bone* root)
    : root(root) {
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

    root = new Bone(joints[joints.size() - 1], joints[rootJointIdx], 0, nullptr);

    boneMap.insert({0, root});
    boneList.push_back(root);

    std::vector<Bone*> rootBones = initializeBone(joints, rootJointIdx, root);
    root->addChildren(rootBones);
}

std::vector<Bone*> Skeleton::initializeBone(std::vector<Joint*> joints, int rootJointIdx, Bone* rootBone) {
    Joint* currJoint = joints[rootJointIdx];
    Joint* nextJoint = nullptr;

    std::vector<Bone*> result;

    int boneId = 1;  // Skips over root bone
    for(size_t i = 0; i < joints.size(); i++) {
        Joint* j = joints[i];
        if(rootJointIdx == j->parent) {
            nextJoint = j;

            Bone* currBone = new Bone(currJoint, nextJoint, boneId++, rootBone);
            boneMap.insert({boneId, currBone});
            boneList.push_back(currBone);

            // Create the child bone before creating this one
            currBone->addChildren(initializeBone(joints, i, currBone));

            result.push_back(currBone);
        }
    }

    return result;
}

Bone* Skeleton::intersectingBone(glm::vec3 s, glm::vec3 dir, float r) {
    float finalT = std::numeric_limits::infinity();
    Bone* result = nullptr;

    for(auto it = boneList.begin(); it != boneList.end(); ++it) {
        Bone* b = *it;
        float t = 0;
        if(b->intestects(s, dir, r, t)) {
            if(t < finalT) {
                finalT = t;
                result = b;
            }
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
