#include <glm/gtx/transform.hpp>
#include <unordered_map>
#include <queue>
#include <stack>
#include <iostream>
#include <mmdadapter.h>
#include "skeleton.h"
#include "config.h"

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
    : startJoint(start), endJoint(end),
      parent(parent),
      R(1.0f), T(1.0f), S(1.0f),
      length(glm::length(endJoint->offset)),
      id(BONE_ID++) {
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
    T = glm::translate(glm::vec3(qVec));

    t = glm::normalize(glm::vec3(glm::transpose(parentR)*glm::vec4(endJoint->offset, 0.0f)));
    // n: first, set to t and replace smallest element with t. Then n = cross(n,t)
    n = t;
    if(std::abs(n.x) <= std::abs(n.y) && std::abs(n.x) <= std::abs(n.z)) n = glm::vec3(1.0f, 0.0f, 0.0f);
    else if(std::abs(n.y) <= std::abs(n.x) && std::abs(n.y) <= std::abs(n.z)) n = glm::vec3(0.0f, 1.0f, 0.0f);
    else n = glm::vec3(0.0f, 0.0f, 1.0f);
    n = glm::normalize(glm::cross(t, n));

    b = glm::normalize(glm::cross(t, n));

    R[0] = glm::vec4(b, 0.0f);
    R[1] = glm::vec4(n, 0.0f);
    R[2] = glm::vec4(t, 0.0f);
    R[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    // Initialize S matrix and corresponding vectors
    S = R;
    tS = t;
    nS = n;
    bS = b;
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

    return result * T * S;
}


glm::mat4 Bone::totalRotate() {
    glm::mat4 result(1.0f);
    
    if(parent != nullptr)
        result = parent->totalRotate() * result;

    return result * S;
}

glm::mat4 Bone::totalTranslate() {
    glm::mat4 result(1.0f);
    
    if(parent != nullptr)
        result = parent->totalTranslate() * result;

    return result * T;
}

void Bone::roll(float theta) {
    glm::mat4 rollMat = glm::rotate(theta, tS);
    nS = glm::normalize(glm::vec3(rollMat * glm::vec4(nS, 0.0f)));
    bS = glm::normalize(glm::vec3(rollMat * glm::vec4(bS, 0.0f)));

    S[0] = glm::vec4(bS, 0.0f);
    S[1] = glm::vec4(nS, 0.0f);
    S[2] = glm::vec4(tS, 0.0f);
    S[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void Bone::rotate(float rotation_speed_, glm::vec3 worldDrag) {
    glm::mat4 rotateMat = glm::rotate(rotation_speed_, worldDrag);
    tS = glm::normalize(glm::vec3(rotateMat * glm::vec4(tS, 0.0f)));
    nS = glm::normalize(glm::vec3(rotateMat * glm::vec4(nS, 0.0f)));
    bS = glm::normalize(glm::vec3(rotateMat * glm::vec4(bS, 0.0f)));

    S[0] = glm::vec4(bS, 0.0f);
    S[1] = glm::vec4(nS, 0.0f);
    S[2] = glm::vec4(tS, 0.0f);
    S[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
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

bool Bone::intersects(glm::vec3 s, glm::vec3 dir, float r, float &t) {
    glm::mat4 model = transform();
    glm::mat4 invModel = glm::inverse(model);

    glm::vec3 mS = glm::vec3(invModel * glm::vec4(s, 1.0f));
    glm::vec3 mDir = glm::vec3(invModel * glm::vec4(dir, 0.0f));

    float a = pow(mDir.x, 2) + pow(mDir.y, 2);
    float b = 2*mS.x*mDir.x + 2*mS.y*mDir.y;
    float c = pow(mS.x, 2) + pow(mS.y, 2) - pow(r, 2);

    float radical = pow(b, 2) - 4*a*c;

    // Imaginary root
    if(radical < 0)
        return false;

    float sqrtRadical = sqrtf(radical);
    float t0 = (-b + sqrtRadical) / (2 * a);
    float t1 = (-b - sqrtRadical) / (2 * a);
    glm::vec3 intersectPoint0 = (mS + mDir*t0);
    glm::vec3 intersectPoint1 = (mS + mDir*t1);


    bool check0 = (t0 >= 0 && intersectPoint0.z >= 0 && intersectPoint0.z <= length);
    bool check1 = (t1 >= 0 && intersectPoint1.z >= 0 && intersectPoint1.z <= length);
    if(check0 && check1) {
        t = std::min(t0, t1);
        return true;
    } else if(check0) {
        t = t0;
        return true;
    } else if(check1) {
        t = t1;
        return true;
    }

    return false;
}

void Bone::compute_joints_r(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines, glm::mat4 parentTransform) {
    glm::vec4 startPoint = parentTransform * T * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    glm::vec4 endPoint = parentTransform * T * S * glm::vec4(0.0f, 0.0f, length, 1.0f);

    lines.push_back(glm::uvec2(points.size(), points.size() + 1));
    points.push_back(startPoint);
    points.push_back(endPoint);

    for(auto it = children.begin(); it != children.end(); ++it) {
        Bone* child = *it;
        child->compute_joints_r(points, lines, parentTransform * T * S);
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

Skeleton::Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent, const std::vector<SparseTuple>& weights)
	: weights(weights) {
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

    boneMap.insert({0, root});
    boneList.push_back(root);

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
            boneMap.insert({currBone->getId(), currBone});
            boneList.push_back(currBone);

            // Create the child bone before creating this one
            currBone->addChildren(initializeBone(joints, i, currBone));

            result.push_back(currBone);
        }
    }

    return result;
}

Bone* Skeleton::getBone(size_t idx) {
    if(idx < boneList.size())
        return boneList[idx];
    return nullptr;
}

Bone* Skeleton::intersectingBone(glm::vec3 s, glm::vec3 dir, float r) {
    float finalT = 0;
    Bone* result = nullptr;

    for(auto it = boneList.begin(); it != boneList.end(); ++it) {
        Bone* b = *it;

        float t = 0;
        if(b->intersects(s, dir, r, t)) {
            if(result == nullptr || t < finalT) {
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
