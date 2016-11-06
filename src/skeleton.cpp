#include "skeleton.h"


Joint::Joint(glm::mat4 origin_rotation, glm::mat4 origin_translation, Joint* parent)
    : origin_rotation(origin_rotation), origin_translation(origin_translation), rotation(1.0f), translation(1.0f), parent(parent)
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

glm::mat4 Joint::transform() {
    return translation* rotation * origin_translation * origin_rotation;
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


glm::mat4 Skeleton::transform(Joint* joint) {
    std::vector<Joint*> path = pathTo(joint);

    if(path.size() == 0)
        return glm::mat4(1.0f);

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

Skeleton::~Skeleton() {
    if(root != nullptr)
        delete root;
}