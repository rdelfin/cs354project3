#include "skeleton.h"


Joint::Joint(glm::vec3 origin, glm::vec3 up, glm::vec3 right, Joint* parent)
    : origin(glm::vec4(origin, 1)), origin_up(glm::vec4(up, 0)), origin_right(glm::vec4(right, 0)), parent(parent)
{

}

std::vector<Joint*> Joint::pathTo(Joint *joint) {

}

void Joint::addChild(Joint* child) {
    if(child != nullptr)
        children.push_back(child);
}


Skeleton::Skeleton(Joint* root)
    : root(root) {

}

glm::vec4 Skeleton::transform(glm::vec4 point, Joint* joint) {

}

std::vector<Joint*> Skeleton::pathTo(Joint* joint) {
    return root->pathTo(joint);
}