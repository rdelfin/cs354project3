/**
 * Class that defines the Skeleton structure, implemented as a scene graph, with transforms at
 * every edge of the graph.
 */

#ifndef GLSL_SKELETON_HPP
#define GLSL_SKELETON_HPP

#include <glm/glm.hpp>

/**
 * Class that represents the skeleton for a character. Contains a full list of all nodes
 */
class Skeleton {
public:
    Skeleton();
    Skeleton(std::vector<glm::vec3> joints);
private:
    Joint* root;
    std::vector<Joint> joints;
};

/**
 * Class that represents the individual nodes in the skeleton DAG
 */
class Joint {
public:
    Joint(glm::mat4 rotation, glm::mat4 translation, Joint* parent);
private:
    glm::mat4 rotation;
    glm::mat4 translation;
    Joint* parent;
};


#endif //GLSL_SKELETON_HPP
