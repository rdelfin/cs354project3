/**
 * Class that defines the Skeleton structure, implemented as a scene graph, with transforms at
 * every edge of the graph.
 */

#ifndef GLSL_SKELETON_HPP
#define GLSL_SKELETON_HPP

#include <glm/glm.hpp>
#include <vector>


/**
 * Class that represents the individual nodes in the skeleton DAG
 */
class Joint {
public:
    /**
     * Creates the joint with coordinates relative to the parent. If there is no parent, the coordinates are in
     * world coordinates.
     * @param origin The position offset relative to the parent
     * @param up The up vector for the joint, where (0, 1, 0) is the up vector for the parent
     * @param right The right vector for the joint, defined the same way as the up vector
     * @param parent The parent node of the joint. Null if this is the root node
     */
    Joint(glm::vec3 offset, Joint* parent, glm::mat4 translation = glm::mat4(1.0f), glm::mat4 rotation = glm::mat4(1.0f));
    glm::mat4 transform();

    void addChild(Joint*);

    std::vector<Joint*> pathTo(Joint* joint);

    ~Joint();
private:
    glm::vec4 offset;
    glm::mat4 rotation, translation;

    Joint* parent;
    std::vector<Joint*> children;
};


/**
 * Class that represents the skeleton for a character. Contains a full list of all nodes
 */
class Skeleton {
public:
    Skeleton();
    Skeleton(Joint* root);
    Skeleton(std::vector<glm::vec3> offset, std::vector<int> parent);

    glm::vec4 transform(glm::vec4 point, Joint* joint);
    glm::mat4 transform(Joint* joint);

    ~Skeleton();
private:
    Joint* root;
    std::vector<Joint> joints;

    std::vector<Joint*> pathTo(Joint* joint);
};


#endif //GLSL_SKELETON_HPP
