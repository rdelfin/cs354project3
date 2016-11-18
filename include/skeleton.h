/**
 * Class that defines the Skeleton structure, implemented as a scene graph, with transforms at
 * every edge of the graph.
 */

#ifndef GLSL_SKELETON_HPP
#define GLSL_SKELETON_HPP

#include <glm/glm.hpp>
#include <vector>


/**
 * Class that represents the individual nodes in the skeleton tree
 */
class Joint {
public:
    Joint(glm::vec3 offset, int parent);

    ~Joint();

    glm::vec3 offset;
    int parent;
};

class Bone {
public:
    Bone(Joint* start, Joint* end, Bone* parent);

    void addChild(Bone* child);
    void addChildren(std::vector<Bone*> child);

    glm::mat4 transform();
    glm::mat4 totalRotate();
    glm::mat4 totalTranslate();

    void compute_joints_r(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines, glm::mat4 parentTransform);

    ~Bone();

private:
    Joint *startJoint, *endJoint;

    Bone* parent;
    double length;
    std::vector<Bone*> children;

    void updateBasis();
    glm::vec4 startWorld();
    glm::vec4 endWorld();

    glm::vec3 t, n, b;
    glm::mat4 trans, rot, totM;
};


/**
 * Class that represents the skeleton for a character. Contains a full list of all nodes
 */
class Skeleton {
public:
    Skeleton();
    Skeleton(Bone* root);
    Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent);

    std::vector<Bone*> initializeBone(std::vector<Joint*> joints, int rootJointIdx, Bone* rootBone);

    void compute_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines);
    void update_joints(std::vector<glm::vec4>& points);

    ~Skeleton();
private:
    Bone* root;
};


#endif //GLSL_SKELETON_HPP
