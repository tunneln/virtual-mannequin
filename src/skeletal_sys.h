#ifndef GLSL_SKELETAL_SYS_HPP
#define GLSL_SKELETAL_SYS_HPP

#include <unordered_map>
#include <glm/glm.hpp>
#include <vector>

class Joint;
class Bone;

class Skeleton {
private:
	Bone* root;
	std::unordered_map<int, Bone*> bone_map;

public:
	Skeleton();
	Skeleton(Bone* root);
	Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent);
	~Skeleton();

	std::vector<Bone*> init_bone(std::vector<Joint*> joints, Bone* rootBone, int r_n);
	void calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines);
	void move_joints(std::vector<glm::vec4>& points);

};


class Bone {
private:
	int id;
	double length;

	Bone* root;
	Joint *first_joint, *last_joint;
	std::vector<Bone*> leaves;
	glm::vec3 t, det;
	glm::mat4 transformation, rotation;

	glm::vec4 first_w();
	glm::vec4 last_w();
	void update();

public:
	Bone(Joint* first, Joint* last, int identifier, Bone* root = nullptr);
	~Bone();

	void add_leaf(Bone* leaf);
	void add_leaves(std::vector<Bone*> leaves);

	glm::mat4 translate();
	glm::mat4 transform();
	glm::mat4 rotate();

	void _calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines,
			glm::mat4 root_trans);
};


class Joint {
public:
	Joint(glm::vec3 offset, int pid);
	~Joint();

	glm::vec3 offset;
	int pid;
};

#endif //GLSL_SKELETAL_SYS_HPP
