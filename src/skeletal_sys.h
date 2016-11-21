#ifndef GLSL_SKELETAL_SYS_HPP
#define GLSL_SKELETAL_SYS_HPP

#include <unordered_map>
#include <glm/glm.hpp>
#include <mmdadapter.h>
#include <vector>

static size_t bone_id = 0;

class Joint {
public:
	Joint(glm::vec3 offset, int pid);
	~Joint();

	glm::vec3 offset;
	int pid;
};

class Bone {
private:
	int id;
	Bone* root;
	Joint *first_joint, *last_joint;
	std::vector<Bone*> leaves;

	float length;
	glm::vec3 t, n, b;
	glm::vec3 tS, bS, nS;
	glm::mat4 transformation, rotation, S;

	glm::vec4 first_w(), last_w();
	void update();

public:
	Bone(Joint* first, Joint* last, Bone* head = nullptr);
	~Bone();

	void add_leaf(Bone* leaf);
	void add_leaves(std::vector<Bone*> leaves);
	glm::mat4 translate();
	glm::mat4 transform();
	glm::mat4 rotate();
	void rotate(float rotation_speed_, glm::vec3 worldDrag);
	void roll(float theta);

	bool intersect(glm::vec3 s_b, glm::vec3 dir, float y, float& x);
	int getId() { return id; }
	float get_length() { return this->length; }

	void _calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines,
			glm::mat4 root_trans);
};

class Skeleton {
private:
	Bone* root;
	std::unordered_map<int, Bone*> bone_map;
	std::vector<Bone*> bone_vector;
	std::vector<SparseTuple> weights;

public:
	Skeleton();
	Skeleton(Bone* root);
	Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent, const std::vector<SparseTuple>& weights);
	~Skeleton();

	Bone* get_at(size_t i);
	size_t get_size() { return bone_vector.size(); }
	Bone* bone_inter(glm::vec3 b, glm::vec3 dir, float y);
	std::vector<Bone*> init_bone(std::vector<Joint*> joints, Bone* root_bone, int r_n);
	void calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines);
	void move_joints(std::vector<glm::vec4>& points);

};



#endif //GLSL_SKELETAL_SYS_HPP
