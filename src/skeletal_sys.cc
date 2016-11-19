#include <iostream>
#include <glm/gtx/transform.hpp>
#include <queue>
#include <stack>
#include "skeletal_sys.h"

Skeleton::Skeleton() : root(nullptr) {  }

Skeleton::Skeleton(Bone* root) : root(root) {  }

Skeleton::Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent)
{
	std::vector<Joint*> joints;
	root = nullptr;
	size_t r_n = 0;

	size_t N = std::max(offset.size(), parent.size());
	for (size_t i = 0; i < N; i++) {
		joints.push_back(new Joint(offset[i], parent[i]));
		if (parent[i] == -1) r_n = joints.size() - 1;
	}
	joints.push_back(new Joint(glm::vec3(0.0f, 0.0f, 0.0f), -1));

	root = new Bone(joints[joints.size() - 1], joints[r_n], 0, nullptr);
	bone_vector.push_back(root);
	bone_map.insert({0, root});
	std::vector<Bone*> head_bones = init_bone(joints, root, r_n);
	root->add_leaves(head_bones);

}

Skeleton::~Skeleton()
{
	if (root != nullptr) delete root;
}

Bone* Skeleton::get_at(size_t i)
{
	return (i < bone_vector.size()) ? bone_vector[i] : nullptr;
}

Bone* Skeleton::bone_inter(glm::vec3 b, glm::vec3 dir, float y)
{
	float lim = std::numeric_limits<float>::infinity();
	Bone* ret;

	for (auto it = bone_vector.begin(); it != bone_vector.end(); it++) {
		float x = 0;
		Bone* bone = *it;
		if (bone->intersect(b, dir, y, x)) {
			if (x < lim) {
				lim = x;
				ret = bone;
			}
		}
	}

	return ret;
}

std::vector<Bone*> Skeleton::init_bone(std::vector<Joint*> joints, Bone* rootBone, int r_n)
{
	Joint* curr = joints[r_n];
	Joint* next = nullptr;

	std::vector<Bone*> result;

	int bid = 1;
	for (size_t i = 0; i < joints.size(); i++) {
		next = joints[i];
		if (r_n == next->pid) {
			Bone* curr_bone = new Bone(curr, next, bid, rootBone);
			bone_vector.push_back(curr_bone);
			bone_map.insert({0, root});
			curr_bone->add_leaves(init_bone(joints, curr_bone, i));

			result.push_back(curr_bone);
		}
	}

	return result;
}

void Skeleton::calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines)
{
	root->_calc_joints(points, lines, glm::mat4(1.0f));
}

void Skeleton::move_joints(std::vector<glm::vec4>& points)
{
	points.clear();
	std::vector<glm::uvec2> lines;
	calc_joints(points, lines);
}

//#####################################################################//

Bone::Bone(Joint* first, Joint* last, int identifier, Bone* root)
	: first_joint(first), last_joint(last),
	root(root), rotation(1.0f), transformation(1.0f),
	length(glm::length(last_joint->offset)),
	id(identifier)
{
	update();
}

Bone::~Bone()
{
	for (auto it = leaves.begin(); it != leaves.end(); it++)
		delete (*it);

	if (leaves.size() == 0)
		delete last_joint;
	delete first_joint;
}

void Bone::update()
{
	glm::mat4 root_t(1.0f);
	if (root != nullptr)
		root_t = root->transform();
	glm::mat4 root_r(1.0f);
	if (root != nullptr)
		root_r = root->rotate();

	glm::mat4 root_t_inv = glm::inverse(root_t);
	glm::vec4 v = root_t_inv * first_w();
	transformation = glm::translate(glm::vec3(v));

	t = glm::normalize(glm::vec3(glm::transpose(root_r) * glm::vec4(last_joint->offset, 0.0f)));
	if (std::abs(t.x) <= std::abs(t.y) && std::abs(t.x) <= std::abs(t.z))
		det = glm::vec3(1.0f, 0.0f, 0.0f);
	else if (std::abs(t.y) <= std::abs(t.x) && std::abs(t.y) <= std::abs(t.z))
		det = glm::vec3(0.0f, 1.0f, 0.0f);
	else
		det = glm::vec3(0.0f, 0.0f, 1.0f);
	det = glm::normalize(glm::cross(t, det));

	rotation[0] = glm::vec4(det, 0.0f);
	rotation[1] = glm::vec4(det, 0.0f);
	rotation[2] = glm::vec4(t, 0.0f);
	rotation[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void Bone::add_leaf(Bone* leaf)
{
	leaves.push_back(leaf);
}

void Bone::add_leaves(std::vector<Bone*> leaves)
{
	this->leaves.insert(this->leaves.end(), leaves.begin(), leaves.end());
}

glm::mat4 Bone::translate()
{
	glm::mat4 ret(1.0f);
	if (root != nullptr)
		ret *= root->translate();
	return ret * transformation;
}

glm::mat4 Bone::transform()
{
	glm::mat4 ret(1.0f);
	if (root != nullptr)
		ret *= root->transform();
	return ret * transformation * rotation;
}

glm::mat4 Bone::rotate()
{
	glm::mat4 ret(1.0f);
	if (root != nullptr)
		ret *= root->rotate();
	return ret * rotation;
}

glm::vec4 Bone::first_w()
{
	glm::vec3 ret(0.0f, 0.0f, 0.0f);
	if (root != nullptr)
		ret += glm::vec3(root->first_w());
	return glm::vec4(ret + first_joint->offset, 1.0f);
}

glm::vec4 Bone::last_w()
{
	return glm::vec4(glm::vec3(first_w()) + last_joint->offset, 1.0f);
}

bool Bone::intersect(glm::vec3 b, glm::vec3 dir, float y, float& x)
{
	glm::vec3 bone_first = glm::vec3(first_w());
	glm::vec3 bone_dir = last_joint->offset;
	glm::vec3 c = bone_first - b;

	float denom = glm::dot(dir, dir) * glm::dot(bone_dir, bone_dir) - glm::dot(dir, bone_dir) * glm::dot(dir, bone_dir);
	float x0 = (glm::dot(bone_dir, c) * glm::dot(dir, dir) - glm::dot(dir, bone_dir) * glm::dot(bone_dir, c)) / denom;
	float x1 = (glm::dot(dir, bone_dir) * glm::dot(bone_dir, c) - glm::dot(bone_dir, c) * glm::dot(dir, dir)) / denom;

	float mag = glm::length(b + dir * x0 - bone_first - bone_dir * x0);
	if (x0 >= 0 && x1 >= 0 && x1 <= 1 && mag < y) {
		x = x0;
		return true;
	}

	return false;
}

void Bone::_calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines,
		glm::mat4 root_trans)
{
	glm::vec4 beg = root_trans * transformation * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 end = root_trans * transformation * rotation * glm::vec4(0.0f, 0.0f, length, 1.0f);

	points.push_back(beg);
	points.push_back(end);
	lines.push_back(glm::uvec2(points.size(), points.size() + 1));

	for (auto it = leaves.begin(); it != leaves.end(); it++) {
		Bone* leaf = *it;
		leaf->_calc_joints(points, lines, root_trans * transformation * rotation);
	}
}

//#####################################################################//

Joint::Joint(glm::vec3 offset, int pid)
	: offset(glm::vec4(offset, 1.0f)), pid(pid) { }

Joint::~Joint() { }
