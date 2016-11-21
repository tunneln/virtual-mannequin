#include <iostream>
#include <glm/gtx/transform.hpp>
#include <unordered_map>
#include "config.h"
#include <queue>
#include <stack>
#include "skeletal_sys.h"
#include <mmdadapter.h>

Skeleton::Skeleton() : root(nullptr) {  }

Skeleton::Skeleton(Bone* root) : root(root) {  }

Skeleton::Skeleton(const std::vector<glm::vec3>& offset, const std::vector<int>& parent, const std::vector<SparseTuple>& weights)
{
	this->weights = weights;
	std::vector<Joint*> joints;
	root = nullptr;
	size_t r_n = 0;

	size_t N = std::max(offset.size(), parent.size());
	for (size_t i = 0; i < N; i++) {
		joints.push_back(new Joint(offset[i], parent[i]));
		if (parent[i] == -1) r_n = joints.size() - 1;
	}
	joints.push_back(new Joint(glm::vec3(0.0f, 0.0f, 0.0f), -1));

	root = new Bone(joints[joints.size() - 1], joints[r_n], nullptr);
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
	Bone* ret = NULL;

	for (auto it = bone_vector.begin(); it != bone_vector.end(); it++) {
		float x = 0;
		Bone* bone = *it;
		if (bone->intersect(b, dir, y, x)) {
			if (x < lim || ret == NULL) {
				lim = x;
				ret = bone;
			}
		}
	}

	return ret;
}

std::vector<Bone*> Skeleton::init_bone(std::vector<Joint*> joints, Bone* root_bone, int r_n)
{
	Joint* curr = joints[r_n];
	Joint* next = nullptr;

	std::vector<Bone*> ret;

	for (size_t i = 0; i < joints.size(); i++) {
		next = joints[i];
		if (r_n == next->pid) {
			Bone* curr_bone = new Bone(curr, next, root_bone);
			bone_vector.push_back(curr_bone);
			bone_map.insert({curr_bone->getId(), curr_bone});
			curr_bone->add_leaves(init_bone(joints, curr_bone, i));

			ret.push_back(curr_bone);
		}
	}

	return ret;
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

Bone::Bone(Joint* first, Joint* last, Bone* head)
	: first_joint(first), last_joint(last), root(head),
	rotation(1.0f), transformation(1.0f), S(1.0f),
	length(glm::length(last_joint->offset)),
	id(bone_id++)
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
	n = t;
	if (std::abs(n.x) <= std::abs(n.y) && std::abs(n.x) <= std::abs(n.z))
		n = glm::vec3(1.0f, 0.0f, 0.0f);
	else if (std::abs(n.y) <= std::abs(n.x) && std::abs(n.y) <= std::abs(n.z))
		n = glm::vec3(0.0f, 1.0f, 0.0f);
	else
		n = glm::vec3(0.0f, 0.0f, 1.0f);
	n = glm::normalize(glm::cross(t, n));
	b = glm::normalize(glm::cross(t, n));
	rotation[0] = glm::vec4(b, 0.0f);
	rotation[1] = glm::vec4(n, 0.0f);
	rotation[2] = glm::vec4(t, 0.0f);
	rotation[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	S = rotation, tS = t, nS = n, bS = b;
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
	return ret * transformation * S;
}

glm::mat4 Bone::rotate()
{
	glm::mat4 ret(1.0f);
	if (root != nullptr)
		ret *= root->rotate();
	return ret * S;
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

void Bone::roll(float theta)
{
	glm::mat4 roll_m = glm::rotate(theta, tS);
	nS = glm::normalize(glm::vec3(roll_m * glm::vec4(nS, 0.0f)));
	bS = glm::normalize(glm::vec3(roll_m * glm::vec4(bS, 0.0f)));

	S[0] = glm::vec4(bS, 0.0f);
	S[1] = glm::vec4(nS, 0.0f);
	S[2] = glm::vec4(tS, 0.0f);
	S[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void Bone::rotate(float rotation_speed_, glm::vec3 worldDrag)
{
	glm::mat4 rotate_m = glm::rotate(rotation_speed_, worldDrag);
	tS = glm::normalize(glm::vec3(rotate_m * glm::vec4(tS, 0.0f)));
	nS = glm::normalize(glm::vec3(rotate_m * glm::vec4(nS, 0.0f)));
	bS = glm::normalize(glm::vec3(rotate_m * glm::vec4(bS, 0.0f)));

	S[0] = glm::vec4(bS, 0.0f);
	S[1] = glm::vec4(nS, 0.0f);
	S[2] = glm::vec4(tS, 0.0f);
 	S[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

bool Bone::intersect(glm::vec3 s_b, glm::vec3 dir, float y, float& x)
{
	glm::mat4 m = transform();
	glm::mat4 _m = glm::inverse(m);
	glm::vec3 m_s = glm::vec3(_m * glm::vec4(s_b, 1.0f));
	glm::vec3 m_dir = glm::vec3(_m * glm::vec4(dir, 0.0f));

	float a = pow(m_dir.y, 2) + pow(m_dir.x, 2);
	float b = 2 * m_s.x * m_dir.x + 2 * m_s.y * m_dir.y;
	float c = pow(m_s.y, 2) + pow(m_s.x, 2) - pow(y, 2);
	float rad = pow(b, 2) - 4 * a * c;

	if(rad < 0)
		return false;

	float sqrt_rad = sqrtf(rad);
	float t0 = (-b + sqrt_rad) / (2 * a),
		  t1 = (-b - sqrt_rad) / (2 * a);
	glm::vec3 point_inter0 = (m_s + m_dir * t0);
	glm::vec3 point_inter1 = (m_s + m_dir * t1);

	bool v0 = (t0 >= 0 && point_inter0.z >= 0 &&
			point_inter0.z <= length);
	bool v1 = (t1 >= 0 && point_inter1.z >= 0 &&
			point_inter1.z <= length);

	if (v0 && v1) x = std::min(t0, t1);
	else if (v0) x = t0;
	else if (v1) x = t1;

	return (v0 || v1) ? true : false;
}

void Bone::_calc_joints(std::vector<glm::vec4>& points, std::vector<glm::uvec2>& lines,
		glm::mat4 root_trans)
{
	glm::vec4 beg = root_trans * transformation * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 end = root_trans * transformation * S * glm::vec4(0.0f, 0.0f, length, 1.0f);

	points.push_back(beg);
	points.push_back(end);
	lines.push_back(glm::uvec2(points.size(), points.size() + 1));

	for (auto it = leaves.begin(); it != leaves.end(); it++) {
		Bone* leaf = *it;
		leaf->_calc_joints(points, lines, root_trans * transformation * S);
	}
}

//#####################################################################//

Joint::Joint(glm::vec3 offset, int pid)
	: offset(glm::vec4(offset, 1.0f)), pid(pid) { }

Joint::~Joint() { }
