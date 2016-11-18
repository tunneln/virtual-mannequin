#ifndef PROCEDURE_GEOMETRY_H
#define PROCEDURE_GEOMETRY_H

#include <vector>
#include <glm/glm.hpp>
#include "Skeleton.h"

class LineMesh;

void create_floor(std::vector<glm::vec4>& floor_vertices, std::vector<glm::uvec3>& floor_faces);
// FIXME: Add functions to generate the bone mesh.

void create_bone_mesh(Skeleton* skeleton);

void create_lattice_cylinders(std::vector<glm::vec4>& vertices, std::vector<glm::vec4>& norm,
		std::vector<glm::uvec3>& faces, size_t branch);

void create_lattice_lines(std::vector<glm::vec4>& vertices, std::vector<glm::uvec2>& lines,
		size_t branch);

#endif
