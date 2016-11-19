#include "procedure_geometry.h"
#include "bone_geometry.h"
#include "config.h"

void create_floor(std::vector<glm::vec4>& floor_vertices, std::vector<glm::uvec3>& floor_faces)
{
	floor_vertices.push_back(glm::vec4(kFloorXMin, kFloorY, kFloorZMax, 1.0f));
	floor_vertices.push_back(glm::vec4(kFloorXMax, kFloorY, kFloorZMax, 1.0f));
	floor_vertices.push_back(glm::vec4(kFloorXMax, kFloorY, kFloorZMin, 1.0f));
	floor_vertices.push_back(glm::vec4(kFloorXMin, kFloorY, kFloorZMin, 1.0f));
	floor_faces.push_back(glm::uvec3(0, 1, 2));
	floor_faces.push_back(glm::uvec3(2, 3, 0));
}

void create_bone_mesh(Skeleton* skeleton) {}

void create_lattice_lines(std::vector<glm::vec4>& vertices, std::vector<glm::uvec2>& lines,
		size_t branch)
{
	size_t n = vertices.size();
	float step = 1.0f / (float)branch;

	for (size_t i = 0; i <= branch; i++) {
		for (size_t j = 0; j <= branch; j++) {
			vertices.push_back(glm::vec4(0.0f, i * step, j * step, 1));

			size_t adj = n + j + i * (branch + 1);
			if (i < branch) lines.push_back(glm::uvec2(adj, adj + branch));
			if (j < branch) lines.push_back(glm::uvec2(adj, adj + 1));
		}
	}
}

void create_lattice_cylinders(std::vector<glm::vec4>& vertices, std::vector<glm::vec4>& norm,
		std::vector<glm::uvec3>& faces, size_t branch)
{
	size_t n = vertices.size();
	float step = 1.0f / (float)(branch - 1);

	for (size_t i = 0; i < branch - 1; i++) {
		for (size_t j = 0; j < branch - 1; j++) {
			size_t a = n + j + i * branch;
			size_t b = n + j + (i + 1) * branch;
			size_t c = n + (j + 1) + i * branch;
			size_t d = n + (j + 1) + (i + 1) * branch;

			faces.push_back(glm::uvec3(a, c, b));
			faces.push_back(glm::uvec3(c, d, b));
		}
	}

	for (size_t i = 0; i < branch; i++) {
		for (size_t j = 0; j < branch; j++) {
			vertices.push_back(glm::vec4(0.0f, i * step, j * step, 1));
			norm.push_back(glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f));
		}
	}

}
