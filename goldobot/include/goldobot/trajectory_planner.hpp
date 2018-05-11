#pragma once
#include <cstdint>
#include <cstddef>
#include "goldobot/core/geometry.hpp"

namespace goldobot
{
	// Quick and dirty, to cleanup after

	struct DijkstraNode
	{
		float x;
		float y;
		float cost;
		uint16_t predecessor_id;
		uint16_t begin_arms;
		uint16_t end_arms;
	};

	struct DijkstraEdge
	{
		uint16_t point_1;// point_1 < point_2
		uint16_t point_2;
		float length;
		float yaw;
	};

	// compute dijkstra between node arms to take into account the time taken for turns
	struct DijkstraNodeArm
	{
		uint16_t origin_node_id;
		uint16_t destination_node_id; //id of node at the destination end of the arm
		uint16_t edge_id;
		uint16_t predecessor_id;
		uint16_t visited;
		float cost;
	};

    class TrajectoryPlanner
    {
    public:
    	TrajectoryPlanner();

    	// Add point. up to 64 points
    	uint16_t add_point(float x, float y);

    	// Add edge. up to 256 edges. automatically add edge in the other direction too
    	void add_edge(uint16_t i, uint16_t j);

    	// Compile graph
    	void compile();

    	// Set current point
    	void set_current_point(uint16_t i);

    	void compute_costs();

    	bool is_reachable(uint16_t i);
    	float cost(uint16_t i);

    	bool compute_trajectory(uint16_t i);
    	size_t num_trajectory_points() const;
    	Vector2D trajectory_point(uint16_t n) const;

    private:
    	DijkstraNode m_nodes[64];
    	DijkstraEdge m_edges[128];
    	DijkstraNodeArm m_node_arms[256];
		uint16_t m_trajectory[64];


    	size_t m_num_nodes;
    	size_t m_num_edges;
		size_t m_num_node_arms;
		size_t m_num_trajectory_points;
		uint16_t m_current_node;

		void initialize();
		void expand_node_arm(DijkstraNodeArm* arm);
		DijkstraNodeArm* get_min_arm();


    };
}
