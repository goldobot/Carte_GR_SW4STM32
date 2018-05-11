#include "goldobot/trajectory_planner.hpp"
#include <cmath>

using namespace goldobot;

TrajectoryPlanner::TrajectoryPlanner() :
	m_num_nodes(0),
	m_num_edges(0),
	m_current_node(0)
{

}

size_t TrajectoryPlanner::num_trajectory_points() const
{
	return m_num_trajectory_points;
}

Vector2D TrajectoryPlanner::trajectory_point(uint16_t n) const
{
	return {m_nodes[m_trajectory[n]].x, m_nodes[m_trajectory[n]].y};
}
uint16_t TrajectoryPlanner::add_point(float x, float y)
{
	m_nodes[m_num_nodes] = { x,y,0,0 };
	m_num_nodes++;
	return m_num_nodes - 1;
}

void TrajectoryPlanner::add_edge(uint16_t i, uint16_t j)
{
	if (i < j)
	{
		m_edges[m_num_edges] = { i,j };
		m_num_edges++;
	}
	else if (j < i)
	{
		m_edges[m_num_edges] = { j, i};
		m_num_edges++;
	}	
}

void TrajectoryPlanner::compile()
{
	uint16_t num_node_arms=0;

	for (unsigned j = 0; j < m_num_edges; j++)
	{
		auto n1 = m_nodes[m_edges[j].point_1];
		auto n2 = m_nodes[m_edges[j].point_2];
		float dx = n2.x - n1.x;
		float dy = n2.y - n1.y;
		m_edges[j].length = sqrtf(dx*dx + dy*dy);
		m_edges[j].yaw = atan2f(dy, dx);

	}
	for (unsigned i = 0; i < m_num_nodes; i++)
	{
		m_nodes[i].begin_arms = num_node_arms;
		for (unsigned j = 0; j < m_num_edges; j++)
		{
			if (m_edges[j].point_1 == i)
			{
				m_node_arms[num_node_arms].edge_id = j;
				m_node_arms[num_node_arms].destination_node_id = m_edges[j].point_2;
				m_node_arms[num_node_arms].origin_node_id = i;
				num_node_arms++;
			}
			if (m_edges[j].point_2 == i)
			{
				m_node_arms[num_node_arms].edge_id = j;
				m_node_arms[num_node_arms].destination_node_id = m_edges[j].point_1;
				m_node_arms[num_node_arms].origin_node_id = i;
				num_node_arms++;
			}
			m_nodes[i].end_arms = num_node_arms;
		}
	}
	m_num_node_arms = num_node_arms;
}

float TrajectoryPlanner::cost(uint16_t i)
{
	return m_nodes[i].cost;
}

void TrajectoryPlanner::set_current_point(uint16_t i)
{
	m_current_node = i;
}
bool TrajectoryPlanner::compute_trajectory(uint16_t idx)
{
	// Count number of elements in trajectory
	// Current point is included
	m_num_trajectory_points = 1;
	uint16_t cur_node = idx;

	while(cur_node != m_nodes[cur_node].predecessor_id)
	{
		m_num_trajectory_points++;
		cur_node = m_nodes[cur_node].predecessor_id;
	}
	if (m_num_trajectory_points > 0)
	{
		size_t i = m_num_trajectory_points;
		cur_node = idx;
		while (i != 0)
		{
			i--;
			m_trajectory[i] = cur_node;
			cur_node = m_nodes[cur_node].predecessor_id;			
		}
		return true;
	}
	else
	{
		return false;
	}
	
}
void TrajectoryPlanner::initialize()
{	
	for (unsigned i = 0; i < m_num_nodes; i++)
	{
		m_nodes[i].cost = 1e9;
		m_nodes[i].predecessor_id = i;
	}
	for (unsigned i = 0; i < m_num_node_arms; i++)
	{
		m_node_arms[i].cost = 1e9;
		m_node_arms[i].predecessor_id = i;
		m_node_arms[i].visited = 0;
	}	
}


void TrajectoryPlanner::compute_costs()
{
	initialize();

	DijkstraNode* start_node = &m_nodes[m_current_node];
	for (unsigned i = start_node->begin_arms; i < start_node->end_arms; i++)
	{
		DijkstraNodeArm* arm = &m_node_arms[i];
		arm->cost = m_edges[arm->edge_id].length;
		arm->predecessor_id = -1;
	}

	// Run Dijkstra algorithm on node arms
	while (1)
	{
		DijkstraNodeArm* min_arm = get_min_arm();
		if (min_arm == nullptr)
		{
			break;
		}
		expand_node_arm(min_arm);
	}

	// Compute node costs
	for (unsigned i = 0; i < m_num_node_arms; i++)
	{
		DijkstraNodeArm* arm = &m_node_arms[i];
		if (arm->cost < m_nodes[arm->destination_node_id].cost)
		{
			m_nodes[arm->destination_node_id].cost = arm->cost;
			m_nodes[arm->destination_node_id].predecessor_id = arm->origin_node_id;
		}		
	}
	m_nodes[m_current_node].cost = 0;
	m_nodes[m_current_node].predecessor_id = m_current_node;
}


void TrajectoryPlanner::expand_node_arm(DijkstraNodeArm* arm)
{
	arm->visited = 1;
	DijkstraNode* node = &m_nodes[arm->destination_node_id];

	for (unsigned i = node->begin_arms; i < node->end_arms; i++)
	{
		DijkstraNodeArm* arm_dest = &m_node_arms[i];
		float cost = arm->cost + m_edges[arm_dest->edge_id].length;
		if (cost < arm_dest->cost)
		{
			arm_dest->cost = cost;
			arm_dest->predecessor_id = (uint16_t)(arm - m_node_arms);
		}
	}
}

DijkstraNodeArm* TrajectoryPlanner::get_min_arm()
{
	float min_val = 1e9;
	DijkstraNodeArm* arm = nullptr;

	for (unsigned i = 0; i < m_num_node_arms; i++)
	{
		if (m_node_arms[i].cost < min_val && m_node_arms[i].visited == 0)
		{
			min_val = m_node_arms[i].cost;
			arm = &m_node_arms[i];
		}		
	}
	return arm;
}
