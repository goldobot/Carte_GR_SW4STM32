#include "goldobot/core/spline_trajectory.hpp"
#include <fstream>

using namespace goldobot;

int main()
{
	SplineTrajectory trajectory;
	Vector2D points_1[2] = { { 0,0 } ,{ 1,0 }};
	Vector2D points_3[5] = { { -1,0 } ,{0,0},{1,0},{1,1}, {1,2}};
	float knots_3[7] = {-2,-1,0,1,2,3,4};

	Vector2D points_2[4] = { { 0,0 } ,{ 0.7,0 },{ 1,0.2 },{ 1,2 }};

	Vector2D pts_1[] = { {1,1,}, {2,1}, {2,2}, {3,2} };
	float knots_1[] = { 0,1,2,3};



	trajectory.setPoints(pts_1, knots_1, 4);
	auto a = trajectory.minParameter();
	auto b = trajectory.maxParameter();

	std::ofstream out_file("out.txt");
	out_file << "t,x,y,tx,ty,s,c\n";
	for (int i = 0; i * 0.1f <= b; i++)
	{
		float t = i * 0.1f;
		auto tp = trajectory.computePoint(t);
		out_file << t << "," << tp.position.x << "," << tp.position.y << ",";
		out_file << tp.tangent.x <<"," << tp.tangent.y << ",";
		out_file << tp.speed << "," << tp.curvature << "\n";
	}

	return 0;
}