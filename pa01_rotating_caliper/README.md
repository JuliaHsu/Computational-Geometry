#Project 1: Rotating Caliper for Computing the Bounding Boxes
Hsin-Ping HSU (G01167652)

1. c_hull.cpp: implement Melkman Algorithm to compute convex hull of 2D simple polygons.
	- Put the first three vertices (P0, P1, P2) into dequeue D.
	- Iteratively compute the convex hull by checking whether the vertex is inside or outside of the processed polygon (D). 
	- To check if it's inside or ouside of the polygon, simply check if the vertex is on the left via the sign of cross product
2. bbox2d.cpp: compute the smallest bounding box through rotating calipers algorithm.
	- Use the convex hulls that were computed in Step 1.
	- Start building the bounding box on the first edge, which is vector v and its perpendicular vector n. 
	- I didn't normalize v and n until start building the bounding box to avoid rounding errors.
	- Find the extreme points with vectors v and n. And store the extreme points in ccw order. (e[0]: lower-left, e[1]: lower-right, e[3]: upper-right, and e[4]: upper-left)
	- Instead of computing the exact angles between each extreme point and the correspond vector (v or n), I used cosine^2 for comparison purpose; the bigger the cosine^2 is the smaller the angle is. 
	- Create the bounding box that is defined by 4 corners by the extreme points and vector v and n.
	- Choose the edge that has the smallest angle as new vector v,  and update extreme points and corresponding consine^2 iteratively. Only need to update the extreme points that are on the edges which have the smallest angles. (also need make sure the extreme points are still in order.)
	- Keep rotating the vectors until all the vertices have been processed, or the edge that has been processed. 

	### Output example:
	1. Simple3.poly:
		- BBox with min area: [w=12.0863, h=11.8066], (-8.35118 4.7303 ) ,(-5.16262 -6.63764 ), (6.47463 -3.37354 ) ,(3.28607 7.99439 )

		- BBox with min perimeter: [w=12.0863, h=11.8066], (-8.35118 4.7303 ) ,(-5.16262 -6.63764 ), (6.47463 -3.37354 ) ,(3.28607 7.99439 )
		
		- BBox contained in 13X13 square: [w=11.9514, h=11.9536], (-8.49939 4.51807 ) ,(-4.85093 -6.86509 ), (6.53018 -3.21729 ) ,(2.88172 8.16587 )


	2. Taiwan.poly:
		- BBox with min area: [w=10.3185, h=27.549], (-1.439 -14.6887 ) ,(8.78626 10.8924 ), (-0.795163 14.7223 ) ,(-11.0204 -10.8588 )

		- BBox with min perimeter: [w=10.3185, h=27.549], (-1.439 -14.6887 ) ,(8.78626 10.8924 ), (-0.795163 14.7223 ) ,(-11.0204 -10.8588 )

		- ! Error: Cannot find a bounding box that fits into 13X13 square

