#pragma once

#include "Core.h"
#include "Sampling.h"
#include "DebugHelper.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		
		t = -(Dot(n, r.o) + d) / Dot(n, r.dir);
		if (t>=0)
		{
			return true;
		}
		return false;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[1].p - vertices[0].p;
		e2 = vertices[2].p - vertices[0].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		/*Plane plane;
		Vec3 planeNormal = n;
		plane.init(planeNormal, -d);
		
		if (plane.rayIntersect(const_cast<Ray&>(r), t))
		{
			Vec3 P = r.at(t);
			Vec3 q1 = P - vertices[0].p;
			Vec3 C1 = e1.cross(q1);
			float invA = 1.0/ Dot((e1.cross(e2)), n);
			float alpha = Dot(C1, n) * invA;

			Vec3 q2 = P - vertices[1].p;
			Vec3 C2 = e2.cross(q2);
			float beta = Dot(C2, n) * invA;

			u = alpha;
			v = beta;

			if (alpha >= 0 && beta >= 0 && (alpha + beta) <= 1)
			{
				return true;
				
			}
			 
		}
		return false;*/

		// Moller-Trumbore
		Vec3 p = r.dir.cross(e2);
		float det = e1.dot(p);
		
		if (abs(det)<EPSILON)
		{
			return false;
		}
		Vec3 T = r.o - vertices[0].p;
		float beta = (T.dot(p)) / det;
		u = beta;
		
		if (beta < 0 || beta > 1) return false;
		Vec3 q = T.cross(e1);
		float gamma = (r.dir.dot(q)) / det;
		v = gamma;
		if (gamma < 0 || gamma>1 || (beta + gamma) > 1) return false;
		t = (e2.dot(q)) / det;
		if (t < 0) return false;
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(tmin, tmax);
		Vec3 Texit = Max(tmin, tmax);
		float tentry = std::max(std::max(Tentry.x, Tentry.y), Tentry.z);
		float texit = std::min(std::min(Texit.x, Texit.y), Texit.z);
		t = std::min(tentry, texit);
		return (tentry <= texit&&texit >= 0);
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(tmin, tmax);
		Vec3 Texit = Max(tmin, tmax);
		float tentry = std::max(std::max(Tentry.x, Tentry.y), Tentry.z);
		float texit = std::min(std::min(Texit.x, Texit.y), Texit.z);
		return (tentry <= texit && texit >= 0);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		// ray - sphere求交

		Vec3 L = centre - r.o;
		float tca = L.dot(r.dir);
		if (tca < 0) return false;
		float d2 = L.dot(L) - tca * tca;
		if (d2 > radius * radius) return false;
		float thc = sqrt(radius * radius - d2);
		t = tca - thc;
		float t1 = tca + thc;
		if (t < 0) t = t1;
		if (t < 0) return false;
		return true ;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

struct Bin
{
	AABB bounds;
	unsigned int count;
	Bin() : count(0) {}

	void reset()
	{
		bounds.reset();
		count = 0;
	}
};

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	std::vector<unsigned int> triangleIndices;

	bool isLeaf()
	{
		return (r == NULL && l == NULL);
	}

	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void subdivide(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles,std::vector<unsigned int>& indices)
	{
		// Add BVH building code here
		// The count of triangles is less than MAXNODE_TRIANGLES, create a leaf node
		if (indices.size() <= MAXNODE_TRIANGLES)
		{
			for (int i = 0; i < indices.size(); i++)
			{
				triangleIndices.push_back(indices[i]);
				outputTriangles.push_back(inputTriangles[indices[i]]);
			}
			return;
		}
		// find best split in three axises using SAH
		int bestAxis = -1;
		int bestBin = -1;
		float bestCost = FLT_MAX;
		float parentArea = bounds.area();

		for (int axis = 0; axis < 3; axis++)
		{
			float minBound = (axis == 0) ? bounds.min.x : (axis == 1) ? bounds.min.y : bounds.min.z;
			float maxBound = (axis == 0) ? bounds.max.x : (axis == 1) ? bounds.max.y : bounds.max.z;
			
			if (minBound == maxBound) continue; 

			Bin bins[BUILD_BINS];
			float binSize = (maxBound - minBound) / BUILD_BINS;
			// assign triangles to bins
			for (unsigned int index : indices)
			{
				Vec3 centre = inputTriangles[index].centre();
				int binIndex = std::min((int)((((axis == 0) ? centre.x : (axis == 1) ? centre.y : centre.z) - minBound) / binSize), BUILD_BINS - 1);
				bins[binIndex].count++;
				bins[binIndex].bounds.extend(inputTriangles[index].vertices[0].p);
				bins[binIndex].bounds.extend(inputTriangles[index].vertices[1].p);
				bins[binIndex].bounds.extend(inputTriangles[index].vertices[2].p);
			}
			// evaluate split cost
			Bin leftBins[BUILD_BINS];
			Bin rightBins[BUILD_BINS];
			// prefix sum
			leftBins[0] = bins[0];
			for (int i = 1; i < BUILD_BINS; i++)
			{
				leftBins[i].bounds = leftBins[i - 1].bounds;
				leftBins[i].count = leftBins[i - 1].count;
				leftBins[i].bounds.extend(bins[i].bounds.min);
				leftBins[i].bounds.extend(bins[i].bounds.max);	
				leftBins[i].count += bins[i].count;
			}
			// suffix sum
			rightBins[BUILD_BINS - 1] = bins[BUILD_BINS - 1];
			for (int i = BUILD_BINS - 2; i >= 0; i--)
			{
				rightBins[i].bounds = rightBins[i + 1].bounds;
				rightBins[i].count = rightBins[i + 1].count;
				rightBins[i].bounds.extend(bins[i].bounds.min);
				rightBins[i].bounds.extend(bins[i].bounds.max);
				rightBins[i].count += bins[i].count;
			}
			// find best split in this axis
			for (int i = 1; i < BUILD_BINS; i++)
			{
				unsigned int lCount = leftBins[i - 1].count;
				unsigned int rCount = rightBins[i].count;

				float lArea = leftBins[i - 1].bounds.area();
				float rArea = rightBins[i].bounds.area();
				float cost = (lArea / parentArea) * lCount * TRIANGLE_COST + (rArea / parentArea) * rCount * TRIANGLE_COST + TRAVERSE_COST;
				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestBin = i;
				}
			}
			
		}
		// using SAH to best split
		float leafCost = indices.size() * TRIANGLE_COST;
		if (bestAxis == -1 || leafCost < bestCost)
		{
			for (int i = 0; i < indices.size(); i++)
			{
				triangleIndices.push_back(indices[i]);
				outputTriangles.push_back(inputTriangles[indices[i]]);
			}
			return;
		}
		float minBound = (bestAxis == 0) ? bounds.min.x : (bestAxis == 1) ? bounds.min.y : bounds.min.z;
		float maxBound = (bestAxis == 0) ? bounds.max.x : (bestAxis == 1) ? bounds.max.y : bounds.max.z;
		float splitPos = minBound + (bestBin * ((maxBound - minBound) / BUILD_BINS));

		std::vector<unsigned int> leftIndices, rightIndices;
		for (unsigned int index : indices)
		{
			Vec3 centre = inputTriangles[index].centre();
			if (((bestAxis == 0) ? centre.x : (bestAxis == 1) ? centre.y : centre.z) < splitPos)
			{
				leftIndices.push_back(index);
			}
			else
			{
				rightIndices.push_back(index);
			}
		}
		// if one side is empty, split equally
		if (leftIndices.empty() || rightIndices.empty())
		{
			for (unsigned int idx : indices)
			{
				triangleIndices.push_back((unsigned int)outputTriangles.size());
				outputTriangles.push_back(inputTriangles[idx]);
			}
			return;
		}
		// create child left/right nodes
		// left
		l = new BVHNode();
		l->bounds.reset();
		for (unsigned int index : leftIndices)
		{
			l->bounds.extend(inputTriangles[index].vertices[0].p);
			l->bounds.extend(inputTriangles[index].vertices[1].p);
			l->bounds.extend(inputTriangles[index].vertices[2].p);
		}
		l->subdivide(inputTriangles, outputTriangles, leftIndices);
		// right
		r = new BVHNode();
		r->bounds.reset();
		for (unsigned int index : rightIndices)
		{
			r->bounds.extend(inputTriangles[index].vertices[0].p);
			r->bounds.extend(inputTriangles[index].vertices[1].p);
			r->bounds.extend(inputTriangles[index].vertices[2].p);
		}
		r->subdivide(inputTriangles, outputTriangles, rightIndices);
	}
	
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// Add BVH building code here
		bounds.reset();
		for (int i = 0; i < inputTriangles.size(); i++)
		{
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}
		std::vector<unsigned int> indices(inputTriangles.size());
		for (unsigned int i = 0; i < inputTriangles.size(); i++)
		{
			indices[i] = i;
		}

		subdivide(inputTriangles, outputTriangles, indices);
		
		
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		float tBox;
		if (!bounds.rayAABB(ray, tBox) ) return;
		if (isLeaf())
		{
			for (unsigned int index : triangleIndices)
			{
				float t, u, v;
				if (triangles[index].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = index;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}
		// non leaf node, traverse child nodes
		float tLeft, tRight;
		bool hitLeft = (l!=NULL)&&l->bounds.rayAABB(ray, tLeft);
		bool hitRight = (r!=NULL)&&r->bounds.rayAABB(ray, tRight);
		if (hitLeft && hitRight)
		{
			// traverse closer child first
			BVHNode* first = (tLeft < tRight) ? l : r;
			BVHNode* second = (first == l) ? r : l;
			first->traverse(ray, triangles, intersection);
			second->traverse(ray, triangles, intersection);
		}else if (hitLeft) l->traverse(ray, triangles, intersection);
		else if (hitRight) r->traverse(ray, triangles, intersection);
		// if no hit, return
		return;
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		float tBox;
		// if ray misses the box or the intersection is farther than maxT, return true
		if (!bounds.rayAABB(ray, tBox) || tBox > maxT) return true;
		if (isLeaf())
		{
			for (unsigned int index : triangleIndices)
			{
				float t, u, v;
				if (triangles[index].rayIntersect(ray, t, u, v))
				{
					if (t < maxT)
					{
						return false;
					}
				}
			}
			return true;
		}
		// non leaf node, traverse child nodes
		float tLeft, tRight;
		bool hitLeft = (l != NULL) && l->bounds.rayAABB(ray, tLeft);
		bool hitRight = (r != NULL) && r->bounds.rayAABB(ray, tRight);
		if (hitLeft && hitRight)
		{
			// traverse closer child first
			BVHNode* first = (tLeft < tRight) ? l : r;
			BVHNode* second = (first == l) ? r : l;
			if (!first->traverseVisible(ray, triangles, maxT)) return false;
			if (!second->traverseVisible(ray, triangles, maxT)) return false;
		}
		 else if (hitLeft && !l->traverseVisible(ray, triangles, maxT)) return false;
		 else if (hitRight && !r->traverseVisible(ray, triangles, maxT)) return false;
		
		return true;
	}
};
