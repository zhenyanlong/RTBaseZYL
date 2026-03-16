#pragma once

#include "Core.h"
#include "Sampling.h"

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

		return false;
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
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
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
		return true;
	}
};
