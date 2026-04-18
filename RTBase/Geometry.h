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
		float r1 = sampler->next();
		float r2 = sampler->next();

		float sqrtr1 = sqrtf(r1);

		float alpha = 1 - sqrtr1;
		float beta = r2 * sqrtr1;
		float gamma = 1 - alpha - beta;

		pdf = 2;

		return vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
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

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	unsigned int offset; // Start index into output triangle list (leaf only)
	unsigned char num;   // Number of triangles in this leaf (0 = internal node)

	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
	}

	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// Step 1: Compute bounds for all input triangles
		bounds.reset();
		for (int i = 0; i < (int)inputTriangles.size(); i++)
		{
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}

		// Step 2: Create leaf if triangle count is within limit
		if ((int)inputTriangles.size() <= MAXNODE_TRIANGLES)
		{
			offset = (unsigned int)outputTriangles.size();
			num = (unsigned char)inputTriangles.size();
			for (int i = 0; i < (int)inputTriangles.size(); i++)
				outputTriangles.push_back(inputTriangles[i]);
			return;
		}

		// Step 3: Binned SAH to find best split axis and bin
		float leafCost = (float)inputTriangles.size() * TRIANGLE_COST;
		float parentArea = bounds.area();
		float bestCost = leafCost;
		int bestAxis = -1;
		int bestBin = -1;

		for (int axis = 0; axis < 3; axis++)
		{
			// Compute centroid range along this axis
			float cMin = FLT_MAX, cMax = -FLT_MAX;
			for (int i = 0; i < (int)inputTriangles.size(); i++)
			{
				Vec3 c = inputTriangles[i].centre();
				float cv = (axis == 0) ? c.x : (axis == 1) ? c.y : c.z;
				if (cv < cMin) cMin = cv;
				if (cv > cMax) cMax = cv;
			}
			if ((cMax - cMin) < 1e-6f) continue;

			float step = (cMax - cMin) / (float)BUILD_BINS;

			// Bin each triangle by its centroid
			struct Bin { AABB bounds; int count; };
			Bin bins[BUILD_BINS];
			for (int b = 0; b < BUILD_BINS; b++) { bins[b].bounds.reset(); bins[b].count = 0; }

			for (int i = 0; i < (int)inputTriangles.size(); i++)
			{
				Vec3 c = inputTriangles[i].centre();
				float cv = (axis == 0) ? c.x : (axis == 1) ? c.y : c.z;
				int b = (int)((cv - cMin) / step);
				if (b >= BUILD_BINS) b = BUILD_BINS - 1;
				bins[b].bounds.extend(inputTriangles[i].vertices[0].p);
				bins[b].bounds.extend(inputTriangles[i].vertices[1].p);
				bins[b].bounds.extend(inputTriangles[i].vertices[2].p);
				bins[b].count++;
			}

			// Prefix sweep: left-side accumulated AABB and counts
			AABB leftAABB[BUILD_BINS - 1];
			int leftCount[BUILD_BINS - 1];
			{
				AABB acc; acc.reset();
				int cnt = 0;
				for (int b = 0; b < BUILD_BINS - 1; b++)
				{
					if (bins[b].count > 0)
					{
						acc.max = Max(acc.max, bins[b].bounds.max);
						acc.min = Min(acc.min, bins[b].bounds.min);
					}
					cnt += bins[b].count;
					leftAABB[b] = acc;
					leftCount[b] = cnt;
				}
			}

			// Suffix sweep: right-side accumulated AABB and counts
			AABB rightAABB[BUILD_BINS - 1];
			int rightCount[BUILD_BINS - 1];
			{
				AABB acc; acc.reset();
				int cnt = 0;
				for (int b = BUILD_BINS - 1; b >= 1; b--)
				{
					if (bins[b].count > 0)
					{
						acc.max = Max(acc.max, bins[b].bounds.max);
						acc.min = Min(acc.min, bins[b].bounds.min);
					}
					cnt += bins[b].count;
					rightAABB[b - 1] = acc;
					rightCount[b - 1] = cnt;
				}
			}

			// Evaluate SAH cost at each candidate split plane
			for (int b = 0; b < BUILD_BINS - 1; b++)
			{
				if (leftCount[b] == 0 || rightCount[b] == 0) continue;
				float lArea = leftAABB[b].area();
				float rArea = rightAABB[b].area();
				float cost = TRAVERSE_COST
					+ (lArea / parentArea) * (float)leftCount[b] * TRIANGLE_COST
					+ (rArea / parentArea) * (float)rightCount[b] * TRIANGLE_COST;
				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestBin = b;
				}
			}
		}

		// Step 4: If no beneficial split found, create a leaf
		if (bestAxis == -1)
		{
			offset = (unsigned int)outputTriangles.size();
			num = (unsigned char)inputTriangles.size();
			for (int i = 0; i < (int)inputTriangles.size(); i++)
				outputTriangles.push_back(inputTriangles[i]);
			return;
		}

		// Step 5: Partition triangles around the best split plane
		float cMin = FLT_MAX, cMax = -FLT_MAX;
		for (int i = 0; i < (int)inputTriangles.size(); i++)
		{
			Vec3 c = inputTriangles[i].centre();
			float cv = (bestAxis == 0) ? c.x : (bestAxis == 1) ? c.y : c.z;
			if (cv < cMin) cMin = cv;
			if (cv > cMax) cMax = cv;
		}
		float step = (cMax - cMin) / (float)BUILD_BINS;
		float splitPos = cMin + (float)(bestBin + 1) * step;

		std::vector<Triangle> leftTris, rightTris;
		for (int i = 0; i < (int)inputTriangles.size(); i++)
		{
			Vec3 c = inputTriangles[i].centre();
			float cv = (bestAxis == 0) ? c.x : (bestAxis == 1) ? c.y : c.z;
			if (cv <= splitPos)
				leftTris.push_back(inputTriangles[i]);
			else
				rightTris.push_back(inputTriangles[i]);
		}

		// Guard against degenerate partitions
		if (leftTris.empty() || rightTris.empty())
		{
			offset = (unsigned int)outputTriangles.size();
			num = (unsigned char)inputTriangles.size();
			for (int i = 0; i < (int)inputTriangles.size(); i++)
				outputTriangles.push_back(inputTriangles[i]);
			return;
		}

		// Step 6: Recurse into children
		l = new BVHNode();
		l->build(leftTris, outputTriangles);
		r = new BVHNode();
		r->build(rightTris, outputTriangles);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		float t;
		if (!bounds.rayAABB(ray, t)) return;
		// Early exit: this node is farther than the best hit found so far
		if (t > intersection.t) return;

		if (num > 0) // Leaf node: test all contained triangles
		{
			for (unsigned int i = offset; i < offset + num; i++)
			{
				float ti, u, v;
				if (triangles[i].rayIntersect(ray, ti, u, v))
				{
					if (ti < intersection.t)
					{
						intersection.t = ti;
						intersection.ID = i;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}

		// Internal node: recurse into children
		if (l) l->traverse(ray, triangles, intersection);
		if (r) r->traverse(ray, triangles, intersection);
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
		float t;
		if (!bounds.rayAABB(ray, t)) return true;
		if (t > maxT) return true;

		if (num > 0) // Leaf node
		{
			for (unsigned int i = offset; i < offset + num; i++)
			{
				float ti, u, v;
				if (triangles[i].rayIntersect(ray, ti, u, v))
				{
					if (ti < maxT)
						return false; // Occluded
				}
			}
			return true;
		}

		// Internal node: recurse, short-circuit on first occlusion found
		if (l && !l->traverseVisible(ray, triangles, maxT)) return false;
		if (r && !r->traverseVisible(ray, triangles, maxT)) return false;
		return true;
	}
};