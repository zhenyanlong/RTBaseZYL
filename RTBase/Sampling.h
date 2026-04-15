#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float phi = 2.0f * M_PI * r1;
		float theta = acosf(r2);
		
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		if (wi.z > 0)
		{
			return M_1_PI / 2.f;
		}
		return 0.0f;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float phi = 2.0f * M_PI * r1;
		float theta = acosf(sqrtf(r2));
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		
		if (wi.z > 0)
			return wi.z / M_PI;
		return 0.0f;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		return Vec3(0, 0, 1);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f;
	}
};