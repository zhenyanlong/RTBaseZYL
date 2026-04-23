#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;

	// tabulated sampling data 
	std::vector<float> marginalCDF;
	std::vector<float> conditionalCDF;
	float envTotalWeight = 0.0f;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		
		{
			int w = env->width;
			int h = env->height;

			marginalCDF.resize(h, 0.0f);
			conditionalCDF.resize(w * h, 0.0f);

			std::vector<float> rowWeights(h, 0.0f);

			// calculate conditional CDF
			for (int j = 0; j < h; ++j)
			{
				
				float v_theta = ((float)j + 0.5f) / (float)h * M_PI;
				float sinTheta = sinf(v_theta);
				float rowSum = 0.0f;

				for (int i = 0; i < w; ++i)
				{
					float lum = env->texels[(j * w) + i].Lum();
					float f_uv = lum * sinTheta; // F[u, v]
					rowSum += f_uv;
					conditionalCDF[j * w + i] = rowSum;
				}
				rowWeights[j] = rowSum;
				envTotalWeight += rowSum;

				
				if (rowSum > 0.0f) {
					for (int i = 0; i < w; ++i) {
						conditionalCDF[j * w + i] /= rowSum;
					}
				}
			}

			// calculate marginal CDF
			float currentMarginal = 0.0f;
			for (int j = 0; j < h; ++j)
			{
				currentMarginal += rowWeights[j];
				if (envTotalWeight > 0.0f) {
					marginalCDF[j] = currentMarginal / envTotalWeight;
				}
				else {
					marginalCDF[j] = (float)(j + 1) / h;
				}
			}
			if (envTotalWeight > 0.0f) {
				marginalCDF[h - 1] = 1.0f;
			}
		}
	}


	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		Vec3 wi = sampleDirectionFromLight(sampler, pdf);
		//pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;

		int w = env->width;
		int h = env->height;
		int ui = std::min((int)(u * w), w - 1);
		int vi = std::min((int)(v * h), h - 1);

		float lum = env->texels[(vi * w) + ui].Lum();
		return (lum * w * h) / (envTotalWeight * 2.0f * M_PI * M_PI);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		int w = env->width;
		int h = env->height;

		float xi_1 = sampler->next();
		float xi_2 = sampler->next();
		float xi_3 = sampler->next();
		float xi_4 = sampler->next();

		// sample v
		auto v_it = std::upper_bound(marginalCDF.begin(), marginalCDF.end(), xi_1);
		int v = std::max(0, std::min((int)std::distance(marginalCDF.begin(), v_it), h - 1));

		// sample u 
		auto row_start = conditionalCDF.begin() + v * w;
		auto row_end = row_start + w;
		auto u_it = std::upper_bound(row_start, row_end, xi_2);
		int u = std::max(0, std::min((int)std::distance(row_start, u_it), w - 1));

		// mapping to [0, 1]
		float u_cont = ((float)u + xi_3) / (float)w;
		float v_cont = ((float)v + xi_4) / (float)h;

		// mapping to spherical coordinates
		float phi = u_cont * 2.0f * M_PI;
		float theta = v_cont * M_PI;

		Vec3 wi(sinf(theta) * cosf(phi), cosf(theta), sinf(theta) * sinf(phi));

		pdf = PDF(ShadingData(), wi);

		return wi;
	}
};