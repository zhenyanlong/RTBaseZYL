#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		float cosThetai = std::min(std::max(cosTheta, -1.0f), 1.0f);
		bool entering = cosThetai > 0.0f;
		float n1 = entering ? iorExt : iorInt;
		float n2 = entering ? iorInt : iorExt;
		cosThetai = std::abs(cosThetai);

		float eta = n1 / n2;
		float sin2Thetai = 1.0f - cosThetai * cosThetai;
		float sin2Thetat = eta * eta * sin2Thetai;

		if (sin2Thetat >= 1.0f)
		{
			return 1.0f;  
		}

		float cosThetaT = std::sqrt(1.0f - sin2Thetat);
		float Rs_numerator = n1 * cosThetai - n2 * cosThetaT;
		float Rs_denominator = n1 * cosThetai + n2 * cosThetaT;
		float Rs = (Rs_numerator * Rs_numerator) / (Rs_denominator * Rs_denominator);
		float Rp_numerator = n1 * cosThetaT - n2 * cosThetai;
		float Rp_denominator = n1 * cosThetaT + n2 * cosThetai;
		float Rp = (Rp_numerator * Rp_numerator) / (Rp_denominator * Rp_denominator);
		return (Rs + Rp) * 0.5f;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		float cosThetai = std::min(std::max(cosTheta, 0.0f), 1.0f);

		float n1 = 1.0f;
		float cos2Thetai = cosThetai * cosThetai;

		Colour Rs_term1 = ior * (-1.0f) + Colour(n1 * cosThetai, n1 * cosThetai, n1 * cosThetai);
		Colour Rs_numerator = Rs_term1 * Rs_term1 + k * k;
		Colour Rs_term2 = ior + Colour(n1 * cosThetai, n1 * cosThetai, n1 * cosThetai);
		Colour Rs_denominator = Rs_term2 * Rs_term2 + k * k;
		Colour Rs = Rs_numerator / Rs_denominator;

		Colour Rp_term1 = (ior * cosThetai) * (-1.0f) + Colour(n1, n1, n1);
		Colour Rp_numerator = Rp_term1 * Rp_term1 + (k * cosThetai) * (k * cosThetai);
		Colour Rp_term2 = ior * cosThetai + Colour(n1, n1, n1);
		Colour Rp_denominator = Rp_term2 * Rp_term2 + (k * cosThetai) * (k * cosThetai);
		Colour Rp = Rp_numerator / Rp_denominator;

		return (Rs + Rp) * 0.5f;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		float cosTheta = wi.z;
		
		float tanThetaSq = (1 - cosTheta * cosTheta) / (cosTheta * cosTheta);
		float alphaSq = alpha * alpha;
		float sqrtTerm = sqrt(1 + alphaSq * tanThetaSq);
		return (sqrtTerm - 1) / 2;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		if (wi.z <= 0 || wo.z <= 0)
		{
			return 0.0f;
		}
		float Gl_wi = 1/(1+lambdaGGX(wi, alpha));
		float Gl_wo = 1/(1+lambdaGGX(wo, alpha));

		return Gl_wi*Gl_wo;
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		float cosThetaM = h.z;
		if (cosThetaM <= 0) return 0;

		float cosThetaSq = cosThetaM * cosThetaM;
		float alphaSq = alpha * alpha;
		float denom = cosThetaSq * (alphaSq - 1) + 1;

		return alphaSq / (M_PI * denom * denom);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	//*** all "wi" are in world space ***//
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		//Vec3 wi = Vec3(0, 1, 0);
		//pdf = 1.0f;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 local_wi = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		Vec3 world_wi = shadingData.frame.toWorld(local_wi);

		pdf = PDF(shadingData, world_wi);
		reflectedColour = evaluate(shadingData, world_wi);

		return world_wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 local_wi = shadingData.frame.toLocal(wi);
		 if (local_wi.z <= 0.0f)
		 {
			 return 0.0f;
		 }
		 
		return local_wi.z / M_PI;
		
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
		Vec3 world_wi = shadingData.frame.toWorld(local_wi);
		reflectedColour = evaluate(shadingData, world_wi);

		pdf = PDF(shadingData, world_wi);

		return world_wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		/*Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		return albedo->sample(shadingData.tu, shadingData.tv)/ fabs(local_wo.z);*/
		// Schlick's Approximation Improvement

		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		//Vec3 local_wi = shadingData.frame.toLocal(wi);

		//Vec3 expected_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);

		

		float cosTheta = std::abs(local_wo.z);
		Colour F0 = albedo->sample(shadingData.tu, shadingData.tv);

		float temp = 1.0f - cosTheta;
		float schlick = temp * temp;  
		schlick = schlick * schlick;  
		schlick = schlick * temp;     

		Colour fresnel = F0 + (Colour(1.0f, 1.0f, 1.0f) - F0) * schlick;

		return fresnel / std::abs(local_wo.z);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 1.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		/*if (local_wo.z <= EPSILON)
		{
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}*/

		//if (alpha / (1.62142f * 1.62142f) < EPSILON)
		if (alpha < EPSILON)
		{
			// mirror-like case
			Vec3 local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
			Vec3 world_wi = shadingData.frame.toWorld(local_wi);

			float cosTheta = std::abs(local_wo.z);
			Colour F0 = albedo->sample(shadingData.tu, shadingData.tv);

			float temp = 1.0f - cosTheta;
			float schlick = temp * temp;
			schlick = schlick * schlick;
			schlick = schlick * temp;

			Colour fresnel = F0 + (Colour(1.0f, 1.0f, 1.0f) - F0) * schlick;

			reflectedColour = fresnel / std::abs(local_wo.z);

			pdf = 1.0f;

			return world_wi;
		}
		// rough case
		float r1 = sampler->next();
		float r2 = sampler->next();
		float phiM = 2.0f * M_PI * r2;

		float cosThetaM = sqrtf((1.0f - r1) / (1.0f + (alpha * alpha - 1.0f) * r1));
		float sinThetaM = sqrtf(1.0f - cosThetaM * cosThetaM);
		Vec3 wm = Vec3(sinThetaM * cosf(phiM), sinThetaM * sinf(phiM), cosThetaM);
		Vec3 local_wi = wm *Dot(local_wo, wm)* 2.0f - local_wo;

		// G, D, F
		float G = ShadingHelper::Gggx(local_wi, local_wo, alpha);
		float D = ShadingHelper::Dggx(wm, alpha);
		Colour F = ShadingHelper::fresnelConductor(local_wi.dot(wm), eta, k);

		// PDF
		pdf = D * cosThetaM / (4.0f * local_wo.dot(wm));

		// reflected colour
		reflectedColour = (F * G * D) / (4.0f * std::abs(local_wi.z) * std::abs(local_wo.z));

		return shadingData.frame.toWorld(local_wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 local_wi = shadingData.frame.toLocal(wi);
		 if (local_wo.z <= EPSILON || local_wi.z <= EPSILON)
		 {
			 return Colour(0.0f, 0.0f, 0.0f);
		 }
		 Vec3 h = (local_wi + local_wo).normalize();
		 // G
		 float G = ShadingHelper::Gggx(local_wi, local_wo, alpha);
		 // D
		 float D = ShadingHelper::Dggx(h, alpha);
		 // F
		 Colour F = ShadingHelper::fresnelConductor(local_wi.dot(h), eta, k);
		 float denominator = 4 * std::abs(local_wi.z) * std::abs(local_wo.z);
		 if (denominator < EPSILON)
		 {
			 return Colour(1.0f, 1.0f, 1.0f);
		 }
		 return (F * G * D) / denominator;
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		// it is not be used in sampling 
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		
		Vec3 hLocal = (woLocal + wiLocal).normalize();

		float D = ShadingHelper::Dggx(hLocal, alpha);
		float woDotH = Dot(woLocal, hLocal);
		return D * hLocal.z / (4.0f * woDotH);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		// Replace this with Glass sampling code
		//Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);

		//float cosTheta = local_wo.z;  
		//float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);

		//bool entering = cosTheta > 0.0f;
		//float etaI = entering ? extIOR : intIOR;  
		//float etaT = entering ? intIOR : extIOR;  
		//float eta = etaI / etaT; 

		//Vec3 local_wi;
		//float random = sampler->next();

		//// choose between reflection and transmission based on Fresnel term
		//if (random < F)
		//{
		//	local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
		//	Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		//	reflectedColour = baseColor * F / std::abs(local_wo.z);
		//	pdf = 1.0f;
		//}
		//else
		//{
		//	float cosThetaI = std::abs(cosTheta);
		//	float sin2ThetaI = 1.0f - cosThetaI * cosThetaI;
		//	float sin2ThetaT = eta * eta * sin2ThetaI;

		//	if (sin2ThetaT >= 1.0f)
		//	{
		//		local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
		//		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		//		reflectedColour = baseColor / std::abs(local_wo.z);
		//		pdf = 1.0f;
		//	}
		//	else
		//	{
		//		float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
		//		float sign = entering ? -1.0f : 1.0f;

		//		local_wi = Vec3(-eta * local_wo.x, -eta * local_wo.y, sign * cosThetaT);

		//		float etaScale = (etaI * etaI) / (etaT * etaT);
		//		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		//		reflectedColour = baseColor * etaScale * (1.0f - F) / std::abs(cosThetaT);

		//		pdf = 1.0f;
		//	}
		//}
		//Vec3 world_wi = shadingData.frame.toWorld(local_wi);
		//return world_wi;
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = local_wo.z;

		
		float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);

		bool entering = cosTheta > 0.0f;
		float etaI = entering ? extIOR : intIOR;
		float etaT = entering ? intIOR : extIOR;
		float eta = etaI / etaT;

		Vec3 local_wi;
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		if (sampler->next() < F)
		{
			// reflection
			local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
			reflectedColour = baseColor / std::abs(local_wo.z);
			pdf = 1.0f;
		}
		else
		{
			// refraction
			float cosThetaI = std::abs(cosTheta);
			float sin2ThetaI = 1.0f - cosThetaI * cosThetaI;
			float sin2ThetaT = eta * eta * sin2ThetaI;

			
			if (sin2ThetaT >= 1.0f)
			{
				local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
				reflectedColour = baseColor / std::abs(local_wo.z);
				pdf = 1.0f;
			}
			else
			{
				float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
				float sign = entering ? -1.0f : 1.0f;
				local_wi = Vec3(-eta * local_wo.x, -eta * local_wo.y, sign * cosThetaT);
				float etaScale = (etaI * etaI) / (etaT * etaT);
				reflectedColour = baseColor * etaScale / std::abs(cosThetaT);
				pdf = 1.0f;
			}
		}

		return shadingData.frame.toWorld(local_wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		// Replace this with Glass evaluation code
		/*Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 local_wi = shadingData.frame.toLocal(wi);

		Vec3 reflected_dir = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
		float eps = 1e-4f;

		bool isReflection = (std::abs(local_wi.x - reflected_dir.x) < eps &&
			std::abs(local_wi.y - reflected_dir.y) < eps &&
			std::abs(local_wi.z - reflected_dir.z) < eps);

		if (isReflection)
		{
			float cosTheta = local_wo.z;
			float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
			Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
			return baseColor * F / std::abs(local_wo.z);
		}

		bool entering = local_wo.z > 0.0f;
		float etaI = entering ? extIOR : intIOR;
		float etaT = entering ? intIOR : extIOR;
		float eta = etaI / etaT;

		float cosThetaI = std::abs(local_wo.z);
		float sin2ThetaI = 1.0f - cosThetaI * cosThetaI;
		float sin2ThetaT = eta * eta * sin2ThetaI;

		if (sin2ThetaT < 1.0f)
		{
			float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
			float sign = entering ? -1.0f : 1.0f;
			Vec3 refracted_dir = Vec3(-eta * local_wo.x, -eta * local_wo.y, sign * cosThetaT);

			bool isRefraction = (std::abs(local_wi.x - refracted_dir.x) < eps &&
				std::abs(local_wi.y - refracted_dir.y) < eps &&
				std::abs(local_wi.z - refracted_dir.z) < eps);

			if (isRefraction)
			{
				float cosTheta = local_wo.z;
				float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
				float etaScale = (etaI * etaI) / (etaT * etaT);
				Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
				return baseColor * etaScale * (1.0f - F) / std::abs(cosThetaT);
			}
		}

		return Colour(0.0f, 0.0f, 0.0f);*/
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);
		// Replace this with GlassPDF
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		float e = alphaToPhongExponent();
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 local_wr = Vec3(-local_wo.x, -local_wo.y, local_wo.z);
		Vec3 world_wr = shadingData.frame.toWorld(local_wr);
		Frame lobeFrame;
		lobeFrame.fromVector(world_wr);

		float xi1 = sampler->next();
		float xi2 = sampler->next();

		// theta_lobe = acos( xi1 ^ (1 / (e+1)) )
		float cosThetaLobe = std::pow(xi1, 1.0f / (e + 1.0f));
		float sinThetaLobe = std::sqrt(std::max(0.0f, 1.0f - cosThetaLobe * cosThetaLobe));

		// phi_lobe = 2 * PI * xi2
		float phiLobe = 2.0f * M_PI * xi2;
		Vec3 w_lobe = Vec3(
			sinThetaLobe * std::cos(phiLobe),
			sinThetaLobe * std::sin(phiLobe),
			cosThetaLobe
		);

		Vec3 world_wi = lobeFrame.toWorld(w_lobe);
		Vec3 local_wi = shadingData.frame.toLocal(world_wi);
		if (local_wi.z <= 0.0f || local_wo.z <= 0.0f)
		{
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return world_wi;
		}

		pdf = PDF(shadingData, world_wi);
		reflectedColour = evaluate(shadingData, world_wi);

		return world_wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 local_wi = shadingData.frame.toLocal(wi);

		if (local_wo.z <= 0.0f || local_wi.z <= 0.0f)
			return Colour(0.0f, 0.0f, 0.0f);

		float e = alphaToPhongExponent();
		Vec3 local_wr = Vec3(-local_wo.x, -local_wo.y, local_wo.z);

		//  max(0, wr · wi)
		float cosAlpha = std::max(0.0f, Dot(local_wr, local_wi));

		// fr = (e + 2) / (2 * PI) * max(0, wr · wi)^e
		float fp = ((e + 2.0f) / (2.0f * M_PI)) * std::pow(cosAlpha, e);
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		Colour fd = baseColor / M_PI;
		
		float F = ShadingHelper::fresnelDielectric(Dot(local_wr, local_wi), intIOR, extIOR);

		return fd*(1-F) + Colour(1.0f,1.0f,1.0f)* fp * F;
		// support to albedoBuf
		/*return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;*/
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 local_wi = shadingData.frame.toLocal(wi);

		if (local_wo.z <= 0.0f || local_wi.z <= 0.0f)
			return 0.0f;

		float e = alphaToPhongExponent();
		Vec3 local_wr = Vec3(-local_wo.x, -local_wo.y, local_wo.z);

		float cosAlpha = std::max(0.0f, Dot(local_wr, local_wi));

		// p(w) = (e + 1) / (2 * PI) * max(0, wr · wi)^e
		float pdf = ((e + 1.0f) / (2.0f * M_PI)) * std::pow(cosAlpha, e);

		return pdf;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};