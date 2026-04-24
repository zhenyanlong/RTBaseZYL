#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <mutex>
#include <OpenImageDenoise/oidn.hpp>

class Tile
{
	int tilex, tiley;
	int width, height;
	int standardWidth, standardHeight; 
public:
	Tile(int _x, int _y, int _width = 128, int _height = 128, int _stdWidth = 128, int _stdHeight = 128) 
		: tilex(_x), tiley(_y), width(_width), height(_height), 
		  standardWidth(_stdWidth), standardHeight(_stdHeight) {}

	int GetTileX() const { return tilex; }
	int GetTileY() const { return tiley; }
	int GetTileWidth() const { return width; }
	int GetTileHeight() const { return height; }

	void GetTileOriPos(int& x, int& y) const
	{
		x = tilex * standardWidth;   
		y = tiley * standardHeight;  
	}

	void ConvertLocalPosToGlobal(int localX, int localY, int& globalX, int& globalY) const
	{
		globalX = tilex * standardWidth + localX;  
		globalY = tiley * standardHeight + localY; 
	}

	static void SplitIntoTiles(int imageWidth, int imageHeight, int tileWidth, int tileHeight, std::vector<Tile>& tiles)
	{
		for (int y = 0; y < imageHeight; y += tileHeight)
		{
			for (int x = 0; x < imageWidth; x += tileWidth)
			{
				int currentTileWidth = std::min(tileWidth, imageWidth - x);
				int currentTileHeight = std::min(tileHeight, imageHeight - y);
				
				tiles.emplace_back(x / tileWidth, y / tileHeight, 
				                   currentTileWidth, currentTileHeight,
				                   tileWidth, tileHeight);
			}
		}
	}
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	std::mutex filmMutex;

	oidn::DeviceRef oidnDevice;
	oidn::BufferRef colorBuf;
	oidn::BufferRef albedoBuf;
	oidn::BufferRef normalBuf;
	oidn::BufferRef outputBuf;
	oidn::FilterRef oidnFilter;
	bool oidnInitialized = false;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		

		// init OIDN
		unsigned int width = (unsigned int)scene->camera.width;
		unsigned int height = (unsigned int)scene->camera.height;
		oidnDevice = oidn::newDevice();
		oidnDevice.commit();
		colorBuf = oidnDevice.newBuffer(width * height * 3 * sizeof(float));
		albedoBuf = oidnDevice.newBuffer(width * height * 3 * sizeof(float));
		normalBuf = oidnDevice.newBuffer(width * height * 3 * sizeof(float));
		outputBuf = oidnDevice.newBuffer(width * height * 3 * sizeof(float));
		oidnFilter = oidnDevice.newFilter("RT");
		oidnFilter.setImage("color", colorBuf, oidn::Format::Float3, width, height);
		oidnFilter.setImage("albedo", albedoBuf, oidn::Format::Float3, width, height);
		oidnFilter.setImage("normal", normalBuf, oidn::Format::Float3, width, height);
		oidnFilter.setImage("output", outputBuf, oidn::Format::Float3, width, height);
		oidnFilter.set("hdr", true);
		//oidnFilter.set("cleanAux", true);
		oidnFilter.commit();

		const char* errorMessage;
		if (oidnDevice.getError(errorMessage) != oidn::Error::None)
			std::cout << "OIDN Init Error: " << errorMessage << std::endl;
		else
			oidnInitialized = false;
		std::cout << "oidnInitialized: " << (oidnInitialized ? "true" : "false") << std::endl;

		film = new Film();
		if (oidnInitialized == true)
			film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter(),&colorBuf,&outputBuf);
		else
			film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter());
		clear();
	}
	void clear()
	{
		film->clear();
		if (oidnInitialized) {
			unsigned int bufSize = film->width * film->height * 3 * sizeof(float);
			memset(colorBuf.getData(), 0, bufSize);
			memset(albedoBuf.getData(), 0, bufSize);
			memset(normalBuf.getData(), 0, bufSize);
			memset(outputBuf.getData(), 0, bufSize);
		}
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		float pmf = 0.0f;
		Light* sampledLight = scene->sampleLight(sampler,pmf);
		float pdf = 0.0f;
		Colour Le;
		Vec3 lightPos = sampledLight->sample(shadingData, sampler, Le, pdf);
		if (sampledLight->isArea())
		{
			// evaluate Geometry term
			Vec3 xtoLight = lightPos - shadingData.x;
			float r = xtoLight.length();
			Vec3 lighttox = -xtoLight;
			Vec3 lightNormal = sampledLight->normal(shadingData, xtoLight.normalize());
			float cosAtSurface = std::max(0.0f, Dot(xtoLight.normalize(), shadingData.sNormal));
			float cosAtLight = std::max(0.0f, Dot(lighttox.normalize(), lightNormal));
			float g_term = (cosAtSurface * cosAtLight) / (r * r);
			// evaluate visibility
			bool visible = scene->visible(shadingData.x, lightPos);
			
			
			// evaluate BSDF
			Colour c = shadingData.bsdf->evaluate(shadingData, xtoLight.normalize());

			return c * g_term* (float)visible * Le / (pdf * pmf);
		}
		else
		{
			// lightPos is direction in environment light
			Vec3 wi = lightPos;
			float cosTheta = std::max(0.0f, Dot(wi, shadingData.sNormal));

			Vec3  envirOrigin = shadingData.x + wi * EPSILON;
			Ray   envirRay;
			envirRay.init(envirOrigin, wi);
			bool  visible = scene->bvh->traverseVisible(
				envirRay, scene->triangles,
				2.0f * use<SceneBounds>().sceneRadius);

			float pdfLight = sampledLight->PDF(shadingData, wi);
			float pdfBSDF = shadingData.bsdf->PDF(shadingData, wi);
			Colour c = shadingData.bsdf->evaluate(shadingData, wi);
			float misWeight = (pdfLight * pdfLight) / (pdfLight * pdfLight + pdfBSDF * pdfBSDF);
			
			return c * cosTheta * (float)visible * Le * misWeight / (pdf * pmf);
		}
		
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		//Colour t_pathThroughput = Colour(1.0f, 1.0f, 1.0f);
		IntersectionData intersection = scene->traverse(r);
		
		if (intersection.t >= FLT_MAX)
		{
			if(depth == 0)
				return scene->background->evaluate(r.dir) * pathThroughput;
			else
				return Colour(0.0f, 0.0f, 0.0f);
		}
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		
		Colour L_emission(0.0f, 0.0f, 0.0f);
		if (shadingData.bsdf->isLight())
		{
			
			//if (depth == 0)
			if(true)
			{
				L_emission = shadingData.bsdf->emit(shadingData, shadingData.wo)* pathThroughput;
				//return L_emission*pathThroughput;
			}
			
		}
		Colour L_direct(0.0f, 0.0f, 0.0f);
		L_direct = computeDirect(shadingData, sampler) * pathThroughput;
		

		float rrProb = 0.9f;
		if (depth >= 5)
		{
			return L_emission + L_direct;
		}
		else if (depth > 2)
		{
			
			if (sampler->next() > rrProb)
			{
				return L_emission + L_direct;
			}
			pathThroughput = pathThroughput / rrProb;
		}


		float r1 = sampler->next();
		float r2 = sampler->next();
		//Vec3 local_wi = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		

		Colour L_indirect(0.0f, 0.0f, 0.0f);
		Colour f;
		float pdf;
		Vec3 world_wi =shadingData.bsdf->sample(shadingData,sampler, f, pdf);
		Vec3 local_wi = shadingData.frame.toLocal(world_wi);
		float cosTheta = std::abs(local_wi.z);
		if (pdf > EPSILON)
		{
			Ray newRay;
			newRay.init(shadingData.x + (world_wi * EPSILON), world_wi);
			
			pathThroughput = pathThroughput * f * cosTheta / pdf;
			IntersectionData nextIntersection = scene->traverse(newRay);
			bool IsSpecular = shadingData.bsdf->isPureSpecular();

			if (nextIntersection.t >= FLT_MAX)
			{
				Colour nextc = scene->background->evaluate(newRay.dir);

				if (IsSpecular)
				{
					L_indirect = pathThroughput * nextc;
				}
				else
				{
					float pdfLight = scene->background->PDF(ShadingData{}, newRay.dir);
					float pmf = 1.0f / (float)scene->lights.size();
					float actualLightPdf = pdfLight * pmf;

					float bsdfPdf2 = pdf * pdf;
					float lightPdf2 = actualLightPdf * actualLightPdf;
					float misWeight = bsdfPdf2 / (bsdfPdf2 + lightPdf2);

					L_indirect = pathThroughput * nextc * misWeight;
				}
			}
			else
			{
				L_indirect = pathTrace(newRay, pathThroughput, depth + 1, sampler);
			}
			//L_indirect = pathTrace(newRay, pathThroughput, depth + 1, sampler);
			//return L_emission + L_direct + L_indirect;

		}


		Colour final_color = L_emission + L_direct + L_indirect;

		// limit max radiance
		float maxRadiance = 10.0f;
		final_color.r = std::min(final_color.r, maxRadiance);
		final_color.g = std::min(final_color.g, maxRadiance);
		final_color.b = std::min(final_color.b, maxRadiance);
		
		return final_color;


		//return L_emission + L_direct + L_indirect;
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 0, 1));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			//return Colour(fabs(shadingData.sNormal.x), fabs(shadingData.sNormal.y), fabs(shadingData.sNormal.z));
			return Colour(shadingData.sNormal.x, shadingData.sNormal.y, shadingData.sNormal.z);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float pixelX, pixelY;
		bool inFrustum = scene->camera.projectOntoCamera(p, pixelX, pixelY);
		if (!inFrustum) return;

		Vec3 cameraOrigin = scene->camera.origin;
		Vec3 viewDir = scene->camera.viewDirection;
		float toCameraLength = (cameraOrigin - p).length();
		Vec3 toCamera = (cameraOrigin - p).normalize(); 
		float cosTheta = Dot(toCamera, -viewDir);      
		if (cosTheta <= 0) return; 

		// Sensor Importance parameter We
		float We = 1.0f / (scene->camera.Afilm * pow(cosTheta, 4));

		bool visible = scene->visible(p, cameraOrigin + toCamera * EPSILON);
		if(!visible) return;

		float cosNormal = Dot(toCamera, n);
		if (cosNormal <= 0) return;
		float geometryTerm = cosNormal / (toCameraLength * toCameraLength);

		Colour finalContribution = col * We * geometryTerm;
		film->splat(pixelX, pixelY, finalContribution);
	}

	void lightTracePath(Ray& r, Colour& pathThroughput, Colour Le, Sampler* sampler)
	{

		const float rrProb = 0.9f;
		float r1 = sampler->next();
		if (r1 > rrProb)
		{
			return; 
		}
		pathThroughput = pathThroughput / rrProb;

		IntersectionData intersection = scene->traverse(r);

		if (intersection.t >= FLT_MAX)
		{
			return;
		}
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		
		Colour f;
		float pdf;
		Vec3 world_wi = shadingData.bsdf->sample(shadingData,sampler,f,pdf);
		Vec3 local_wi = shadingData.frame.toLocal(world_wi);
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);
		if (pdf <= 1e-8f)
			return;

		bool IsTransmission = (local_wi.z * local_wo.z < 0);
		float nonsymmetricFactor = 1.0f;
		if (IsTransmission)
		{
			float up = fabs(Dot(shadingData.wo, shadingData.sNormal))*fabs(Dot(world_wi,shadingData.gNormal));
			float down = fabs(Dot(shadingData.wo, shadingData.gNormal)) * fabs(Dot(world_wi, shadingData.sNormal));
			nonsymmetricFactor = up / down;
		}
		//float N = (float)film->width * (float)film->height;
		Vec3 toCamera = (scene->camera.origin - shadingData.x).normalize();
		Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, toCamera);
		connectToCamera(shadingData.x, shadingData.sNormal, col * nonsymmetricFactor);
		
		float cosTheta = Dot(world_wi, shadingData.sNormal);
		pathThroughput = pathThroughput * f * cosTheta * nonsymmetricFactor / pdf;

		Ray nextRay;
		nextRay.init(shadingData.x + world_wi * EPSILON, world_wi);
		lightTracePath(nextRay, pathThroughput, Le, sampler);

		return;
	}

	void lightTrace(Sampler* sampler)
	{
		// Create a ray starting at p in direction wi
		// create ray
		float pdfLight = 0.0f;
		Light* sampledLight = scene->sampleLight(sampler, pdfLight);
		
		float pdfPosition = 0.0f;
		Vec3 lightPos = sampledLight->samplePositionFromLight(sampler, pdfPosition);

		float pdfDirection = 0.0f;
		Vec3 lightDir = sampledLight->sampleDirectionFromLight(sampler, pdfDirection);

		Vec3 normalLight = sampledLight->normal(ShadingData{}, lightDir);

		
		Colour Le = sampledLight->evaluate(-lightDir);
		float cosLight = std::max(0.0f, Dot(normalLight, lightDir));
		Colour pathThroughput = Le * cosLight / (pdfPosition * pdfLight * pdfDirection);
		connectToCamera(lightPos, normalLight, pathThroughput);

		

		Ray lightRay;
		lightRay.init(lightPos + lightDir * EPSILON, lightDir);

		lightTracePath(lightRay, pathThroughput, Le, sampler);
	}

	void render()
	{
		film->incrementSPP();
		// tile-based rendering
		// generate tiles
		//std::vector<Tile> tiles;
		//Tile::SplitIntoTiles(film->width, film->height, 128, 128, tiles);

		//std::atomic<int> nextTile(0);
		//int totalTiles = tiles.size();

		//auto TileRenderFunction = [this, &nextTile, &tiles, totalTiles](int threadId) {
		//	while (true) {
		//		int tileIndex = nextTile.fetch_add(1);
		//		if (tileIndex >= totalTiles) break;
		//		const Tile& tile = tiles[tileIndex];
		//		
		//		for (unsigned int y = 0; y < tile.GetTileHeight(); y++)
		//		{
		//			for (unsigned int x = 0; x < tile.GetTileWidth(); x++)
		//			{
		//				//int globalX, globalY;
		//				//tile.ConvertLocalPosToGlobal(x, y, globalX, globalY);
		//				///*float r1 = samplers[threadId].next();
		//				//float r2 = samplers[threadId].next();
		//				//r1 = (r1 >= 1.0f) ? 1.0f - 1e4f : r1;
		//				//r2 = (r2 >= 1.0f) ? 1.0f - 1e4f : r2;*/
		//				//float px = globalX + samplers[threadId].next();
		//				//float py = globalY + samplers[threadId].next();
		//				//float normal_px = globalX + 0.5f;
		//				//float normal_py = globalY + 0.5f;
		//				////float px = globalX + 0.5f;
		//				////float py = globalY + 0.5f;
		//				//Ray ray = scene->camera.generateRay(px, py);
		//				//Ray uniform_ray = scene->camera.generateRay(normal_px, normal_py);

		//				//Colour albedo_col = albedo(ray);
		//				//Colour normal_col = viewNormals(ray);
		//				//
		//				//Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
		//				//Colour col = pathTrace(ray, pathThroughput, 0, &samplers[threadId]);
		//				////Colour col = direct(ray, &samplers[threadId]);
		//				////Colour col = albedo(ray);
		//				//film->splat(px, py, col);
		//				///*unsigned char r;
		//				//unsigned char g;
		//				//unsigned char b;
		//				//film->tonemap(globalX, globalY, r, g, b);*/
		//				////canvas->draw(globalX, globalY, r, g, b);
		//				//// fill color buffer
		//				//if (oidnInitialized) {
		//				//	int pixelIndex = globalY * film->width + globalX;
		//				//	int bufIndex = pixelIndex * 3;
		//				//	// albedo buffer
		//				//	float* albedoData = (float*)albedoBuf.getData();
		//				//	albedoData[bufIndex] = albedo_col.r;
		//				//	albedoData[bufIndex + 1] = albedo_col.g;
		//				//	albedoData[bufIndex + 2] = albedo_col.b;
		//				//	// normal buffer
		//				//	float* normalData = (float*)normalBuf.getData();
		//				//	normalData[bufIndex] = normal_col.r;
		//				//	normalData[bufIndex + 1] = normal_col.g;
		//				//	normalData[bufIndex + 2] = normal_col.b;
		//				//	/*float* colorData = (float*)colorBuf.getData();
		//				//	colorData[bufIndex] += col.r;
		//				//	colorData[bufIndex + 1] += col.g;
		//				//	colorData[bufIndex + 2] += col.b;*/
		//				//}

		//				lightTrace(&samplers[threadId]);
		//			}
		//		}
		//	}
		//};
		/*for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(TileRenderFunction, i);
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
			threads[i] = nullptr;
		}*/
		// single-threaded rendering
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				//float px = x + samplers[0].next();
				//float py = y + samplers[0].next();
				//Ray ray = scene->camera.generateRay(px, py);
				//
				////Colour col = viewNormals(ray);
				////Colour col = albedo(ray);
				//Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
				//Colour col = pathTrace(ray, pathThroughput, 0, &samplers[0]);
				//film->splat(px, py, col);
				///*unsigned char r = (unsigned char)(col.r * 255);
				//unsigned char g = (unsigned char)(col.g * 255);
				//unsigned char b = (unsigned char)(col.b * 255);*/
				////film->splat(px, py, col);
				//unsigned char r;
				//unsigned char g;
				//unsigned char b;
				//film->tonemap(x, y, r, g, b);

				//canvas->draw(x, y, r, g, b);

				lightTrace(&samplers[0]);
			}
		}
		// OIDN Denoising
		if (oidnInitialized) {
			/*float* colorData = (float*)colorBuf.getData();
			float invSPP = 1.0f / (float)film->SPP;
			for (unsigned int y = 0; y < film->height; y++) {
				for (unsigned int x = 0; x < film->width; x++) {
					int pixelIndex = y * film->width + x;
					int bufIndex = pixelIndex * 3;
					
					colorData[bufIndex] += film->film[pixelIndex].r;
					colorData[bufIndex + 1] += film->film[pixelIndex].g;
					colorData[bufIndex + 2] += film->film[pixelIndex].b;
				}
			}*/
			
			oidnFilter.execute();
			//float* outputData = (float*)albedoBuf.getData();
			//float* outputData = (float*)outputBuf.getData();

			auto toByte = [](float val) {
				float v = val * 255.0f;
				v = std::min(std::max(v, 0.f), 255.0f);
				return (unsigned char)v;
				};

			for (unsigned int y = 0; y < film->height; y++)
			{
				for (unsigned int x = 0; x < film->width; x++)
				{
					//int index = (y * film->width + x) * 3;
					//float invSPP = 1.0f / (float)film->SPP;
					//float r = outputData[index] * invSPP;
					//float g = outputData[index + 1] * invSPP;
					//float b = outputData[index + 2] * invSPP;
					//// tonemap 
					//// L_exposed = L_in * 2^exposure
					//float expScale = std::pow(2.0f, 1.0f);
					//r *= expScale;
					//g *= expScale;
					//b *= expScale;

					//// Apply filmic tonemapping curve
					//r = film->filmicFunc(r);
					//g = film->filmicFunc(g);
					//b = film->filmicFunc(b);
					//const float W = 11.2f;
					//float denom = 1.0f / film->filmicFunc(W);
					//r = r * denom;
					//g = g * denom;
					//b = b * denom;

					////c.r = std::max(0.0f, std::min(1.0f, c.r));
					////c.g = std::max(0.0f, std::min(1.0f, c.g));
					////c.b = std::max(0.0f, std::min(1.0f, c.b));

					//

					//////L_exposed = L_in * 2^exposure
					////float expScale = pow(2.0f, exposure);
					////rr = rr * expScale;
					////gg = gg * expScale;
					////bb = bb * expScale;

					////// L_out = (L_exposed)^(1/2.2)
					//float invGamma = 1.0f / 2.2f;
					//r = pow(std::max(0.0f, r), invGamma);
					//g = pow(std::max(0.0f, g), invGamma);
					//b = pow(std::max(0.0f, b), invGamma);
					//unsigned char charR = toByte(r);
					//unsigned char charG = toByte(g);
					//unsigned char charB = toByte(b);
					unsigned char r;
					unsigned char g;
					unsigned char b;
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
		else
		{
			for (unsigned int y = 0; y < film->height; y++)
			{
				for (unsigned int x = 0; x < film->width; x++)
				{
					unsigned char r;
					unsigned char g;
					unsigned char b;
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};