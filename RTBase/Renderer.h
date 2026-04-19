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
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
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
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		//Colour t_pathThroughput = Colour(1.0f, 1.0f, 1.0f);
		IntersectionData intersection = scene->traverse(r);
		
		if (intersection.t >= FLT_MAX)
		{
			return scene->background->evaluate(r.dir) * pathThroughput;
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
		if (pdf > 1e-4f)
		{
			Ray newRay;
			newRay.init(shadingData.x + (world_wi * EPSILON), world_wi);
			
			pathThroughput = pathThroughput * f * cosTheta / pdf;
			L_indirect = pathTrace(newRay, pathThroughput, depth + 1, sampler);
			//return L_emission + L_direct + L_indirect;
		}


		Colour final_color = L_emission + L_direct + L_indirect;

		
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
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		film->incrementSPP();
		// tile-based rendering
		// generate tiles
		std::vector<Tile> tiles;
		Tile::SplitIntoTiles(film->width, film->height, 128, 128, tiles);

		std::atomic<int> nextTile(0);
		int totalTiles = tiles.size();

		auto TileRenderFunction = [this, &nextTile, &tiles, totalTiles](int threadId) {
			while (true) {
				int tileIndex = nextTile.fetch_add(1);
				if (tileIndex >= totalTiles) break;
				const Tile& tile = tiles[tileIndex];
				
				for (unsigned int y = 0; y < tile.GetTileHeight(); y++)
				{
					for (unsigned int x = 0; x < tile.GetTileWidth(); x++)
					{
						int globalX, globalY;
						tile.ConvertLocalPosToGlobal(x, y, globalX, globalY);
						float px = globalX + samplers[threadId].next();
						float py = globalY + samplers[threadId].next();
						//float px = globalX + 0.5f;
						//float py = globalY + 0.5f;
						Ray ray = scene->camera.generateRay(px, py);
						
						//Colour col = viewNormals(ray);
						//Colour col = albedo(ray);
						Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
						Colour col = pathTrace(ray, pathThroughput, 0, &samplers[threadId]);
						//Colour col = direct(ray, &samplers[threadId]);
						//Colour col = albedo(ray);
						film->splat(px, py, col);
						unsigned char r;
						unsigned char g;
						unsigned char b;
						film->tonemap(globalX, globalY, r, g, b);
						canvas->draw(globalX, globalY, r, g, b);
					}
				}
			}
		};
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(TileRenderFunction, i);
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
			threads[i] = nullptr;
		}
		// single-threaded rendering
		//for (unsigned int y = 0; y < film->height; y++)
		//{
		//	for (unsigned int x = 0; x < film->width; x++)
		//	{
		//		float px = x + samplers[0].next();
		//		float py = y + samplers[0].next();
		//		Ray ray = scene->camera.generateRay(px, py);
		//		
		//		//Colour col = viewNormals(ray);
		//		//Colour col = albedo(ray);
		//		Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
		//		Colour col = pathTrace(ray, pathThroughput, 0, &samplers[0]);
		//		film->splat(px, py, col);
		//		/*unsigned char r = (unsigned char)(col.r * 255);
		//		unsigned char g = (unsigned char)(col.g * 255);
		//		unsigned char b = (unsigned char)(col.b * 255);*/
		//		//film->splat(px, py, col);
		//		unsigned char r;
		//		unsigned char g;
		//		unsigned char b;
		//		film->tonemap(x, y, r, g, b);

		//		canvas->draw(x, y, r, g, b);
		//	}
		//}
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