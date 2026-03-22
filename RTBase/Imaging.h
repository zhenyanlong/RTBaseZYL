#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class GassianFilter : public ImageFilter
{
	float sizef = 2.0f;
	float alpha = 2.0f;

public:
	virtual float filter(float x, float y) const override
	{
		if (fabsf(x) > sizef && fabs(y) > sizef)
		{
			return 0.0f;
		}
		float d2 = (x * x) + (y * y);
		return std::exp(-alpha * d2) - std::exp(-alpha * sizef * sizef);
	}

	virtual int size() const override
	{
		return static_cast<int>(std::ceil(sizef));
	}
};

class MitchellFilter : public ImageFilter
{
	float B;
	float C;

	float p3, p2, p0; // pre-compute for |x|<1
	float q3, q2, q1, q0; // pre-compute for 1<=|x|<2
public:
	MitchellFilter(float _B = 1.0f / 3.0f, float _C = 1.0f / 3.0f)
	{
		B = _B;
		C = _C;

		p3 = (12.0f - 9.0f * B - 6.0f * C);
		p2 = (-18.0f + 12.0f * B + 6.0f * C);
		p0 = (6.0f - 2.0f * B);
		
		
		q3 = (-B - 6.0f * C);
		q2 = (6.0f * B + 30.0f * C);
		q1 = (-12.0f * B - 48.0f * C);
		q0 = (8.0f * B + 24.0f * C);

	}

	float Mitchell1D(float x) const
	{
		float ax = std::abs(x);
		if (ax < 1.0f)
		{
			return (p3 * ax * ax * ax + p2 * ax * ax + p0) / 6.0f;
		}
		else if (ax < 2.0f)
		{
			return (q3 * ax * ax * ax + q2 * ax * ax + q1 * ax + q0) / 6.0f;
		}
		return 0.0f;
	}

	virtual float filter(float x, float y) const override
	{
		return Mitchell1D(x) * Mitchell1D(y);
	}
	virtual int size() const override
	{
		return 2;
	}
};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;
	void splat(const float x, const float y, const Colour& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		if (total == 0) return;
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		// Return a tonemapped pixel at coordinates x, y
		// Lout= (Lin * 2^e)/(1/2.2)
		/*Colour c = film[y * width + x];
		if (SPP > 0) {
			c = c / (float)SPP;
		}
		float factor2e = std::pow(2.0f, exposure);
		float invGamma = 1.0f / 2.2f;
		r = (unsigned char)std::max(std::min((std::pow(c.r * factor2e, invGamma) * 255.0f),255.0f),0.0f);
		g = (unsigned char)std::max(std::min((std::pow(c.g * factor2e, invGamma) * 255.0f), 255.0f), 0.0f);
		b = (unsigned char)std::max(std::min((std::pow(c.b * factor2e, invGamma) * 255.0f), 255.0f), 0.0f);*/
		
		Colour c = film[y * width + x];
		if (SPP > 0) {
			c = c / (float)SPP;
		}

		//L_exposed = L_in * 2^exposure
		float expScale = pow(2.0f, exposure);
		float rr = c.r * expScale;
		float gg = c.g * expScale;
		float bb = c.b * expScale;

		// L_out = (L_exposed)^(1/2.2)
		float invGamma = 1.0f / 2.2f;
		rr = pow(rr, invGamma);
		gg = pow(gg, invGamma);
		bb = pow(bb, invGamma);

		
		auto toByte = [](float val) {
			float v = val * 255.0f;
			if (v < 0.0f) return (unsigned char)0;
			if (v > 255.0f) return (unsigned char)255;
			return (unsigned char)v;
			};

		r = toByte(rr);
		g = toByte(gg);
		b = toByte(bb);


	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};