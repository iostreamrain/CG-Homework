// Image header file to define the output. 
#pragma once

#include <Eigen/Core>
#include <vector>

#include "stb_image_write.h"

namespace ComputerGraphicsCourse
{
	class Image
	{
	public:
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Data;
		int Width, Height;

		Image(const int width, const int height)
			: Width(width), Height(height), Data(width*height, Eigen::Vector3d(0, 0, 0))
		{
		}
		~Image()
		{
		}
		void SetColor(const int w, const int h, const Eigen::Vector3d &color)
		{
			Data[Width*h + w] = color;
		}
		inline double clamp(double x) { return x < 0 ? 0 : x>1 ? 1 : x; }
		inline int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }
		inline int SavePng(const char* filename)
		{
			unsigned char* data = new unsigned char[Width*Height * 3];
			for (int i = 0; i < Width*Height; i++)
				for (int j = 0; j < 3; j++) data[3 * i + j] = toInt(Data[i](j));
			int ret = stbi_write_png(filename, Width, Height, 3, data, Width * 3);
			delete[] data;
			return ret;
		}
	};
}

#undef STB_IMAGE_WRITE_IMPLEMENTATION
