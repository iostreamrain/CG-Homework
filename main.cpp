#include <iostream>
#include <fstream>
#include <stack>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include "Image.h"
#include "Scene.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace ComputerGraphicsCourse;

// Function to read the input data values

bool readvals(std::stringstream &s, const int numvals, float* values)
{
	for (int i = 0; i < numvals; i++) {
		s >> values[i];
		if (s.fail()) {
			std::cout << "Failed reading value " << i << " will skip\n";
			return false;
		}
	}
	return true;
}

void readfile(const char* filename, Scene &scene)
{
	std::string str, cmd;
	std::ifstream in;
	in.open(filename);
	if (in.is_open()) {

		// I need to implement a matrix stack to store transforms.  
		// This is done using standard STL Templates 
		std::stack <Eigen::Matrix4d, std::deque<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > > transfstack;
		Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
		transfstack.push(I);  // identity

		getline(in, str);
		while (in) {
			if ((str.find_first_not_of(" \t\r\n") != std::string::npos) && (str[0] != '#')) {
				// Ruled out comment and blank lines 

				std::stringstream s(str);
				s >> cmd;
				float values[10]; // Position and color for light, colors for others
				// Up to 10 params for cameras.  
				bool validinput; // Validity of input 

				// size Command
				if (cmd == "size") {
					validinput = readvals(s, 2, values);
					if (validinput) {
						scene.Width = (int)values[0];
						scene.Height = (int)values[1];
					}
				}
				else if (cmd == "maxdepth") {
					validinput = readvals(s, 1, values);
					if (validinput) {
						scene.MaxDepth = (int)values[0];
					}
				}
				else if (cmd == "output") {
					s >> scene.output;
				}
				else if (cmd == "camera") {
					validinput = readvals(s, 10, values); // 10 values eye cen up fov
					if (validinput) {
						scene.camera;
						Camera camera(Eigen::Vector3d(values[0], values[1], values[2]), Eigen::Vector3d(values[3], values[4], values[5]), Eigen::Vector3d(values[6], values[7], values[8]), scene.Width, scene.Height, M_PI*values[9] / 180);
						scene.camera.push_back(camera);
					}
				}

				// for loading objects
				else if (cmd == "sphere") {
					validinput = readvals(s, 4, values);
					if (validinput) {
						Sphere s(Eigen::Vector3d(values[0], values[1], values[2]), values[3]);
						s.transform = transfstack.top();
						scene.spheres.push_back(s);
					}
				}
				else if (cmd == "maxverts") {
					// do nothing
				}
				else if (cmd == "maxvertnorms") {
					// do nothing
				}
				else if (cmd == "vertex" || cmd == "v") {
					validinput = readvals(s, 3, values);
					if (validinput) {
						scene.vertices.push_back(Eigen::Vector3d(values[0], values[1], values[2]));
					}
				}
				else if (cmd == "tri" || cmd == "vt") {
					validinput = readvals(s, 3, values);
					if (validinput) {
						Triangle tri(&scene.vertices[(int)values[0]], &scene.vertices[(int)values[1]], &scene.vertices[(int)values[2]]);
						tri.transform = transfstack.top();
						scene.triangles.push_back(tri);
					}
				}
#ifdef TRIANGLE_NORMAL
				else if (cmd == "vertexnormal") {
					validinput = readvals(s, 6, values);
					if (validinput) {
						scene.verticesWithNormal.push_back(
							VertexNormal(Eigen::Vector3d(values[0], values[1], values[2]), Eigen::Vector3d(values[3], values[4], values[5]))
							);
					}
				}
				else if (cmd == "trinormal") {
					validinput = readvals(s, 3, values);
					if (validinput) {
						TriangleNormal tri(&scene.verticesWithNormal[(int)values[0]], &scene.verticesWithNormal[(int)values[1]], &scene.verticesWithNormal[(int)values[2]]);
						tri.transform = transfstack.top();
						scene.triNormals.push_back(tri);
					}
				}
#endif
				// for transformations
				else if (cmd == "translate") {
					validinput = readvals(s, 3, values);
					if (validinput) {
						transfstack.top() = transfstack.top() * Transform::translate(values[0], values[1], values[2]);
					}
				}
				else if (cmd == "scale") {
					validinput = readvals(s, 3, values);
					if (validinput) {
						transfstack.top() = transfstack.top() * Transform::scale(values[0], values[1], values[2]);
					}
				}
				else if (cmd == "rotate") {
					validinput = readvals(s, 4, values);
					if (validinput) {
						transfstack.top() = transfstack.top() * Transform::rotate(values[3], Eigen::Vector3d(values[0], values[1], values[2]));
					}
				}

				// I include the basic push/pop code for matrix stacks
				else if (cmd == "pushTransform") {
					transfstack.push(transfstack.top());
				}
				else if (cmd == "popTransform") {
					if (transfstack.size() <= 1) {
						std::cerr << "Stack has no elements.  Cannot Pop\n";
					}
					else {
						transfstack.pop();
					}
				}

				// Process the light, add it to database.
				// Lighting Command
				else if (cmd == "directional") {
					validinput = readvals(s, 6, values);
					if (validinput) {
						Light l(values[0], values[1], values[2], 0, Eigen::Vector3d(values[3], values[4], values[5]));
						scene.lights.push_back(l);
					}
				}
				else if (cmd == "point") {
					validinput = readvals(s, 6, values);
					if (validinput) {
						Light l(values[0], values[1], values[2], 1, Eigen::Vector3d(values[3], values[4], values[5]));
						scene.lights.push_back(l);
					}
				}
				else if (cmd == "attenuation") {
					validinput = readvals(s, 3, values);
					if (validinput)
						Light::Attenuation = Eigen::Vector3d(values[0], values[1], values[2]);
				}
				else if (cmd == "ambient") {
					validinput = readvals(s, 3, values);
					if (validinput)
						Light::Ambient = Eigen::Vector3d(values[0], values[1], values[2]);
				}

				// Material Commands 
				else if (cmd == "diffuse") {
					validinput = readvals(s, 3, values);
					if (validinput)
						Material::Diffuse = Eigen::Vector3d(values[0], values[1], values[2]);
				}
				else if (cmd == "specular") {
					validinput = readvals(s, 3, values);
					if (validinput)
						Material::Specular = Eigen::Vector3d(values[0], values[1], values[2]);
				}
				else if (cmd == "emission") {
					validinput = readvals(s, 3, values);
					if (validinput)
						Material::Emission = Eigen::Vector3d(values[0], values[1], values[2]);
				}
				else if (cmd == "shininess") {
					validinput = readvals(s, 1, values);
					if (validinput) {
						Material::Shininess = values[0];
					}
				}

				else {
					std::cerr << "Unknown Command: " << cmd << " Skipping \n";
				}
			}
			getline(in, str);
		}
	}
	else {
		std::cerr << "Unable to Open Input Data File " << filename << "\n";
		throw 2;
	}


	// construct object list
	for (std::vector<Sphere, Eigen::aligned_allocator<Sphere> >::iterator itr = scene.spheres.begin(); itr < scene.spheres.end(); itr++)
		scene.objList.push_back(&(*itr));
	for (std::vector<Triangle, Eigen::aligned_allocator<Triangle> >::iterator itr = scene.triangles.begin(); itr < scene.triangles.end(); itr++)
		scene.objList.push_back(&(*itr));
#ifdef TRIANGLE_NORMAL
	for (std::vector<TriangleNormal, Eigen::aligned_allocator<TriangleNormal> >::iterator itr = scene.triNormals.begin(); itr < scene.triNormals.end(); itr++)
		scene.objList.push_back(&(*itr));
#endif

	return;
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		std::cerr << argv[0] << " [scene file]" << std::endl;
		std::exit(-1);
	}

	Scene scene;
	readfile(argv[1], scene);

	for (unsigned int n = 0; n < scene.camera.size(); n++) {
		/* format output filepath */
		std::string filepath(scene.output);

		if (scene.camera.size() > 1) {
			char num[1024];	sprintf(num, "-%04d.", n + 1);
			filepath.replace(filepath.rfind("."), 1, num);
		}

		/* Do the ray tracing */
		Image* img = RayTrace(scene, scene.camera[n]);

		/* Save image */
		//img->SavePng(scene.output.c_str());
		if (img->SavePng(filepath.c_str()))
			std::cout << "\t Successfully output image file. : " << filepath.c_str() << std::endl;

		delete img;
	}

	return 0;
}

#undef _USE_MATH_DEFINES
