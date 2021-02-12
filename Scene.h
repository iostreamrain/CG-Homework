// header file. 

/**
	@file Scene.h
	@version 1.0.1
	@brief \n
	@date  2018/10/11
	@author Takuya Funatomi
	@author Hiroyuki Kubo

	@par Information
	 -# Computer Graphcs 2018
 */

/**
	@mainpage
	Course Page: http://omilab.naist.jp/~funatomi/courses/CG/2018/index.html

	@note Course goals: @n
	Computer Graphics is one of the most important functional element in computer systems. @n
	This class aims to introduce the principles and current trends in computer graphics field. @n
	At the end of the class, the learner will be able to explain the standard pipeline of @n
	computer graphics and build a program of generating computer graphics by himself/herself.@n

	@attention If you need to help, please tell TA and professor.
*/

#pragma once

#include "Image.h"
#include <Eigen/Geometry>
#include <Eigen/LU>

#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

/**
	@namespace ComputerGraphicsCourse
	@brief Computer Graphics Course 2018
*/
namespace ComputerGraphicsCourse
{
	/**
	@brief
	*/
	class Ray
	{
	public:
		Eigen::Vector3d P0; /** @brief Origin */
		Eigen::Vector3d P1; /** @brief Direction */
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		/**
		@brief Ray class constructor
		@param origin Origin of ray
		@param direction Direction of ray
		*/
		Ray(const Eigen::Vector3d &origin, const Eigen::Vector3d &dirction) : P0(origin), P1(dirction) {};
		/**
		@brief Ray class copy constructor
		@attention This constructor is deep copy.@n See also : https://en.wikipedia.org/wiki/Object_copying/
		@param ray Ray class object
		*/
		Ray(const Ray &ray) : P0(ray.P0), P1(ray.P1) {};
	};

	/**
	@brief
	*/
	class Camera
	{
	public:
		Eigen::Vector3d LookFrom;
		Eigen::Vector3d LookAt;
		Eigen::Vector3d Up;
		double	 FoV_Y;
		int		 Width; /** @brief Sensor Resolution */
		int		 Height;/** @brief Sensor Resolution */

		double	 Aspect;
		double	 FoV_X;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Camera() {};
		/**
		@brief Camera class constructor
		@param lookfrom 
		@param lookat 
		@param up 
		@param width Output image width
		@param height Output image height
		@param fov_y field of view (y axis)
		*/
		Camera(const Eigen::Vector3d &lookfrom, const Eigen::Vector3d &lookat, const Eigen::Vector3d &up, const int width, const int height, const double fov_y)
			:LookFrom(lookfrom), LookAt(lookat), Up(up), Width(width), Height(height), FoV_Y(fov_y)
		{
			this->Aspect = (double)width / (double)height;
			this->FoV_X = 2.0 * atan(this->Aspect*tan(this->FoV_Y / 2.0));
		};
	};

	/**
	@brief
	*/
	class Light
	{
	public:
		float position[4];	/* homogeneous coordinate */
		Eigen::Vector3d color;
		Eigen::Vector3d attenuation;
		// for maintaining a state
		static Eigen::Vector3d Attenuation;
		static Eigen::Vector3d Ambient;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Light(float x, float y, float z, float w, const Eigen::Vector3d &c) :color(c), attenuation(Attenuation) {
			position[0] = x;
			position[1] = y;
			position[2] = z;
			position[3] = w;
		};
	};

	/**
	@brief
	*/
	class Material
	{
	public:
		// material for each object
		Eigen::Vector3d diffuse;
		Eigen::Vector3d specular;
		float shininess;
		Eigen::Vector3d emission;
		Eigen::Vector3d ambient;

		// for maintaining a state
		static Eigen::Vector3d Diffuse;
		static Eigen::Vector3d Emission;
		static Eigen::Vector3d Specular;
		static float Shininess;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		/**
		@brief Default constructor
		@note Copy current state for new object
		*/
		Material() :diffuse(Diffuse), specular(Specular), shininess(Shininess), emission(Emission), ambient(Light::Ambient) {};
		Material(const Material &m) :diffuse(m.diffuse), specular(m.specular), shininess(m.shininess), emission(m.emission), ambient(m.ambient) {};
	};

	/**
	@brief
	This class is pure virtual class @n
	You need implementation virtual function
	*/
	class Object
	{
	public:
		Material material;
		Eigen::Matrix4d transform;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Object() {};
		Object(const Object &o) :material(o.material), transform(o.transform) {};
		Object(const Material &_material, const Eigen::Matrix4d &_transform) :material(_material), transform(_transform) {};
		virtual double Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const = 0;
	};

	/**
	@brief
	*/
	class Sphere : public Object
	{
	public:
		Eigen::Vector3d C;		/* Center */
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		double Radius;	/* Radius */
		Sphere(Eigen::Vector3d center, double r) : C(center), Radius(r) {};
		Sphere(const Sphere& s) : C(s.C), Radius(s.Radius), Object(s) {};
		double Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const;
	};

	/**
	@brief
	*/
	class Triangle : public Object
	{
	public:
		Eigen::Vector3d* vertices[3];
		Eigen::Vector3d n;
		Eigen::Matrix3d inv;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Triangle(Eigen::Vector3d* a, Eigen::Vector3d* b, Eigen::Vector3d* c)
		{
			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;

			/* Calculate plane normal for intersection detection*/
			{
				n = (*(vertices[1]) - *(vertices[0])).cross(*(vertices[2]) - *(vertices[0])).normalized();
				Eigen::Vector3d frame[] = {
					(*(vertices[1]) - *(vertices[0])).cross(*(vertices[2]) - *(vertices[0])),
					*(vertices[1]) - *(vertices[0]),
					*(vertices[2]) - *(vertices[0]),
				};
				Eigen::Matrix3d ABC;
				ABC << frame[0], frame[1], frame[2];
				ABC.transpose();
				inv = ABC.inverse();
			}
		};
		Triangle(const Triangle& tr) : Object(tr), n(tr.n), inv(tr.inv)
		{
			for (int n = 0; n < 3; n++)
				vertices[n] = tr.vertices[n];
		};
		virtual double Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const;
	};

#ifdef TRIANGLE_NORMAL
	class VertexNormal
	{
	public:
		Eigen::Vector3d vertex;
		Eigen::Vector3d normal;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexNormal(const Eigen::Vector3d &_v, const Eigen::Vector3d &_n) :vertex(_v), normal(_n.normalized()) {};
	};

	class TriangleNormal : public Object
	{
	public:
		VertexNormal* vertices[3];

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		TriangleNormal(VertexNormal* a, VertexNormal* b, VertexNormal* c)
		{
			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;
		};
		TriangleNormal(const TriangleNormal &tr) : Object(tr) { memcpy(vertices, tr.vertices, sizeof(VertexNormal*) * 3); };
		double Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const;
	};
#endif

	/**
	@brief
	This class holds the elements of scenes such as lights, objects, materials, and normals
	*/
	class Scene
	{
	public:
		std::vector<Light, Eigen::aligned_allocator<Light> > lights;
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vertices;

		std::vector<Sphere, Eigen::aligned_allocator<Sphere> > spheres;
		std::vector<Triangle, Eigen::aligned_allocator<Triangle> > triangles;
#ifdef TRIANGLE_NORMAL
		std::vector<VertexNormal, Eigen::aligned_allocator<VertexNormal> > verticesWithNormal;
		std::vector<TriangleNormal, Eigen::aligned_allocator<TriangleNormal> > triNormals;
#endif

		std::vector<Object*> objList;

		std::vector<Camera, Eigen::aligned_allocator<Camera> > camera;
		int		 Width; /** @brief Sensor Resolution */
		int		 Height;/** @brief Sensor Resolution */
		int		 MaxDepth;
		std::string	output;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		/**
		@brief Default constructor @n
		Width 160, Height 120
		MaxDepth = 5
		output = "raytrace.bmp"
		*/
		Scene() : Width(160), Height(120), MaxDepth(5), output("raytrace.bmp") {};
	};

	/**
	@brief
	ray intersection information class @n
	This class holds the information of ray intersection @n
	*/
	class IntersectionInfo
	{
	public:
		double Distance;
		const Object* HitObject;

		Eigen::Vector3d pos;
		Eigen::Vector3d nor;
		Ray ray;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		IntersectionInfo(const Object* hitObject, const double dist, const Eigen::Vector3d &_pos, const Eigen::Vector3d &_nor, const Ray &_ray)
			: HitObject(hitObject), Distance(dist), pos(_pos), nor(_nor), ray(_ray)
		{
		}
	};

	/**
	@brief
	transform matrix class @n
	*/
	class Transform
	{
	public:
		Transform();
		virtual ~Transform();
		static Eigen::Matrix4d rotate(const float degrees, const Eigen::Vector3d& axis);
		static Eigen::Matrix4d scale(const float &sx, const float &sy, const float &sz);
		static Eigen::Matrix4d translate(const float &tx, const float &ty, const float &tz);
	};

	/**
	@brief
	Makes a background of generated image @n
	Casts a ray at each pixel of image @n
	@return ray trace result image @n
	*/
	Image* RayTrace(const Scene& scene, const Camera& camera);
	/**
	@brief
	Makes Ray object to cast a ray at given pixel of image @n
	@return ray object at the pixel of image coordinate @n @n
	*/
	Ray RayThruPixel(const Camera& camera, const int i, const int j);
	/**
	@brief
	calculates the intersection b/w the ray and the objects in the scene @n
	@return ray intersection information @n
	*/
	IntersectionInfo Intersect(const Ray& ray, const Scene& scene);
	/**
	@brief
	Calculates the position/normal of the obj intersecting with the ray @n
	@return intersection distance @n
	@attention Negative distance is not intersect @n
	*/
	double Intersect(const Ray& ray, const Object* obj, Eigen::Vector3d &position, Eigen::Vector3d &normal);
	/**
	@brief
	Calculates shading according to Light, Material, and IntersectionInfo @n
	@return Color at intersection position @n
	*/
	Eigen::Vector3d FindColor(const Scene& scene, const IntersectionInfo& hit, const int depth);
}