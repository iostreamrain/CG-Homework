// Scene.cpp: implementation.

#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846 // pi
#endif

#include "Scene.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>

namespace ComputerGraphicsCourse
{
	Image* RayTrace(const Scene& scene, const Camera& camera)
	{
		const int width = camera.Width, height = camera.Height;
		Image* image = new Image(width, height);
		int count = 0;
				int num = width;

		for (int j = 0; j < height ; j++)
		{
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < num ; i++)
			{
				// YOUR CODE FOR ASSIGNMENT 0 HERE.
			double k0 = ( cos( M_PI*j  /camera.Height) + j % 2) * camera.Width;
				double k[3], netx = 0;
				k[0] = double(i) / camera.Width + 0.01;
				k[1] = double(j) / camera.Height - 0.01;
				if (int(k0 - i) % int(camera.Width / 1 + 1)  < 1) {
					netx = -k[0] + 0.50;
				}
				k[0] += netx;
				image->Data[width*j + i] = Eigen::Vector3d(k[0], k[1], 1); /* Background Color */

				
				
				Ray ray = RayThruPixel(camera, i, j);
				IntersectionInfo hit = Intersect(ray, scene);
				if (hit.HitObject != NULL) {
					image->Data[width*j + i] = FindColor(scene, hit, scene.MaxDepth);
				}
			}
			std::cout << "\rRender: " << std::setprecision(2) << std::fixed << (100.0*(count++)) / (height - 1) << "%" << std::flush;
		}
		return image;
	}

	Ray RayThruPixel(const Camera& camera, const int i, const int j)
    {
        // YOUR CODE FOR ASSIGNMENT 2 HERE.  
        Eigen::Vector3d origin = camera.LookFrom;
        Eigen::Vector3d w, v, u, direction;
        w = (camera.LookFrom - camera.LookAt).normalized();
		u = -w.cross(camera.Up).normalized();
		v = u.cross(-w);

		double alpha, beta;
		alpha = tan(camera.FoV_X / 2.) * (i - (camera.Width / 2.)) / (camera.Width / 2.);
		beta = tan(camera.FoV_Y / 2.) * ((camera.Height / 2.) - j) / (camera.Height / 2.);
		Eigen::Vector3d point = alpha * u + beta * v - w;
		direction = point / point.norm();
        
        return Ray(origin, direction);
    }

	IntersectionInfo Intersect(const Ray& ray, const Scene& scene)
	{
		double mindist = 1e8;
		Object* hitobject = NULL;
		Eigen::Vector3d hitpos(0,0,0);	// hit position
		Eigen::Vector3d normal(0, 0, 0);	// hit position normal
		for (std::vector<Object*>::const_iterator o = scene.objList.begin(); o < scene.objList.end(); o++) // find closest intersection; test all objects
		{
			Eigen::Vector3d p, n;
			double t = Intersect(ray, *o, p, n);
			if (t > 0 && t < mindist) // closer than previous closest object
			{
				mindist = t;
				hitobject = *o;
				hitpos = p;
				normal = n;
			}
		}

		return IntersectionInfo(hitobject, mindist, hitpos, normal, ray);	// may already be in Intersect()
	}


	double Intersect(const Ray& ray, const Object* obj, Eigen::Vector3d &position, Eigen::Vector3d &normal)
	{
		Eigen::Vector4d p0(ray.P0[0], ray.P0[1], ray.P0[2], 1), p1(ray.P1[0], ray.P1[1], ray.P1[2], 0);
		// invert transform
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		Eigen::Matrix4d M = obj->transform;
		p0 = M.inverse() * p0;
		p1 = M.inverse() * p1;
		Ray transformedRay(p0.block<3, 1>(0, 0), p1.block<3, 1>(0, 0));
		double t = obj->Intersect(transformedRay, position, normal);

		if (t < 0) return t;

		// transform the results
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		Eigen::Vector4d pos(position[0], position[1], position[2], 1), 
			norm(normal[0], normal[1], normal[2], 0);
		
		// pos = M*pos, norm = (M-1).transpose() * norm (covariant vector)
		pos = M * pos;
		norm = M.inverse().transpose() * norm;
		// set pos and norm into position and normal, respectively.
		for (int i = 0; i < 3; i++)
		{
			position[i] = pos[i];
			normal[i] = norm[i];
		}
		return t;
	}


	double Triangle::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
	{
		// YOUR CODE FOR ASSIGNMENT 1 HERE.
		
		/*TODO: Implement ray-triangle intersection. */

     double distance, v, w;
	 double a,b;
	 a = ray.P0.dot(n);
	 b = ray.P1.dot(n);
     distance = ((*vertices[0]).dot(n) - a) / b; 
     Eigen::Vector3d Position = ray.P0 + distance * ray.P1; 

     Eigen::Vector3d uvw = inv*(Position - (*vertices[0]));
     v = uvw[1];
     w = uvw[2];

     position = Position;
     normal = n;

     if (v >= 0 && w>= 0 && v + w <= 1) {
      return distance;
     }
     else
      return -1;
    }
		
	


	double Sphere::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
	{
		// YOUR CODE FOR ASSIGNMENT 2 HERE.  
		/*TODO: Implement ray-sphere intersection. */
		/* return positive distance from ray origin to intersection */
		/* return -1, if no sphere intersects */
		double distance, a, b, c, root;
		a = ray.P1.dot(ray.P1);
		b = 2 * ray.P1.dot(ray.P0 - C);
		c = (ray.P0 - C).dot(ray.P0 - C) - Radius * Radius;

		/* solve the eq. : at^2 + bt + c = 0 */
		double del = b * b - 4 * a * c;
		double root1 = (-b + sqrt(del)) / (2*a);
		double root2 = (-b - sqrt(del)) / (2*a);

		/* Complex roots: no intersection */
		if(del < 0){
			distance = -1;
		}
		/* 2 roots found */
		else{
			root = root1;
		/* 2 real positive roots: pick smaller root */
			if(root1 > root2){
				root = root2;
			}
		/* One positive, one negative root: ray origin inside sphere (pick + root) */
			if(root1 * root2 < 0){
				root = fmax(root1, root2);
			}
		/* Both negative, no intersection */
			if( root1 < 0 && root2 < 0){
				distance = -1;
			}
			distance = root;
		}
		position = distance*ray.P1 + ray.P0;
		normal = (position - C) / (position - C).norm();
	
		return distance;
	}

#ifdef TRIANGLE_NORMAL
	double TriangleNormal::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
	{
		/*TODO: Implement ray-triangle intersection. */
		// 1. Ray-triangle intersection

		// 2. interpolate normal by coefficient


		return -1;
	}
#endif

	Eigen::Vector3d FindColor(const Scene& scene, const IntersectionInfo& hit, const int depth)
	{
		if (hit.HitObject == NULL) return Eigen::Vector3d(0, 0, 0);
		Material mtrl(hit.HitObject->material);
		Eigen::Vector3d color = mtrl.ambient + mtrl.emission;

		/* Ignore Ka, Ke, and Ks terms from the shading equation on pp.7 */
		Eigen::Vector3d p = hit.pos;
		Eigen::Vector3d n = hit.nor.normalized();

		// YOUR CODE FOR ASSIGNMENT 3 HERE.  

		// Execute the following processes for each light
		for (int i = 0; i < int(scene.lights.size()); i++)
		{
			Light light = scene.lights[i];
			Eigen::Vector3d LightPosition;
			LightPosition[0] = light.position[0];
			LightPosition[1] = light.position[1];
			LightPosition[2] = light.position[2];
			Eigen::Vector3d intensity = light.color;
			Eigen::Vector3d dir;

			// 1. calculate direction and intensity of incoming light
			//Point light
			if (light.position[3] == 1)
			{
				dir = LightPosition - p;
				//Intensity with attenuation
				intensity /= light.attenuation[0] + light.attenuation[1] * dir.norm() + light.attenuation[2] * pow(dir.norm(),2);
			}
			else
			{
				dir = - LightPosition;
			}
			// 2. check if the light is visible from the hit point or not
			Ray ShadowRay(p + 0.00001 * dir.normalized(), dir.normalized());
			IntersectionInfo sHit = Intersect(ShadowRay, scene);

			if (sHit.HitObject != NULL && sHit.Distance < dir.norm() / light.position[3]) {
				// if the shadow ray hits an object, skip the light
				continue;
			}
			// 3. calculate diffuse and specular shading 
			Eigen::Vector3d L = ShadowRay.P1.normalized();
			Eigen::Vector3d H = (dir.normalized() - hit.ray.P1).normalized();

			/* Diffuse color(RGB) * light color(RGB) * reflection term(scalar)*/
			color += mtrl.diffuse.cwiseProduct(intensity) * fmax(L.dot(n),0);
			
			/* specular color(RGB) * light color(RGB) * reflection term(scalar)*/
		
			color += mtrl.specular.cwiseProduct(intensity) * pow(fmax(H.dot(n), 0),mtrl.shininess);
		}
		// 4. *option* calculate recursive specular reflection
		if (depth > 0) {
            // Eigen::Vector3d ReflectedP1 = hit.ray.P1 - 2 * (hit.ray.P1.dot(n) / n.dot(n)) * n;
            // Ray RayReflect(p + 1e-5 * ReflectedP1, ReflectedP1);
            // IntersectionInfo NewHit = Intersect(RayReflect, scene);
            // if (NewHit.HitObject == hit.HitObject || NewHit.HitObject == NULL) {
            //     return color;
            // }
            // Eigen::Vector3d ReflectColor = FindColor(scene, NewHit, depth - 1);
            // return color + Eigen::Vector3d(NewHit.HitObject->material.specular(0) * ReflectColor(0),
            //                                NewHit.HitObject->material.specular(1) * ReflectColor(1),
            //                                NewHit.HitObject->material.specular(2) * ReflectColor(2));
        }
		return color;
	}


	// Helper rotation function.  Please implement this.  
	Eigen::Matrix4d Transform::rotate(const float degrees, const Eigen::Vector3d& axis)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
		Eigen::Matrix3d R;

		double rad = degrees * M_PI / 180.0;
		// YOUR CODE FOR ASSIGNMENT 4 HERE. 
		A(0, 1) = -axis(2);
		A(0, 2) = axis(1);
		A(1, 0) = axis(2);
		A(1, 2) = -axis(0);
		A(2, 0) = -axis(1);
		A(2, 1) = axis(0);

		R = cos(rad)*Eigen::Matrix3d::Identity() + (1 - cos(rad))*axis*axis.transpose() + sin(rad)*A;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				ret(i, j) = R(i, j);
		}

		return ret;
	}

	Eigen::Matrix4d Transform::scale(const float &sx, const float &sy, const float &sz)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		ret(0, 0) = sx;
		ret(1, 1) = sy;
		ret(2, 2) = sz;
		return ret;
	}

	Eigen::Matrix4d Transform::translate(const float &tx, const float &ty, const float &tz)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		ret(0, 3) = tx;
		ret(1, 3) = ty;
		ret(2, 3) = tz;
		return ret;
	}

	Eigen::Vector3d Light::Ambient = Eigen::Vector3d(0.2, 0.2, 0.2);
	Eigen::Vector3d Light::Attenuation = Eigen::Vector3d(1., 0., 0.);

	Eigen::Vector3d Material::Diffuse = Eigen::Vector3d(0., 0., 0.);
	Eigen::Vector3d Material::Emission = Eigen::Vector3d(0., 0., 0.);
	Eigen::Vector3d Material::Specular = Eigen::Vector3d(0., 0., 0.);
	float Material::Shininess = 0.f;

}