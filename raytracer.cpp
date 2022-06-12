#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cmath> 

using namespace std::chrono;
using namespace cv;
using namespace std;
using namespace Eigen;

//Global variables
float PI = 3.14159265358979323846;
int POINT_LIGHT = 0;
int SPOT_LIGHT = 1;
int IMAGE_RES_X = 1024;
int IMAGE_RES_Y = 1024;
int MAX_REFLECTION_DEPTH = 3;

//global variable for debug
int rays = 0;

//using floating point close-to-equals function  from https://stackoverflow.com/questions/19837576/comparing-floating-point-number-to-zero
 bool isCloseToEqual(float x, float y)
{
	 const float epsilon = 0.00001;
	return std::abs(x - y) <= epsilon * std::abs(x);
	// see Knuth section 4.2.2 pages 217-218
}

Point3f normalize(Point3f vector)
{
	float length = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
	return Point3f(vector.x / length, vector.y / length, vector.z / length);
}

Point3f point_plus_vector(Point3f a, Vector3f b)
{
	return Point3f(a.x + b[0], a.y + b[1], a.z + b[2]);
}

Vec3f p2v(Point3f a)
{
	return Vec3f(a.x, a.y, a.z);
}

float fastDet(float a_, float b_, float c_, float d_, float e_, float f_, float g_, float h_, float i_)
{
	return a_ * e_ * i_ + b_ * f_ * g_ + c_ * d_ * h_ - c_ * e_ * g_ - b_ * d_ * i_ - a_ * f_ * h_;

}

//DEBUG values
//int smallerZeroNum = 0;
//int normRefractionNum = 0;

Vec3f computeRefractionRay(float indexRefraction, float indexRefraction2, Vec3f viewVecOrig, Vec3f normal)
{

	float indxRef = indexRefraction / indexRefraction2;
	Vec3f viewVec = normalize(viewVecOrig);
	float S_N = normal.dot(viewVec);

	float sqrtTerm = 1 - pow(indxRef, 2) * (1 - pow(S_N, 2));
	Vec3f transmissionRayDir;
	//Test for total internal reflection
	if (sqrtTerm < 0)
	{
		//If total internal reflection will occur, set transmission ray as the inverse of the view vector
		transmissionRayDir = -viewVec;
		//smallerZeroNum++;
	}
	else
	{
		//else, continue with the transmission ray computation
		transmissionRayDir = (indxRef * (S_N)-sqrt(sqrtTerm)) * normal - indxRef * viewVec;
		//normRefractionNum++;
	}
	return transmissionRayDir;

}

struct Ray {
	Point3f o; //origin
	Point3f d; //direction
};

struct Tris {
	Point3f vtx1;
	Point3f vtx2;
	Point3f vtx3;
	Point3f faceTopology; //To keep track of the topology for normal calculation in TriMesh.testIntersect
};


struct Texture {
	int type; //1 - 2D, >2 - 3D and 3D type
	string path;
	Mat image;
};

class ShadeInfo {
	//Test for shadows
	//Compute Diffuse color
	//possible recurse for specular and translucency

public:
	ShadeInfo(); 
	ShadeInfo(Vec3f ambientC, Vec3f diffuseC, Vec3f specularC, int shineC, int reflections, int refractions, int iof);

	//Internal variables for computation:
	bool hit_an_object;			//if true, ray has hit an object
	Point3f local_hit_point;	//the hit point in world coords
	Vec3f normal;				//the normal vector at hit point
	Vec3b color;			//storing color for Assignment1

	//User-specified Variables:
	Vec3f specularColor;
	Vec3f diffuseColor;
	Vec3f ambientColor;
	int shininess;
	int reflectiveness;
	int refractiveness;
	float indexoOfRefraction;
	
};

ShadeInfo::ShadeInfo()
{
	hit_an_object = false;
	color = Vec3b(0, 0, 0);
	specularColor = Vec3b(0.8, 0.8, 0.8);
	diffuseColor = Vec3b(0.6,0.6,0.6);
	ambientColor = Vec3b(0.3, 0.3, 0.3);
	shininess = 80;
	reflectiveness = 0;
	refractiveness = 0;
	indexoOfRefraction = 1;
}

ShadeInfo::ShadeInfo(Vec3f ambientC, Vec3f diffuseC, Vec3f specularC, int shineC, int reflections, int refractions, int iof)
{
	hit_an_object = false;
	color = Vec3b(0, 0, 0);
	specularColor = specularC;
	diffuseColor = diffuseC;
	ambientColor = ambientC;
	shininess = shineC;
	reflectiveness = reflections;
	refractiveness = refractions;
	indexoOfRefraction = iof;
}

class Light {
public:
	Light(int type, Point3f o, Vec3f i, float attenuation);
	Light(int t, Point3f o, Vec3f i, float attenuation, Vec3f d);

	int type = 0; //0 - Point Light
	Point3f origin = Point3f(0, 0, 0);
	Vec3f colorIntensity = Vec3f(255, 255, 255);
	float att = 3;
	Vec3f direction;
	float coneAngle = PI / 4; //Leaving this as static - doesn't work well with the spotlight algorithm i have in computeRayColor
	float fallOffParam = 1; //Leaving this as static - doesn't work well with the spotlight algorithm i have in computeRayColor

};


Light::Light(int t, Point3f o, Vec3f i, float attenuation)
{
	type = t;
	origin = o;
	colorIntensity = i;
	att = attenuation;
}

Light::Light(int t, Point3f o, Vec3f i, float attenuation, Vec3f d)
{
	type = t;
	origin = o;
	colorIntensity = i;
	att = attenuation;
	if (t == 1)
	{
		direction = d;
	}
	else
	{
		direction = Vec3f(0, 0, 0);
	}
}

class Entity {
public:
	virtual bool testIntersect(const Ray& ray, float& t_min, ShadeInfo& si) {
		return false;
	};
	virtual bool fullLightTransmission() {
		return false;
	}
	virtual void setFullLightTransmission(bool val) {
		return;
	}
	virtual float getRadiusIfExist() {
		return -1;
	}
	virtual Point3f getOrigin() {
		Point3f s;
		return s;
	}
	virtual void setTexture(Texture tx) {
		return;
	}
	virtual Texture getTexture() {
		Texture s;
		s.type = 0;
		return s;
	}
	//Add getPointer to Material here?
};



class Cylinder : public Entity {
public:
	Cylinder(); //Create sphere with radius 1 at origin with no transformations
	Cylinder(float r, float z, Point3f o);
	Cylinder(float r, float z, Point3f o, Matrix4f tMtx);
	float getRadiusIfExist();
	bool testIntersect(const Ray& ray, float& t_min, ShadeInfo& si);
	bool fullLightTransmission();
	void setFullLightTransmission(bool val);
	void setTexture(Texture tx) {
		return;
	}
	Point3f getOrigin() {
		return origin;
	}
	
	Texture getTexture() {
		Texture s;
		s.type = 0;
		return s;
	}
private:
	float z_limit;
	float radius;
	Point3f origin;
	Matrix4f transformMtx; //This should be moved to become an ellipsoid as it can be scaled and rotated
	vector<Point3f> normal;
	static const double kEpsilon; //This is to provide the foundation for shadows
	bool fullLightTransmissionAllowed;
};

Cylinder::Cylinder()
{
	radius = 1;
	z_limit = 1;
	origin = Point3f(0, 0, 0);
	transformMtx << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Identity Mtx
}

Cylinder::Cylinder(float r, float z, Point3f o)
{
	z_limit = z;
	radius = r;
	origin = o;
	transformMtx << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //Identity Mtx
}

Cylinder::Cylinder(float r, float z, Point3f o, Matrix4f tMtx)
{
	z_limit = z;
	radius = r;
	origin = o;
	transformMtx = tMtx;
}

float Cylinder::getRadiusIfExist()
{
	return radius;
}

bool Cylinder::fullLightTransmission()
{
	return fullLightTransmissionAllowed;
}

void Cylinder::setFullLightTransmission(bool val)
{
	fullLightTransmissionAllowed = val;
}

bool Cylinder::testIntersect(const Ray& ray, float& t_min, ShadeInfo& si)
{
	//Plug ray equation into cylinder equation and we get a quadratic equation of the form At^2 + Bt + C = 0
	float a = ray.d.x * ray.d.x + ray.d.y * ray.d.y;
	float b = 2 * (ray.o.x - origin.x) * ray.d.x + 2 * (ray.o.y - origin.y) * ray.d.y;
	float c = (ray.o.x - origin.x) * (ray.o.x - origin.x) + (ray.o.y - origin.y) * (ray.o.y - origin.y);
	c = c - radius * radius; 

	//solve for t using the quadratic equation
	float determinant = b * b - 4 * a * c;
	//Note: if det < 0: no interesection. If det = 0: there is 1
	if (determinant < 0)
	{
		si.hit_an_object = false;
		si.color = Vec3b{ 0,0,0 }; //Background color
		return false;
	}
	float t_0 = (-b - sqrt(determinant)) / 2;
	float t_1 = (-b + sqrt(determinant)) / 2;
	
	float z_min, z_max;
	bool capMinX = false; //There exists a cap Zmin intersection
	bool capMaxX = false; //There exists a cap Zmax intersection
	bool openCylinderX = true; //t1 or t2 (non-cap cylinder x's) won the battle against t3 and t4 (cap t values)


	//Test1: check that t_0 and t_1 are between z_min and z_max
	Point3f z_0 = ray.o + t_0 * ray.d;
	Point3f z_1 = ray.o + t_1 * ray.d;

	z_min = origin.z - z_limit;
	z_max = origin.z + z_limit;

	if ((z_min >= z_0.z) || (z_max <= z_0.z))
	{
		t_0 = -1; //t_0 intersection is outside of the cylinder z limit - the next step will elimnate it b/c it is now < 0
	}
	if ((z_min >= z_1.z) || (z_max <= z_1.z))
	{
		t_1 = -1; //same process as t_0
	}

	
	//Intersect ray with cap planes
	Point3f zMinPt = Point3f(origin.x, origin.y, z_min);
	Point3f zMaxPt = Point3f(origin.x, origin.y, z_max);
	Point3f zMinNormal = Point3f(0, 0, -1);
	Point3f zMaxNormal = Point3f(0, 0, 1);
	float t_3 = (zMinNormal.dot(zMinPt - (ray.o - origin))) / (zMinNormal.dot(ray.d));
	float t_4 = (zMaxNormal.dot(zMaxPt - ray.o)) / (zMaxNormal.dot(ray.d));
	Point3f zMinPlaneX = (ray.o - origin) + ray.d * t_3;
	Point3f zMaxPlaneX = (ray.o - origin) + ray.d * t_4;


	if (zMinPlaneX.x * zMinPlaneX.x + zMinPlaneX.y * zMinPlaneX.y <= radius * radius)
	{
		//intersection with ZMin cap
		capMinX = true;
	}
	if (zMaxPlaneX.x * zMaxPlaneX.x + zMaxPlaneX.y * zMaxPlaneX.y <= radius * radius)
	{
		//intersection with ZMax cap
		capMaxX = true;
	}

	//Test2: If any of the solutions are <0. the cylinder is behind us so throw it out
	if (t_0 < 0) 
	{ 
		           //&& !capMinX && !capMaxX  //this could return an intersection if a cap is behind the camera
		if (t_1 < 0 ) 
		{ 
			si.hit_an_object = false;
			si.color = Vec3b{ 0,0,0 }; //Background color
			return false; 
		}//there are no intersections}
		
		//t_0 isn't valid, so t_1 will be our intersection since it is valid (>= 0)
		t_min = t_1; 
	}
	else if (t_1 < 0) { t_min = t_0; }
	else
	{

		//Take the smallest t as our hit point
		if (t_0 <= t_1)
		{
			t_min = t_0;
		}
		else
		{
			t_min = t_1;
		}
	}
	//Finally test with caps, and take final smallest hitpoint
	if (capMinX && t_3 > 0)
	{
		if (t_3 < t_min)
		{
			t_min = t_3;
			si.normal = zMinNormal;
			openCylinderX = false;
		}
	}
	if (capMaxX && t_4 > 0)
	{
		if (t_4 < t_min)
		{
			t_min = t_4;
			si.normal = zMaxNormal;
			openCylinderX = false;
		}
	}
	
	si.local_hit_point = ray.o + (t_min * ray.d);
	if (openCylinderX)
	{
		si.normal[0] = (si.local_hit_point.x - origin.x) / radius;
		si.normal[1] = (si.local_hit_point.y - origin.y) / radius;
		si.normal[2] = 0; //No z-component normal for cylinders
		si.normal = normalize(si.normal); //Normalize the normal for use in phong shading later on
	}
	si.hit_an_object = true;

	//Use phong shading here:
	si.color = Vec3b{ 255, 255, 255 };
	return true;
};



class Sphere : public Entity {
public:
	Sphere(); //Create sphere with radius 1 at origin with no transformations
	Sphere(float r, Point3f o);
	Sphere(float r, Point3f o, Matrix4f tMtx);
	float getRadiusIfExist();
	bool testIntersect(const Ray& ray, float& t_min, ShadeInfo& si);
	bool fullLightTransmission();
	void setFullLightTransmission(bool val);
	void setTexture(Texture tx);
	Texture getTexture();
	Point3f getOrigin()	{
		return origin;
	}
private:
	float radius;
	Point3f origin;
	Matrix4f transformMtx; //This should be moved to become an ellipsoid as it can be scaled and rotated
	vector<Point3f> normal;
	bool fullLightTransmissionAllowed = false;
	Texture texture;
};

Sphere::Sphere()
{
	radius = 0;
	origin = Point3f(0, 0, 0);
	transformMtx << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Identity Mtx
}

Sphere::Sphere(float r, Point3f o)
{

	radius = r;
	origin = o;
	transformMtx << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //Identity Mtx
}

Sphere::Sphere(float r, Point3f o, Matrix4f tMtx)
{
	
	radius = r;
	origin = o;
	transformMtx = tMtx;
}

float Sphere::getRadiusIfExist()
{
	return radius;
}

void Sphere::setTexture(Texture tx)
{
	texture = tx;
}
Texture Sphere::getTexture()
{
	return texture;
}
bool Sphere::fullLightTransmission()
{
	return fullLightTransmissionAllowed;
}

void Sphere::setFullLightTransmission(bool val)
{
	fullLightTransmissionAllowed = val;
}

bool Sphere::testIntersect(const Ray& ray, float& t_min, ShadeInfo& si)
{
	//Plug ray equation into sphere equation and we get a quadratic equation of the form At^2 + Bt + C = 0
	float a = ray.d.x * ray.d.x + ray.d.y * ray.d.y + ray.d.z * ray.d.z;
	float b = 2 * (ray.d.x * (ray.o.x - origin.x) + ray.d.y * (ray.o.y - origin.y) + ray.d.z * (ray.o.z - origin.z));
	float c = (ray.o.x - origin.x) * (ray.o.x - origin.x) + (ray.o.y - origin.y) * (ray.o.y - origin.y) + (ray.o.z - origin.z) * (ray.o.z - origin.z);
	c = c - radius * radius;

	//solve for t using the quadratic equation
	float determinant = b * b - 4 * a * c;
	//Note: if det < 0: no interesection. If det = 0: there is 1
	if (determinant < 0)
	{
		si.hit_an_object = false;
		si.color = Vec3b{ 0,0,0}; //Background color
		return false;
	}
	float t_0 = (-b - sqrt(determinant)) / 2;
	float t_1 = (-b + sqrt(determinant)) / 2;
	//If any of the solutions are <0. throw it out
	if (t_0 < 0) { t_min = t_1; }
	else if (t_1 < 0) { t_min = t_0; }
	else
	{
		//Take the smallest t as our hit point
		if (t_0 <= t_1)
		{
			t_min = t_0;
		}
		else
		{
			t_min = t_1;
		}
	}
	si.local_hit_point = ray.o + (t_min * ray.d);
	si.normal[0]= (si.local_hit_point.x - origin.x) / radius;
	si.normal[1] = (si.local_hit_point.y - origin.y) / radius;
	si.normal[2] = (si.local_hit_point.z - origin.z) / radius;
	si.normal = normalize(si.normal); //Normalize the normal for use in phong shading later on
	si.hit_an_object = true;

	//Use phong shading here:
	si.color = Vec3b{ 255, 255, 255 }; 
	return true;
};

class TriMesh : public Entity {
public:
	TriMesh(string path); //Create triangle mesh using the .smf found in path parameter
	TriMesh(string path, Matrix4f tMtx); //Create a triangle mesh with transformations applied to it

	float getRadiusIfExist() {
		return -1; //No radius in TriMesh
	}
	//bool testIntersect2(const Ray& ray, float& t_min, ShadeInfo& si);
	bool testIntersect(const Ray& ray, float& t_min, ShadeInfo& si);
	bool fullLightTransmission();
	void setFullLightTransmission(bool val);
	void setTexture(Texture tx) {
		texture = tx;
	}
	Texture getTexture() {
		return texture;
	}
	Point3f getOrigin() {
		Point3f s; //Not implemented
		return s;
	}

private:
	string path;
	vector<Tris> triangles;
	vector<Vec3f> normals;
	static const double kEpsilon; //This is to provide the foundation for shadows
	bool fullLightTransmissionAllowed = false;
	Texture texture;

};

bool TriMesh::fullLightTransmission()
{
	return fullLightTransmissionAllowed;
}

void TriMesh::setFullLightTransmission(bool val)
{
	fullLightTransmissionAllowed = val;
}

TriMesh::TriMesh(string path)
{
	//Import .smf and save as triangles. Does not transform verts using transform mtx

	ifstream file_var; //ifstream is for input from plain text files
	file_var.open(path); //open input.txt
	if (file_var.is_open()) //checking whether the file is open
	{
		string tp;
		vector<Point3f> verts{};
		vector<Point3f> faces{};
		Point3f vert = Point3f(0, 0, 0);
		verts.push_back(vert); //Add a filler vtx because .smf face ordering is 1-based
		Point3f face;
		while (getline(file_var, tp)) //read data from file object and put it into string.
		{
			vector<string> elems{};
			std::string token;
			size_t pos = 0;
			while ((pos = tp.find(' ')) != std::string::npos) {
				token = tp.substr(0, pos);
				elems.push_back(tp.substr(0, pos));
				tp.erase(0, pos + 1);
			}
			//Add the last element (will be either last vtx or last face of the line)
			elems.push_back(tp);

			switch (elems[0][0])
			{
				case('v'):
					vert = Point3f(stof(elems[1]), stof(elems[2]), stof(elems[3]));
					verts.push_back(vert);
					break;
				case('f'): // do more stuff
					face = Point3f(stof(elems[1]), stof(elems[2]), stof(elems[3]));
					faces.push_back(face);
					break;
			}

		}

		//Initialize every vertex normal to 0,0,0
		for (int i = 0; i < verts.size(); i++)
		{
			normals.push_back(Vec3f(0.0f, 0.0f, 0.0f));
		}

		for (int i = 0; i < faces.size(); i++)
		{
			//Populate triangle vector structure
			Tris tri;
			tri.vtx1 = verts[faces[i].x];
			tri.vtx2 = verts[faces[i].y];
			tri.vtx3 = verts[faces[i].z];
			tri.faceTopology = faces[i];
			triangles.push_back(tri);

			//Populate normals vector structure using Gourand vtx avg algorithm
			Point3f vtx0 = verts[faces[i].x];
			Point3f vtx1 = verts[faces[i].y];
			Point3f vtx2 = verts[faces[i].z];
			Vec3f val1 = vtx1 - vtx0;
			Vec3f val2 = vtx2 - vtx0;
			Vec3f faceNormal = normalize(val1.cross(val2));
			normals[faces[i].x] = normals[faces[i].x] + faceNormal;
			normals[faces[i].y] = normals[faces[i].y] + faceNormal;
			normals[faces[i].z] = normals[faces[i].z] + faceNormal;
		}

		//Normalize every vertex normal
		for (int i = 0; i < verts.size(); i++)
		{
			normals[i] = normalize(normals[i]);
		}

		file_var.close();   //close the file object.
	}
	this->path = path;
}

TriMesh::TriMesh(string path, Matrix4f tMtx)
{
	//Import .smf and save as triangles. 

	ifstream file_var; //ifstream is for input from plain text files
	file_var.open(path); //open input.txt
	if (file_var.is_open()) //checking whether the file is open
	{
		string tp;
		vector<Point3f> verts{};
		vector<Point3f> faces{};
		Matrix<float, 4, 1> vert; //TODO: Another case of using Eigen for Vector/Mtx math but using opencv Point3f otherwise. Lets choose one or the other.
		Point3f cv_vert = Point3f(0, 0, 0);
		verts.push_back(cv_vert); //Add a filler vtx because .smf face ordering is 1-based
		Point3f face;
		while (getline(file_var, tp)) //read data from file object and put it into string.
		{
			vector<string> elems{};
			std::string token;
			size_t pos = 0;
			while ((pos = tp.find(' ')) != std::string::npos) {
				token = tp.substr(0, pos);
				elems.push_back(tp.substr(0, pos));
				tp.erase(0, pos + 1);
			}
			//Add the last element (will be either last vtx or last face of the line)
			elems.push_back(tp);

			switch (elems[0][0])
			{
			case('v'):
				
				//Apply transformation matrix
				vert(0, 0) = stof(elems[1]);
				vert(1, 0) = stof(elems[2]);
				vert(2, 0) = stof(elems[3]);
				vert(3, 0) = 1.0f;

				vert = tMtx* vert;

				cv_vert = Point3f(vert(0, 0), vert(1, 0), vert(2, 0));
				verts.push_back(cv_vert);
				break;
			case('f'): // do more stuff
				face = Point3f(stof(elems[1]), stof(elems[2]), stof(elems[3]));
				faces.push_back(face);
				break;
			}

		}

		//Initialize every vertex normal to 0,0,0
		for (int i = 0; i < verts.size(); i++)
		{
			normals.push_back(Vec3f(0.0f, 0.0f, 0.0f));
		}

		for (int i = 0; i < faces.size(); i++)
		{
			//Populate triangle vector structure
			Tris tri;
			tri.vtx1 = verts[faces[i].x];
			tri.vtx2 = verts[faces[i].y];
			tri.vtx3 = verts[faces[i].z];
			tri.faceTopology = faces[i];
			triangles.push_back(tri);

			//Populate normals vector structure using Gourand vtx avg algorithm
			Point3f vtx0 = verts[faces[i].x];
			Point3f vtx1 = verts[faces[i].y];
			Point3f vtx2 = verts[faces[i].z];
			Vec3f val1 = vtx1 - vtx0;
			Vec3f val2 = vtx2 - vtx0;
			Vec3f faceNormal = normalize(val1.cross(val2));
			normals[faces[i].x] = normals[faces[i].x] + faceNormal;
			normals[faces[i].y] = normals[faces[i].y] + faceNormal;
			normals[faces[i].z] = normals[faces[i].z] + faceNormal;
		}

		//Normalize every vertex normal
		for (int i = 0; i < verts.size(); i++)
		{
			normals[i] = normalize(normals[i]);
		}

		file_var.close();   //close the file object.
	}
	this->path = path;
}

//I am using Cramers rule here. 
bool TriMesh::testIntersect(const Ray& ray, float& t_min, ShadeInfo& si)
{
	float best_a = -1; //Barycentric coordinates of closest tri intersection
	float best_b = -1;
	float best_y = -1;
	int best_tri = -1; //Tri index for closest tri intersection 
	float best_t_min = -1; //ray's t value for closest tri intersection

	for (int i = 0; i < triangles.size(); i++)
	{
		//TODO: Do this through a library function? Run as its own exe outside of IDE? Raytracer spends a lot of time here
		
		float a_ = triangles[i].vtx1.x - triangles[i].vtx2.x;
		float b_ = triangles[i].vtx1.x - triangles[i].vtx3.x;
		float c_ = ray.d.x;
		float d_ = triangles[i].vtx1.y - triangles[i].vtx2.y;
		float e_ = triangles[i].vtx1.y - triangles[i].vtx3.y;
		float f_ = ray.d.y;
		float g_ = triangles[i].vtx1.z - triangles[i].vtx2.z;
		float h_ = triangles[i].vtx1.z - triangles[i].vtx3.z;
		float i_ = ray.d.z;


		float detA = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);

		//float detA = a_ * (e_ * i_ - f_ * h_) - b_ * (d_ * i_ - f_ * g_);
		//detA = detA + c_ * (d_ * h_ - e_ * g_);

		a_ = triangles[i].vtx1.x - ray.o.x;
		b_ = triangles[i].vtx1.x - triangles[i].vtx3.x;
		c_ = ray.d.x;
		d_ = triangles[i].vtx1.y - ray.o.y;
		e_ = triangles[i].vtx1.y - triangles[i].vtx3.y;
		f_ = ray.d.y;
		g_ = triangles[i].vtx1.z - ray.o.z;
		h_ = triangles[i].vtx1.z - triangles[i].vtx3.z;
		i_ = ray.d.z;

		//float detB = b_mtx.determinant();
		float detB = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);
		float b = detB / detA; //beta  (barymetric weight)

		//b = a_ * (e_ * i_ - f_ * h_) - b_ * (d_ * i_ - f_ * g_);
		//b = b + c_ * (d_ * h_ - e_ * g_);
		//b = b / detA;

		a_ = triangles[i].vtx1.x - triangles[i].vtx2.x;
		b_ = triangles[i].vtx1.x - ray.o.x;
		c_ = ray.d.x;
		d_ = triangles[i].vtx1.y - triangles[i].vtx2.y;
		e_ = triangles[i].vtx1.y - ray.o.y;
		f_ = ray.d.y;
		g_ = triangles[i].vtx1.z - triangles[i].vtx2.z;
		h_ = triangles[i].vtx1.z - ray.o.z;
		i_ = ray.d.z;
		Matrix3f y_mtx;
		y_mtx << a_, b_, c_,
			d_, e_, f_,
			g_, h_, i_;
		//y = a_ * (e_ * i_ - f_ * h_) - b_ * (d_ * i_ - f_ * g_);
		//y = b + c_ * (d_ * h_ - e_ * g_);
		//y = y / detA;
		float detY = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);
		//float detY = y_mtx.determinant();
		float y = detY / detA; //gamma (barymetric weight)

		a_ = triangles[i].vtx1.x - triangles[i].vtx2.x;
		b_ = triangles[i].vtx1.x - triangles[i].vtx3.x;
		c_ = triangles[i].vtx1.x - ray.o.x;
		d_ = triangles[i].vtx1.y - triangles[i].vtx2.y;
		e_ = triangles[i].vtx1.y - triangles[i].vtx3.y;
		f_ = triangles[i].vtx1.y - ray.o.y;
		g_ = triangles[i].vtx1.z - triangles[i].vtx2.z;
		h_ = triangles[i].vtx1.z - triangles[i].vtx3.z;
		i_ = triangles[i].vtx1.z - ray.o.z;

		//t = a_ * (e_ * i_ - f_ * h_) - b_ * (d_ * i_ - f_ * g_);
		//t = t + c_ * (d_ * h_ - e_ * g_);
		//t = t / detA;
		float detT = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);
		//float detT = t_mtx.determinant();
		float t = detT / detA; //t value (parametric value of ray where interesection occurs)

		if (b >= 0 && y >= 0 && t >= 0)
		{
			if (b + y <= 1)
			{
				//TODO: Optimize this. Will store the t_min of the intersection from the first found 'i'
				if ((t < best_t_min) || best_t_min == -1)
				{
					best_t_min = t;
					best_a = 1 - b - y; //Also saving the barycentric coordinates of the triangle 
					best_b = b;
					best_y = y;
					best_tri = i; //And the face that with the closest intersection for normal calculation - TODO: Optimize faces_usedforNormals and triangles together
				}

			}
		}

	}

	if (best_t_min != -1)
	{
		//We have an intersection with triangle 'i'!
		t_min = best_t_min;
		si.local_hit_point = ray.o + (t_min * ray.d);
		//Using the barycentric coordinates calculated in the intersection finding code above to help us get the normal
		si.normal = Vec3f(best_a * normals[triangles[best_tri].faceTopology.x] + best_b * normals[triangles[best_tri].faceTopology.y] + best_y * normals[triangles[best_tri].faceTopology.z]);
		si.hit_an_object = true;
		si.color = Vec3b{ 255, 255, 255 };
		return true;
	}
	si.hit_an_object = false;
	si.color = Vec3b{ 0,0,0 }; //Background color
	return false;
}


Vec3f getPixelColorFromRay(vector<Entity*> worldObjects, vector<Light*> worldLights, ShadeInfo* si, vector<ShadeInfo*> objectMaterials, Point3f cam_loc, int entity, int depth)
{
	if (si->hit_an_object || depth > 0) // If the relevant entity was hit, or if we are reaching this point from a reflection ray computation
	{
		//There must be at least 1 light in the scene
		Vec3f color = Vec3f(0, 0, 0);

		Ray shadowRay;
		Ray reflectionRay;
		//TODO: Can I reduce the number of shade information variables used?
		
		ShadeInfo si_reflection = ShadeInfo();
		ShadeInfo test = ShadeInfo();//don't need this, but it is required in testIntersect function.
		ShadeInfo* pTemp_si = &test;
		float t_min;

		Vec3f specularSum = Vec3f(0.0, 0.0, 0.0);
		Vec3f diffuseSum = Vec3f(0.0, 0.0, 0.0);
		Vec3f reflectedColor = Vec3f(0.0, 0.0, 0.0);
		Vec3f refractedColor = Vec3f(0.0, 0.0, 0.0);

		//viewVec needed for both specular color inside for loop and reflected color outside for loop
		Vec3f viewVec = p2v(cam_loc - si->local_hit_point);

		float k_s = 0.15; //Specular Coeffecient
		float k_d = 0.45; //Diffuse Coeffecient
		float k_t = 0.4; //Transmissive Coeffecient
		

		for (int l = 0; l < worldLights.size(); l++)
		{
			//If an object occludes the ray from the ray's intersection point to this light, skip the loop to the next light
			shadowRay.d = normalize(worldLights[l]->origin - si->local_hit_point);
			Point3f normalPt = Point3f((0.005 * si->normal)[0], (0.005 * si->normal)[1], (0.005 * si->normal)[2]);
			shadowRay.o = si->local_hit_point + normalPt;
			bool skipLight = false;
			for (int k = 0; k < worldObjects.size(); k++)
			{
				if (worldObjects[k]->testIntersect(shadowRay, t_min, *pTemp_si) && t_min > 0 && !worldObjects[k]->fullLightTransmission())
				{
					//Object occludes this light ray
					skipLight = true;
					break;
				}

			}
			if (skipLight)
			{
				continue;
			}


			//Pull texture if entity is textured

			Vec3f diffuseColor = si->diffuseColor;
			Texture tex;
			tex = worldObjects[entity]->getTexture();
			if (tex.type == 1)
			{
				float r = worldObjects[entity]->getRadiusIfExist();
				Point3f o = worldObjects[entity]->getOrigin();

				Vec3f N = normalize(si->local_hit_point - o);

				float u = 0.5 + atan2(N[2], N[0]) / (2 * PI);
				float v = 0.5 - asin(N[1]) / PI;

				int x = (u * tex.image.rows);
				int y = (v * tex.image.cols);

				Vec3b testing = tex.image.at<cv::Vec3b>(x, y);
				diffuseColor = Vec3f(testing) / 255;

				//float phi = acos(z / r);
				//loat theta = acos(x / (r * sin(acos(z / r))));
				//float theta2 = asin(y / (r * sin(acos(z / r))));

				
			}
			else if (tex.type == 2)
			{
				//We are doing a pulse-based 3D Procedural texture here
				float xPos = fmod(si->local_hit_point.x * 10.0,2.0) / 10.0;
				float yPos = fmod(si->local_hit_point.y * 10.0, 1.0) / 10.0;
				float zPos = fmod(si->local_hit_point.z * 10.0, 2.0) / 10.0;

				Vec3f wood = Vec3f(0.43, 0.37, 0.17) + Vec3f(yPos, yPos, yPos) - 0.1 * Vec3f(0.73, 0.56, 0.56) + Vec3f(xPos, xPos, xPos);
				diffuseColor = Vec3f(1, 1, 1) * (fmod(si->local_hit_point.x * si->local_hit_point.x + si->local_hit_point.z * si->local_hit_point.z, 1.0) * 3.0) * wood;
			}
			else if(tex.type == 3)
			{
				//We are doing a noisy 3D Procedural texture here
				//diffuseColor = Vec3f(fmod(si->local_hit_point.y * 20.0, 21.0) / 10.0, fmod(si->local_hit_point.x * 200.0, 21.0) / 10.0, fmod(si->local_hit_point.z * 200.0, 2.0) / 10.0);
			}


			Vec3f lightVec = p2v(worldLights[l]->origin - si->local_hit_point);
			Vec3f reflectedRay = (2 * std::max(si->normal.dot(lightVec), 0.0f) * si->normal) - lightVec;
			Vec3f halfwayVec = normalize(lightVec + viewVec);

			reflectedRay = normalize(reflectedRay);
			lightVec = normalize(lightVec);
			viewVec = normalize(viewVec);

			float attenuation = min(1 / (norm(Mat(si->local_hit_point) - Mat(worldLights[l]->origin)) * worldLights[l]->att), 1.0); //TODO: Make our own euclidean distance func?
			float V_R = std::max(reflectedRay.dot(viewVec), 0.0f);
			float H_N = halfwayVec.dot(si->normal);
			Vec3f specular = worldLights[l]->colorIntensity.mul(si->specularColor) * pow(H_N, si->shininess);

			float N_L = std::max(lightVec.dot(si->normal), 0.0f);
			Vec3f diffuse = worldLights[l]->colorIntensity.mul(diffuseColor) * N_L * attenuation;

			float spotLightCoeff = 1;
			if (worldLights[l]->type == SPOT_LIGHT)
			{
				float falloff = lightVec.dot(worldLights[l]->direction);
				if (falloff < 0)
				{
					//If dot product is negative, lets get a useable angle instead of a negative angle
					falloff = PI / 2 + abs(falloff);
				}
				if (falloff < worldLights[l]->coneAngle)
				{
					//Spotlight does not contribute to this intersection
					continue;
				}
				else
				{
					spotLightCoeff = pow(cos((PI / 2) * (falloff / worldLights[l]->coneAngle)), worldLights[l]->fallOffParam);
					spotLightCoeff = max(spotLightCoeff, 0.0f);
				}
				
			}
			
			
			diffuseSum += diffuse *  spotLightCoeff;
			specularSum += specular * spotLightCoeff;
			
		}
		
		Vec3f ambience = worldLights[0]->colorIntensity.mul(si->ambientColor);

	
		//Shoot Reflection ray
		Vec3f viewReflectionRay = 2 * (si->normal.dot(viewVec) * si->normal) - viewVec;

		reflectionRay.d = viewReflectionRay;
		Point3f normalPt = Point3f((0.001 * si->normal)[0], (0.001 * si->normal)[1], (0.001 * si->normal)[2]);
		reflectionRay.o = si->local_hit_point + normalPt;
		float best_t_min = -1;
		int entityNum = -1;
		ShadeInfo* temp_si;

		//For some reason Changing si_x (si from the intersection point) to si gives a good reflection of the cieling (b/c first object in worldObjects?). 
		for (int k = 0; k < worldObjects.size() && si->reflectiveness != 0; k++)
		{

			//Pull the material shader info for this object
			temp_si = objectMaterials[k];

			//Shoot reflection ray and see if it hits anything
			//the computed shader info varibles will be set in pTemp_si by testIntersect
			if (worldObjects[k]->testIntersect(reflectionRay, t_min, *temp_si) && t_min > 0)
			{

				if ((t_min < best_t_min) || (best_t_min == -1))
				{
					best_t_min = t_min;

					//Pull the material shader info for this object
					si_reflection = *temp_si; //= *objectMaterials[k];
					si_reflection.hit_an_object = temp_si->hit_an_object;
					si_reflection.local_hit_point = temp_si->local_hit_point;
					si_reflection.normal = temp_si->normal;

					entityNum = k;
				}
			}

			else if (best_t_min == -1)
			{
				//No interesction from any of the world objects were found
				si_reflection = *temp_si;
			}

		}
		//Adding the reflected ray colors

		if (best_t_min != -1)
		{
			depth++;
			if (depth < MAX_REFLECTION_DEPTH)
			{
				temp_si = &si_reflection;
 				reflectedColor = getPixelColorFromRay(worldObjects, worldLights, temp_si, objectMaterials, cam_loc, entityNum, depth) / 255;
				
			}
		}
		
		


		//Calculating Refraction ray
		float indxRef = si->indexoOfRefraction; //index of refraction		

		//Raycasting refraction ray

		best_t_min = -1;
		Ray refractionRay;
		normalPt = Point3f((0.001 * si->normal)[0], (0.001 * si->normal)[1], (0.001 * si->normal)[2]);
		refractionRay.o = si->local_hit_point + normalPt; //Normals are facing outwards, but we are headed in the object
		
		refractionRay.d = computeRefractionRay(indxRef, 1, viewVec, si->normal);

		//reusing temp_si variable that was defined in reflections code
		ShadeInfo refraction_si = ShadeInfo(); //Need to initialize with something
		int temp = 0;
		entityNum = -1;
		for (int k = 0; k < worldObjects.size() && (si->refractiveness != 0); k++)
		{
			if (k == entity)
			{
				continue;
			}
			//Pull the material shader info for this object
			temp_si = objectMaterials[k];


			if (worldObjects[k]->testIntersect(refractionRay, t_min, *temp_si) && t_min > 0)
			{
				if ((t_min < best_t_min) || (best_t_min == -1))
				{
					best_t_min = t_min;
					refraction_si = *temp_si;
					entityNum = k;
				}
			}

			else if (best_t_min == -1)
			{
				//the point we have hit is not part of a closed 3d model so it can' be refractive
			}

		}
		if (si->refractiveness && best_t_min != -1)
		{
			depth++;
			if (depth < 2)
			{
				refractedColor = getPixelColorFromRay(worldObjects, worldLights, &refraction_si, objectMaterials, cam_loc, entityNum, depth) / 255;
			}
		}
	


		Vec3f reflectance = reflectedColor.mul(si->specularColor) * si->reflectiveness; // reflectedColor* si_x.reflectiveness //* worldLights[0]->att;
		
		//si_x.reflectiveness is being hijacked to mean refractiveness for Assignment7
		Vec3f refractions = refractedColor.mul(si->specularColor) * si->refractiveness; //kt - transmissive coefficient * Ct - transmissive color of object * It - returned intensity of refraction ray

		color += (ambience +  k_d * diffuseSum + k_s * specularSum + reflectance + k_t * refractions);
		color *= 255;
		//Clamp colors to 255 (max power white)
		if (color(0) > 255) { color(0) = 255; }
		if (color(1) > 255) { color(1) = 255; }
		if (color(2) > 255) { color(2) = 255; }

		//Never have a pure black color (0)
		if (color(0) == 0) { color(0) = 1; }
		if (color(0) == 0) { color(1) = 1; }
		if (color(0) == 0) { color(2) = 1; }

		return color;

	}
	else
	{
		//si.color will be the background color if there no ray intersections were found
		return si->color;
	}
}


Vec3b getColorFromComputedRay(float i, float j, Point3f topLeftPxl, float imgPlaneW, float imgPlaneH, int xRes, int yRes, Point3f camCoordX, Point3f camCoordY, Point3f cam_loc, vector<Entity*> worldObjects, ShadeInfo* si, vector<ShadeInfo*> objectMaterials, vector<Light*> worldLights)
{
	//Very odd artifacts introduced between Assignment 4 and Assignment 5.
	// si->normal and lightVec are (-0.707106, -0.353778, ~0) and (1, 0.045, 0) and no diffuse is added (thus it is black) for both lights
	// si->normal and lightVec are (0.5, -0.707, ~0) and (1, 0.1, 0) and diffuse is added (thus it has color) for both lights
	// These 2 pixels are right next to eachother... wierd normals being saved?

	Point3f vectorSum = imgPlaneW * (i / (xRes - 1)) * (Vec3f)camCoordX - imgPlaneH * (j / (yRes - 1)) * (Vec3f)camCoordY;
	Point3f ray_direction = (topLeftPxl + vectorSum) - cam_loc;
	ray_direction = normalize(ray_direction);
	Ray currentRay;
	ShadeInfo* temp_si;

	currentRay.d = ray_direction;
	currentRay.o = cam_loc;
	float best_t_min = -1;
	float t_min;

	//The raytracing code:

	int temp = 0;
	int entityNum = -1;
	for (int k = 0; k < worldObjects.size(); k++)
	{
		//Pull the material shader info for this object
		temp_si = objectMaterials[k];

		if (i == 77 && j == 200)
		{
			int dumb = 0;
		}


		//the computed shader info varibles will be set in temp_si by testIntersect
		if (worldObjects[k]->testIntersect(currentRay, t_min, *temp_si))
		{
			if ((t_min < best_t_min) || (best_t_min == -1))
			{
				best_t_min = t_min;
				si = temp_si;
				entityNum = k;
			}
		}

		else if (best_t_min == -1)
		{
			//No interesction from any of the world objects were found
			si = temp_si;
		}

	}

	//this is seperated into a new function for readability \/
	Vec3f colorF = getPixelColorFromRay(worldObjects, worldLights, si, objectMaterials, cam_loc, entityNum, 0);
	Vec3b color = Vec3b(colorF(2), colorF(1), colorF(0)); //TODO: use Eigen's vectors instead of opencv vectors. Also this is BGR?

	//never return a pure black (0) color
	if (color[0] == 0) { color[0] = 1; }
	if (color[1] == 0) { color[1] = 1; }
	if (color[2] == 0) { color[2] = 1; }
	return color;
}


bool aSupSampToleranceTest(Vec3b A, Vec3b B, float tolerance)
{
	bool tTest1 = std::abs(A[0] - B[0]) < tolerance * 255;
	bool tTest2 = std::abs(A[0] - B[0]) < tolerance * 255;
	bool tTest3 = std::abs(A[0] - B[0]) < tolerance * 255;

	return tTest1 && tTest2 && tTest3;
}

int main(int argc, char** argv)
{

	//Starting values
	//Point3f cam_loc = Point3f(4, 0.5, 0); //desklamp scene
	Point3f cam_loc = Point3f(4, 0.5, 0);
	Point3f cam_view_out = Point3f(-1, 0, -0.15);
	//Point3f cam_view_out = Point3f(-1, 0, -0.15);
	Point3f cam_view_up = Point3f(0, 1, 0);
	float cam_horiz_angle = 56 * PI / 180;
	int xRes = IMAGE_RES_X + 1;
	int yRes = IMAGE_RES_Y + 1;
	float d = 3;
	cv::Mat3b image = cv::Mat3b(xRes, yRes, cv::Vec3f{ 0,0,0 });


	//Setup object,material, and light vectors
	vector<Entity*> worldObjects;
	vector<ShadeInfo*> objectMaterials;
	vector<Light*> worldLights;

	//Add lights to scene
	

	
	Matrix4f rotate90X;
	rotate90X << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;//Scale cube to make it a bridge
	Matrix4f rotate90Y;
	rotate90Y << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1;
	Matrix4f rotate90Z;
	rotate90Z << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

	//Scene -1
	/*
	Light l1 = Light(POINT_LIGHT, Point3f(3, 0, 1), Vec3f(1.0, 1.0, 1.0), 0.4);
	worldLights.push_back(&l1);

	Cylinder cylinder = Cylinder(1, 1, Point3f(0,0, 0));
	ShadeInfo cylinderMat = ShadeInfo(Vec3f(0.05, 0.06, 0.08), Vec3f(0.2, 0.2, 0.25), Vec3f(1, 1, 1), 20.0, 0, 0);
	worldObjects.push_back(&cylinder);
	objectMaterials.push_back(&cylinderMat);

	Cylinder cylinder2 = Cylinder(1, 0.3, Point3f(0, -1, 0));
	ShadeInfo cylinderMat2 = ShadeInfo(Vec3f(0.05, 0.06, 0.08), Vec3f(0.2, 0.2, 0.25), Vec3f(1, 1, 1), 20.0, 0, 0);
	worldObjects.push_back(&cylinder2);
	objectMaterials.push_back(&cylinderMat2);
	*/

	//Scene 1
	/*
	Light l1 = Light(POINT_LIGHT, Point3f(3, 2, 2), Vec3f(0.8, 0.8, 1.0), 0.1);
	worldLights.push_back(&l1);
	
	Light l4 = Light(POINT_LIGHT, Point3f(3, 2, 2), Vec3f(0.8, 0.8, 1.0), 0.1);
	worldLights.push_back(&l4);

	Light l2 = Light(POINT_LIGHT, Point3f(3, 0, -1), Vec3f(1.0, 1.0, 1.0), 0.1);
	worldLights.push_back(&l2);
	
	Light l3 = Light(POINT_LIGHT, Point3f(2, -1, 0), Vec3f(1.0, 1.0, 1.0), 0.1);
	worldLights.push_back(&l3);
	

	//Create the room
	Matrix4f cubeToCeiling;
	Matrix4f cubeToWall;
	Matrix4f cubeToWall2;
	Matrix4f cubeToWall3;
	Matrix4f cubeToFloor;
	//TODO: something really wierd going on: vertex position is affecting lighting?
	cubeToCeiling << 4, 0, 0, -1.5,  0, 4, 0, 0,  0, 0, 4, 4,  0, 0, 0, 1;
	cubeToWall << 4, 0, 0, -5, 0, 4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 1; //Back wall
	cubeToWall2 << 4, 0, 0, -1.5,  0, 4, 0, 4,  0, 0, 4, 0, 0, 0, 0, 1;
	cubeToWall3 << 4, 0, 0, -1.5, 0, 4, 0, -4, 0, 0, 4, 0, 0, 0, 0, 1;
	cubeToFloor << 4, 0, 0, -1.5,  0, 4, 0, 0,  0, 0, 4, -4,  0, 0, 0, 1;


	

	TriMesh wall = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", cubeToWall	);
	worldObjects.push_back(&wall);
	ShadeInfo si_wall = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.3, 0.3, 0.3), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_wall);
	
	TriMesh ceiling = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", cubeToCeiling);
	worldObjects.push_back(&ceiling);
	ShadeInfo si_ceiling = ShadeInfo(Vec3f(0.01, 0.1, 0.2), Vec3f(0.2, 0.26, 0.3), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_ceiling);

	TriMesh wall2 = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", cubeToWall2);
	worldObjects.push_back(&wall2);
	ShadeInfo si_wall2 = ShadeInfo(Vec3f(0.2, 0.1, 0.01), Vec3f(0.3, 0.26, 0.2), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_wall2);

	TriMesh wall3 = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", cubeToWall3);
	worldObjects.push_back(&wall3);
	ShadeInfo si_wall3 = ShadeInfo(Vec3f(0.2, 0.01, 0.1), Vec3f(0.3, 0.2, 0.26), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_wall3);
	
	TriMesh floor = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", cubeToFloor);
	worldObjects.push_back(&floor);
	ShadeInfo si_floor = ShadeInfo(Vec3f(0.1, 0.01, 0.2), Vec3f(0.26, 0.22, 0.3), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_floor);
	/*
	Sphere s2 = Sphere(0.3, Point3f(1, -1, -1));
	ShadeInfo randSphere1 = ShadeInfo(Vec3f(0.05, 0.1, 0.05), Vec3f(0.1, 0.2, 0.1), Vec3f(1, 1, 1), 100.0, 0, 1, 1.2);
	worldObjects.push_back(&s2);
	objectMaterials.push_back(&randSphere1);



	Sphere s3 = Sphere(0.3, Point3f(-1, -1, 0));
	ShadeInfo randSphere4 = ShadeInfo(Vec3f(0.01, 0.05, 0.1), Vec3f(0.05, 0.05, 0.15), Vec3f(1, 1, 1), 100.0, 0, 0, 1);
	worldObjects.push_back(&s3);
	objectMaterials.push_back(&randSphere4);
	*/
	/*
	//Huge sphere
	Sphere sphere2 = Sphere(0.9, Point3f(2, 0, 0));
	ShadeInfo randSphere3 = ShadeInfo(Vec3f(.1, 0.1, 0.1), Vec3f(0.04, 0.2, 0.4), Vec3f(1, 1, 1), 100, 1, 0);
	worldObjects.push_back(&sphere2);
	objectMaterials.push_back(&randSphere3);


	Sphere sphere1 = Sphere(0.8, Point3f(-1, 0, 1));
	ShadeInfo randSphere2 = ShadeInfo(Vec3f(0.2, 0.1, 0.1), Vec3f(0.5, 0.2, 0.2), Vec3f(1, 1, 1), 100.0, 1, 0);
	worldObjects.push_back(&sphere1);
	objectMaterials.push_back(&randSphere2);



	
	
	Sphere reflectiveSphere = Sphere(1, Point3f(-2, 1, 1));
	ShadeInfo reflectiveSphere2 = ShadeInfo(Vec3f(.1, 0.1, 0.1), Vec3f(0.04, 0.2, 0.4), Vec3f(1, 1, 1), 100, 1, 0, 1);
	worldObjects.push_back(&reflectiveSphere);
	objectMaterials.push_back(&reflectiveSphere2);

	Matrix4f translate;
	translate << 1, 0, 0, 0,  0, 1, 0, -0.5,  0, 0, 1, -0.5,  0, 0, 0, 1;
	TriMesh octo = TriMesh("C:/Users/austi/Documents/Raytracer/A5/octahedron.smf", translate);
	ShadeInfo si2 = ShadeInfo(Vec3f(0.15, 0.05, 0.01), Vec3f(0.4, 0.2, 0.01), Vec3f(1, 1, 1), 10.0, 0, 0, 1);
	Texture wood;
	wood.type = 3;
	octo.setTexture(wood);
	worldObjects.push_back(&octo);
	objectMaterials.push_back(&si2);

		*/
	/*
	Matrix4f translateOcta;
	translateOcta << 0.1, 0, 0, 2, 0, 0.1, 0, 0, 0, 0, 0.1, -1, 0, 0, 0, 1;
	TriMesh octahedren = TriMesh("C:/Users/austi/Documents/Raytracer/A5/octahedron.smf", translateOcta);
	ShadeInfo si2_octa = ShadeInfo(Vec3f(0.15, 0.05, 0.01), Vec3f(0.4, 0.2, 0.01), Vec3f(1, 1, 1), 10.0, 1);
	worldObjects.push_back(&octahedren);
	objectMaterials.push_back(&si2_octa);


	


	Matrix4f translate2;
	translate2 << 0.3, 0, 0, 0,  0, 0.3, 0, 11,  0, 0, 0.3, 11,  0, 0, 1, 0; //Scale cube to make it a bridge
	TriMesh outhouse = TriMesh("C:/Users/austi/Documents/Raytracer/A5/teapot.smf", translate2);
	ShadeInfo si5 = ShadeInfo(Vec3f(0.2, 0.1, 0.01), Vec3f(0.3, 0.5, 0.01), Vec3f(1, 1, 1), 30.0, 1);
	worldObjects.push_back(&outhouse);
	objectMaterials.push_back(&si5);
	*/

	//Scene 2
	
	//Guilty Spark Glow
	/*
	Light l1 = Light(POINT_LIGHT, Point3f(2, 2.7, 1), Vec3f(1.0, 1.0, 1.0), 0.1);
	worldLights.push_back(&l1);
	
	Light l4 = Light(POINT_LIGHT, Point3f(2, 2.7, 1), Vec3f(1.0, 1.0, 1.0), 0.1);
	worldLights.push_back(&l4);

	Light l2 = Light(POINT_LIGHT, Point3f(2, -1, 1), Vec3f(1.0, 0.5, 0.5), 0.1);
	worldLights.push_back(&l2);
	
	Light l3 = Light(POINT_LIGHT, Point3f(0.2, -0.1, 0.5), Vec3f(0.05, 0.5, 1.0), 0.7);
	worldLights.push_back(&l3);
	
	
	Sphere s2 = Sphere(0.15, Point3f(1, -0.6, -1.35));
	ShadeInfo randSphere1 = ShadeInfo(Vec3f(0.01, 0.1, 0.3), Vec3f(0.1, 0.2, 0.4), Vec3f(1, 1, 1), 80.0, 0, 0, 1);
	worldObjects.push_back(&s2);
	objectMaterials.push_back(&randSphere1);

	Sphere s3 = Sphere(0.2, Point3f(1, -1.0, -1.3));
	ShadeInfo randSphere2 = ShadeInfo(Vec3f(0.01, 0.07, 0.1), Vec3f(0.05, 0.16, 0.25), Vec3f(1, 1, 1), 80.0, 0, 1, 2);
	worldObjects.push_back(&s3);
	objectMaterials.push_back(&randSphere2);

	Sphere ballOnTop = Sphere(0.2, Point3f(0, -1.53, -0.4));
	ShadeInfo ballOnTopMat = ShadeInfo(Vec3f(0.02, 0.02, 0.04), Vec3f(0.05, 0.06, 0.1), Vec3f(1, 1, 1), 10.0, 1, 0, 1.5);
	worldObjects.push_back(&ballOnTop);
	objectMaterials.push_back(&ballOnTopMat);
	/*
	Sphere s4 = Sphere(0.45, Point3f(-2, -1.4, -0.9));
	ShadeInfo randSphere3 = ShadeInfo(Vec3f(0.01, 0.03, 0.07), Vec3f(0.1, 0.12, 0.2), Vec3f(1, 1, 1), 10.0, 1, 0, 1);
	worldObjects.push_back(&s4);
	objectMaterials.push_back(&randSphere3);
	
	Sphere planet = Sphere(0.45, Point3f(0, 1.3, 1.5));
	ShadeInfo planetMat = ShadeInfo(Vec3f(0.1, 0.12, 0.2), Vec3f(0.1, 0.12, 0.2), Vec3f(1, 1, 1), 20.0, 0, 0, 1);
	Texture haloPlanet;
	haloPlanet.path = "C:/Users/austi/Documents/Raytracer/Extra Credit/haloPlanet.png";
	haloPlanet.type = 1;
	Mat img = imread(haloPlanet.path, IMREAD_COLOR);
	if (img.empty())
	{
		std::cout << "Texture at " << haloPlanet.path << " failed to load";
		return 0;
	}
	haloPlanet.image = img;
	planet.setTexture(haloPlanet);
	worldObjects.push_back(&planet);
	objectMaterials.push_back(&planetMat);

	//TODO: Low priority: right now if I use the same Shading Info si object for two entities, one of the entities dissapears. Fix this


	Matrix4f rotate45Mtx;
	ShadeInfo si2 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.2, 0.2, 0.2), Vec3f(1, 1, 1), 10, 0, 0, 1);
	rotate45Mtx << 0.707, -0.707, 0, 0, 0.707, 0.707, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //Rotate 45 deg
	TriMesh monitor = TriMesh("C:/Users/austi/Documents/Raytracer/A5/monitor.smf", rotate45Mtx);	
	worldObjects.push_back(&monitor);
	objectMaterials.push_back(&si2);
	
	ShadeInfo si0 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.2, 0.2, 0.2), Vec3f(1, 1, 1), 10, 0, 0, 1);
	TriMesh monitorFin = TriMesh("C:/Users/austi/Documents/Raytracer/A5/monitor_fin.smf", rotate45Mtx);
	monitorFin.setFullLightTransmission(true);
	worldObjects.push_back(&monitorFin);
	objectMaterials.push_back(&si0);
	
	Sphere s1 = Sphere(0.32, Point3f(-0.1, -0.1, 0));
	s1.setFullLightTransmission(true);
	ShadeInfo si1 = ShadeInfo(Vec3f(0.1, 0.1, 0.5), Vec3f(0.05, 0.7, 1.0), Vec3f(1, 1, 1), 5, 0, 0, 1);
	worldObjects.push_back(&s1);
	objectMaterials.push_back(&si1);
	
	Sphere eye = Sphere(0.28, Point3f(0.15, 0.15, 0));
	eye.setFullLightTransmission(true);
	ShadeInfo si_eye = ShadeInfo(Vec3f(0.1, 0.1, 0.5), Vec3f(0.05, 0.7, 1.0), Vec3f(1, 1, 1), 5.0, 0, 0, 1);
	worldObjects.push_back(&eye);
	objectMaterials.push_back(&si_eye);
	
	


		//TODO: Bug here, the homogeneous translation matrix doesn't work as it should (-15 represents -2?)\
	//this is because we are doing the transofmration ordering wrong. scaling should be last?

	Matrix4f cubeToBridge;
	cubeToBridge << 1, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 1; //Scale cube to make it a bridge
	Matrix4f cubeToFloor;
	cubeToFloor << 9, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 1; //Scale cube to make it a bridge
	Matrix4f translateZDown;

	translateZDown << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -15, 0, 0, 0, 1;
	Matrix4f productBridge = cubeToFloor * translateZDown;
	TriMesh bridge = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", productBridge);
	//ShadeInfo si3 = ShadeInfo(Vec3f(.09, 0.25, 1), Vec3f(0.6, 1, 1), Vec3f(1, 1, 1), 10.0);
	worldObjects.push_back(&bridge);
	ShadeInfo si_bridge = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_bridge);



	Matrix4f translateZDownYLeft;
	translateZDownYLeft << 1, 0, 0, 0.5,  0, 1, 0, 1,  0, 0, 0.5, -1,  0, 0, 0, 1;
	TriMesh pillar2 = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cylinder.smf", translateZDownYLeft);
	worldObjects.push_back(&pillar2);
	ShadeInfo si_pillar2 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_pillar2);
	
	Matrix4f translateZDownYLeft2;
	translateZDownYLeft2 << 1, 0, 0, 0.75,  0, 1, 0, 1.5,  0, 0, 0.5, -1,  0, 0, 0, 1;
	TriMesh pillar = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cylinder.smf", translateZDownYLeft2);
	worldObjects.push_back(&pillar);
	ShadeInfo si_pillar = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.2, 0.2, 0.2), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_pillar);
	
	Matrix4f translateZDownYRight;
	translateZDownYRight << 1, 0, 0, 0.75, 0, 1, 0, -1.25, 0, 0, 0.5, -1, 0, 0, 0, 1;
	TriMesh pillar3 = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cylinder.smf", translateZDownYRight);
	worldObjects.push_back(&pillar3);
	ShadeInfo si_pillar3 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_pillar3);
	*/
	
	
	//Scene 3
	
	//Lamp light
	
	Light worldLight = Light(POINT_LIGHT, Point3f(2, -0.5, 2), Vec3f(1.0, 1.0, 1.0), 0.5);
	worldLights.push_back(&worldLight);

	
	//Light lightbulb = Light(POINT_LIGHT, Point3f(0.6, 0.60, 0.09), Vec3f(1, 0.98, 0.82), 0.1);
	//worldLights.push_back(&lightbulb);
	
	Light lightBulb1 = Light(SPOT_LIGHT, Point3f(0.6, 0.60, 0.09), Vec3f(1, 0.98, 0.82), 0.1, Vec3f(0, 0.4, -1));
	worldLights.push_back(&lightBulb1);

	Light lightBulb2 = Light(SPOT_LIGHT, Point3f(0.6, 0.60, 0.09), Vec3f(1, 0.98, 0.82), 0.1, Vec3f(0, 0.4, -1));
	worldLights.push_back(&lightBulb2);

	Light lightBulb3 = Light(SPOT_LIGHT, Point3f(0.6, 0.60, 0.09), Vec3f(1, 0.98, 0.82), 0.1, Vec3f(0, 0.4, -1));
	worldLights.push_back(&lightBulb3);

	Sphere bulb = Sphere(0.15, Point3f(0.1, 0.45, 0.02));
	ShadeInfo bulbMat = ShadeInfo(Vec3f(1, 0.98, 0.57), Vec3f(1, 0.98, 0.57), Vec3f(1, 1, 1), 100.0, 0, 0, 1);
	bulb.setFullLightTransmission(true);
	worldObjects.push_back(&bulb);
	objectMaterials.push_back(&bulbMat);
	
	
	Matrix4f translateZDownScaleHalf;
	translateZDownScaleHalf << 0.35, 0, 0, -0.4,   0, 0.35, 0, -1.5,   0, 0, 0.35, 0,   0, 0, 0, 1;
	Matrix4f rotateMtx;
	rotateMtx << 0.866, -0.5, 0, 0, 0.5, 0.866, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //Rotate deg along z axis
	Matrix4f rotateMtx2;
	rotateMtx2 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1; //Rotate 90 deg along x axis
	Matrix4f product = rotateMtx * rotateMtx2 * translateZDownScaleHalf;
	ShadeInfo lamp = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	
	TriMesh pixarlamp = TriMesh("C:/Users/austi/Documents/Raytracer/A5/pixarLampMediumRes.smf", product);
	worldObjects.push_back(&pixarlamp);
	objectMaterials.push_back(&lamp);
	

	Matrix4f cubeToFloor;
	cubeToFloor << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 1; //Scale cube to make it a bridge
	Matrix4f translateZDown;
	//TODO: Bug here, the homogeneous translation matrix doesn't work as it should (-15 represents -2?)
	translateZDown << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -15, 0, 0, 0, 1;
	Matrix4f productBridge = cubeToFloor * translateZDown;
	TriMesh bridge = TriMesh("C:/Users/austi/Documents/Raytracer/A5/cube.smf", productBridge);
	//ShadeInfo si3 = ShadeInfo(Vec3f(.09, 0.25, 1), Vec3f(0.6, 1, 1), Vec3f(1, 1, 1), 10.0);
	worldObjects.push_back(&bridge);
	ShadeInfo si_bridge = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	objectMaterials.push_back(&si_bridge);

	Sphere s1 = Sphere(0.4, Point3f(1, 1.5, 0.5));
	ShadeInfo si1 = ShadeInfo(Vec3f(0.006, 0.06, 0.06), Vec3f(0.1, 0.1, 0.1), Vec3f(1, 1, 1), 30.0, 0, 0, 1);
	Texture moon;
	moon.path = "C:/Users/austi/Documents/Raytracer/Extra Credit/moon_512.jpg";
	moon.type = 1;
	Mat img = imread(moon.path, IMREAD_COLOR);
	if (img.empty())
	{
		std::cout << "Texture at " << moon.path << " failed to load";
		return 0;
	}
	moon.image = img;
	
	s1.setTexture(moon);
	worldObjects.push_back(&s1);
	objectMaterials.push_back(&si1);
	
	//Matrix4f translateMtx;
	//translateMtx << 0.3, 0, 0, -1,  0, 0.3, 0, -1,  0, 0, 0.3, 0.8,  0, 0, 0, 1;
	//ShadeInfo si2 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.5, 0.64, 0.76), Vec3f(1, 1, 1), 5.0, 0, 0, 1);
	//TriMesh icos = TriMesh("C:/Users/austi/Documents/Raytracer/A5/octahedron.smf", translateMtx); 
	//worldObjects.push_back(&icos);
	//objectMaterials.push_back(&si2);

	

	Sphere reflectiveBall = Sphere(0.5, Point3f(-0.1, -1.2, -1));
	ShadeInfo reflectiveBallMat = ShadeInfo(Vec3f(.08, 0.08, 0.1), Vec3f(0.1, 0.1, 0.15), Vec3f(1, 1, 1), 30, 1, 0, 1);
	worldObjects.push_back(&reflectiveBall);
	objectMaterials.push_back(&reflectiveBallMat);

	Sphere s3 = Sphere(0.2, Point3f(1, 1.6, -1.3));
	ShadeInfo si3 = ShadeInfo(Vec3f(.09, 0.1, .2), Vec3f(0.2, 0.3, 0.8), Vec3f(1, 1, 1), 30, 0, 0, 1);
	worldObjects.push_back(&s3);
	objectMaterials.push_back(&si3);

	Sphere ball = Sphere(0.2, Point3f(1.5, 1, -1.3));
	ShadeInfo ballSi = ShadeInfo(Vec3f(0.2, 0.2, 0.1), Vec3f(0.8, 0.8, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	worldObjects.push_back(&ball);
	objectMaterials.push_back(&ballSi);

	Matrix4f translateMtx2;
	translateMtx2 << 0.25, 0, 0, 1,   0, 0.25, 0, 0.7,  0, 0, 0.25, -1.2,  0, 0, 0, 1;
	ShadeInfo si5 = ShadeInfo(Vec3f(0.2, 0.1, 0.1), Vec3f(0.8, 0.4, 0.4), Vec3f(1, 1, 1), 10, 0, 0, 1);
	TriMesh supEllipse = TriMesh("C:/Users/austi/Documents/Raytracer/A5/icos.smf", translateMtx2);
	worldObjects.push_back(&supEllipse);
	objectMaterials.push_back(&si5);
	

	



	//Calculate preliminary values
	//(left handed coordinate system)
	Point3f camCoordX = cam_view_out.cross(cam_view_up);
	Point3f camCoordY = camCoordX.cross(cam_view_out);
	Point3f camCoordZ = cam_view_out;
	normalize(camCoordX);
	normalize(camCoordY);
	normalize(camCoordZ);

	float imgPlaneW = 2 * d * tan(cam_horiz_angle / 2);
	float imgPlaneH = imgPlaneW * (yRes / xRes);

	//Calculate top left pixel in screen

	Point3f vectorSum = d * (Vec3f)camCoordZ - (imgPlaneW / 2) * (Vec3f)camCoordX + (imgPlaneH / 2) * (Vec3f)camCoordY;
	Vec3f test;

	Point3f topLeftPxl = cam_loc + vectorSum;
	Ray currentRay;
	float t_min;
	float best_t_min = -1;
	ShadeInfo* temp_si;
	ShadeInfo* si = new ShadeInfo;

	auto start = high_resolution_clock::now();
	
	//We are doing one more row and column than the resolution calls for (xRes + 1 and yRes + 1) to enable us to use adaptive subdivision
	for (int i = 0; i < xRes; i++)
	{
		cout << i;
		for (int j = 0; j < yRes; j++)
		{
			if ((i == 128) && (j == 66))
			{
				int test = true;
			}

			Vec3b color = getColorFromComputedRay(i, j, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
			//rays++;
			image.at<cv::Vec3b>(i, j) = color;
		}
	}
	auto stop = high_resolution_clock::now();
	//screen
		//for each pixel
		//	if intersection - shade point

	auto duration = duration_cast<microseconds>(stop - start);
	cout << "\n" << duration.count();
	cv::imshow("generated", image);
	cv::imwrite("C:/Users/austi/Documents/Raytracer/output/render512.png", image);
	
	cv::waitKey(0);
	
	
	//Perform adaptive supersampling algorithm
	//TODO: Make this algorithm recursive
	start = high_resolution_clock::now();
	float tolerance = 0.05;
	
	cv::Mat3b newImage = cv::Mat3b(xRes, yRes, cv::Vec3d{ 0,0,0 });
	cv::Mat3b adaptiveSuperSampLevelMap = cv::Mat3b(xRes, yRes, cv::Vec3d{ 0,0,0 });
	for (int i = 0; i < xRes - 1; i++)
	{
		cout << i;
		for (int j = 0; j < yRes - 1; j++)
		{


			Vec3b D = image.at<cv::Vec3b>(i, j); //topLeftCorner
			Vec3b E = image.at<cv::Vec3b>(i + 1, j); //topRightCorner
			Vec3b A = image.at<cv::Vec3b>(i, j + 1); //bottomLeftCorner
			Vec3b B = image.at<cv::Vec3b>(i + 1, j + 1); //bottomRightCorner

			bool toleranceTest1 = aSupSampToleranceTest(A, B, tolerance);
			bool tTest2 = aSupSampToleranceTest(A, D, tolerance);
			bool tTest3 = aSupSampToleranceTest(E, D, tolerance);
			bool tTest4 = aSupSampToleranceTest(E, B, tolerance);




			if (!(toleranceTest1 && tTest2 && tTest3 && tTest4))
			{




				//Create array of potential colors
				Vec3b level1[5];
				Vec3b br[5]; //level2


				//Shoot 5 additional rays: 4 through midtpoint of sides, one through center
				level1[0] = getColorFromComputedRay(i + 0.5, j, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
				level1[1] = getColorFromComputedRay(i + 1, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
				level1[2] = getColorFromComputedRay(i + 0.5, j + 1, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
				level1[3] = getColorFromComputedRay(i, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
				level1[4] = getColorFromComputedRay(i + 0.5, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights); //midpoint
				//rays += 5;
				toleranceTest1 = aSupSampToleranceTest(level1[2], level1[1], tolerance);
				tTest2 = aSupSampToleranceTest(level1[2], level1[3], tolerance);
				tTest3 = aSupSampToleranceTest(level1[0], level1[3], tolerance);
				tTest4 = aSupSampToleranceTest(level1[0], level1[1], tolerance);

				Vec3f color_0;
				Vec3f color_1;
				Vec3f color_2;
				Vec3f color_3;
				if (!(toleranceTest1 && tTest2 && tTest3 && tTest4))
				{
					if (!toleranceTest1)
					{
						//The subpixels are not in tolerance

						//Shoot 5 additional rays: 4 through midtpoint of sides, one through center
						br[0] = getColorFromComputedRay(i + 0.75, j, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[1] = getColorFromComputedRay(i + 1, j + 0.25, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[2] = getColorFromComputedRay(i + 0.75, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[3] = getColorFromComputedRay(i + 0.5, j + 0.25, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[4] = getColorFromComputedRay(i + 0.75, j + 0.25, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights); //midpoint
						//rays += 5;

						//CHANGE THE BELOW TO GO DOWN TO LEVEL 2
						color_1 = (Vec3f(br[3]) + Vec3f(br[4]) + Vec3f(br[0]) + Vec3f(level1[2])) / 4;
						color_1 += (Vec3f(br[4]) + Vec3f(br[1]) + Vec3f(E) + Vec3f(br[0])) / 4;
						color_1 += (Vec3f(br[2]) + Vec3f(level1[1]) + Vec3f(br[1]) + Vec3f(br[4])) / 4;
						color_1 += (Vec3f(level1[4]) + Vec3f(br[2]) + Vec3f(br[4]) + Vec3f(br[3])) / 4;
						color_1 = color_1 / 4;

					}
					if (!tTest2)
					{
						//Shoot 5 additional rays: 4 through midtpoint of sides, one through center
						br[0] = getColorFromComputedRay(i + 0.25, j, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[1] = getColorFromComputedRay(i + 0.5, j + 0.25, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[2] = getColorFromComputedRay(i + 0.25, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[3] = getColorFromComputedRay(i, j + 0.25, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[4] = getColorFromComputedRay(i + 0.25, j + 0.25, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights); //midpoint
						//rays += 5;

						//CHANGE THE BELOW TO GO DOWN TO LEVEL 2
						color_0 = (Vec3f(br[3]) + Vec3f(br[4]) + Vec3f(br[0]) + Vec3f(D)) / 4;
						color_0 += (Vec3f(br[4]) + Vec3f(br[1]) + Vec3f(level1[2]) + Vec3f(br[0])) / 4;
						color_0 += (Vec3f(br[2]) + Vec3f(level1[4]) + Vec3f(br[1]) + Vec3f(br[4])) / 4;
						color_0 += (Vec3f(level1[3]) + Vec3f(br[2]) + Vec3f(br[4]) + Vec3f(br[3])) / 4;
						color_0 = color_0 / 4;
					}
					if (!tTest3)
					{
						br[0] = getColorFromComputedRay(i + 0.25, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[1] = getColorFromComputedRay(i + 0.5, j + 0.75, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[2] = getColorFromComputedRay(i + 0.25, j + 1, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[3] = getColorFromComputedRay(i, j + 0.75, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[4] = getColorFromComputedRay(i + 0.25, j + 0.75, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights); //midpoint
						//rays += 5;

						//CHANGE THE BELOW TO GO DOWN TO LEVEL 2
						color_3 = (Vec3f(br[3]) + Vec3f(br[4]) + Vec3f(br[0]) + Vec3f(level1[3])) / 4;
						color_3 += (Vec3f(br[4]) + Vec3f(br[1]) + Vec3f(level1[4]) + Vec3f(br[0])) / 4;
						color_3 += (Vec3f(br[2]) + Vec3f(level1[0]) + Vec3f(br[1]) + Vec3f(br[4])) / 4;
						color_3 += (Vec3f(A) + Vec3f(br[2]) + Vec3f(br[4]) + Vec3f(br[3])) / 4;
						color_3 = color_3 / 4;

					}
					if (!tTest4)
					{
						//Shoot 5 additional rays: 4 through midtpoint of sides, one through center
						br[0] = getColorFromComputedRay(i + 0.75, j + 0.5, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[1] = getColorFromComputedRay(i + 1, j + 0.75, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[2] = getColorFromComputedRay(i + 0.75, j + 1, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[3] = getColorFromComputedRay(i + 0.5, j + 0.75, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights);
						br[4] = getColorFromComputedRay(i + 0.75, j + 0.75, topLeftPxl, imgPlaneW, imgPlaneH, xRes, yRes, camCoordX, camCoordY, cam_loc, worldObjects, si, objectMaterials, worldLights); //midpoint
						//rays += 5;

						//CHANGE THE BELOW TO GO DOWN TO LEVEL 2
						color_2 = (Vec3f(br[3]) + Vec3f(br[4]) + Vec3f(br[0]) + Vec3f(level1[4])) / 4;
						color_2 += (Vec3f(br[4]) + Vec3f(br[1]) + Vec3f(level1[1]) + Vec3f(br[0])) / 4;
						color_2 += (Vec3f(br[2]) + Vec3f(B) + Vec3f(br[1]) + Vec3f(br[4])) / 4;
						color_2 += (Vec3f(level1[0]) + Vec3f(br[2]) + Vec3f(br[4]) + Vec3f(br[3])) / 4;
						color_2 = color_2 / 4;
					}
					//Set color values that have not been set yet
					//Because a ray will never return a color of pure black (0), a color with (0,0,0) is not set.
					if (color_0(0) == 0 && color_0(1) == 0 && color_0(2) == 0)
					{
						color_0 = (Vec3f(level1[3]) + Vec3f(level1[4]) + Vec3f(level1[2]) + Vec3f(D)) / 4;
					}
					if (color_1(0) == 0 && color_1(1) == 0 && color_1(2) == 0)
					{
						color_1 = (Vec3f(level1[4]) + Vec3f(level1[1]) + Vec3f(E) + Vec3f(level1[2])) / 4;
					}
					if (color_2(0) == 0 && color_2(1) == 0 && color_2(2) == 0)
					{
						color_2 = (Vec3f(level1[0]) + Vec3f(B) + Vec3f(level1[1]) + Vec3f(level1[4])) / 4;
					}
					if (color_3(0) == 0 && color_3(1) == 0 && color_3(2) == 0)
					{
						color_3 = (Vec3f(A) + Vec3f(level1[0]) + Vec3f(level1[4]) + Vec3f(level1[3])) / 4;
					}

					Vec3b color = Vec3b((color_0 + color_1 + color_2 + color_3) / 4);
					newImage.at<cv::Vec3b>(i, j) = color;
					adaptiveSuperSampLevelMap.at<cv::Vec3b>(i, j) = Vec3b(Vec3d(150, 150, 150));
				}
				else
				{
					//The subpixels are in tolerance

					Vec3f color_0 = (Vec3f(level1[3]) + Vec3f(level1[4]) + Vec3f(level1[2]) + Vec3f(D)) / 4;
					Vec3f color_1 = (Vec3f(level1[4]) + Vec3f(level1[1]) + Vec3f(E) + Vec3f(level1[2])) / 4;
					Vec3f color_2 = (Vec3f(level1[0]) + Vec3f(B) + Vec3f(level1[1]) + Vec3f(level1[4])) / 4;
					Vec3f color_3 = (Vec3f(A) + Vec3f(level1[0]) + Vec3f(level1[4]) + Vec3f(level1[3])) / 4;
					Vec3b color = Vec3b((color_0 + color_1 + color_2 + color_3)/4);
					newImage.at<cv::Vec3b>(i, j) = color;
					adaptiveSuperSampLevelMap.at<cv::Vec3b>(i, j) = Vec3b(Vec3d(50, 50, 50));
				}


			}
			else
			{
				//All corner colors are within tolerance - get average of 4 corners for pixel color
				Vec3b color = Vec3b( (Vec3f(A) + Vec3f(B) + Vec3f(D) + Vec3f(E))/4 );
				newImage.at<cv::Vec3b>(i, j) = color;
				adaptiveSuperSampLevelMap.at<cv::Vec3b>(i, j) = Vec3b(0,0,0);
			}


		}
	}
	 stop = high_resolution_clock::now();
	 duration = duration_cast<microseconds>(stop - start);
	cout << "\n" << duration.count();

	cv::imshow("generated", newImage);
	cv::imwrite("C:/Users/austi/Documents/Raytracer/output/adaptiveSuperSample.png", newImage);
	cv::waitKey(0);
	cv::imshow("generated", adaptiveSuperSampLevelMap);
	cv::imwrite("C:/Users/austi/Documents/Raytracer/output/adaptiveSuperSample_levelMap.png", adaptiveSuperSampLevelMap);
	cout << "\n" << rays;
	cv::waitKey(0);
	
	return 0;
}

