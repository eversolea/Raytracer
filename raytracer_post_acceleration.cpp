# define PI           3.14159265358979323846 
# define FLOAT_MAX   340282346638528859811704183484516925439
# define POINT_LIGHT 0
# define IMAGE_RES_X 1024
# define IMAGE_RES_Y 1024
# define TRIMESH 1
# define SPHERE 2

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono;
using namespace cv;
using namespace std;
using namespace Eigen;

//TODO: take the BVH interesction code outside of testintersect in TriMesh, so we dont need this global variable below to keep track of all TriMeshes
vector<vector<Vec3f>> allTriMeshNormals;

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

using std::string;
using std::vector;


struct Ray {
	Point3f o; //origin
	Point3f d; //direction
};

struct Tris {
	Point3f vtx1;
	Point3f vtx2;
	Point3f vtx3;
	Point3f faceTopology; //To keep track of the topology for normal calculation in TriMesh.testIntersect
	Point3f bBox_bottomLeft;
	Point3f bBox_topRight;
	Point3f centroid;
	int owner; //To keep track of which Trimesh this tri belongs to, neccesary for current Bounding Volume Heirarchy implementation
};

struct boundingBox {
	//TODO: Set better initial starting bbox
	Point3f bottomLeft = Point3f(99999, 99999, 99999);
	Point3f topRight = Point3f(-99999, -99999, -99999);
};

struct BVH {
	boundingBox bbox;
	vector<Tris> obj_list;
	int sort_axis;
	BVH* lPtr = NULL;
	BVH* rPtr = NULL;

	BVH(int sortAxis=0)
	{
		sort_axis = sortAxis;
	}
};

BVH pBvh;

vector<Tris> mergeVectors(const vector<Tris>& vector1, const vector<Tris>& vector2)
{
	vector<Tris> merged = vector1;

	merged.insert(merged.end(), vector2.begin(), vector2.end());

	return merged;
}

//TODO: Can we use cv.boundingRect(array) instead of our own code?
Point3f saveBBoxValues(Point3f a, Point3f b, bool lowest)
{
	if (lowest)
	{
		//we are attempting to save the bottom left 3D bbox corner into point a. (lowest corner)
		//We will compare point b to a to see if an y of b's values are low enough to change this bbox corner
		if (a.x < b.x) { a.x = b.x; }
		if (a.y < b.y) { a.y = b.y; }
		if (a.z < b.z) { a.z = b.z; }
	}
	else
	{
		//we are attempting to save the top right 3D bbox corner into point a. (highest corner)
		if (a.x > b.x) { a.x = b.x; }
		if (a.y > b.y) { a.y = b.y; }
		if (a.z > b.z) { a.z = b.z; }
	}
	return a;
}



class ShadeInfo {
	//Test for shadows
	//Compute Diffuse color
	//possible recurse for specular and translucency

public:
	ShadeInfo(); 
	ShadeInfo(Vec3f ambientC, Vec3f diffuseC, Vec3f specularC, int shineC);

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
	
};

ShadeInfo::ShadeInfo()
{
	hit_an_object = false;
	color = Vec3b(0, 0, 0);
	specularColor = Vec3b(0.8, 0.8, 0.8);
	diffuseColor = Vec3b(0.6,0.6,0.6);
	ambientColor = Vec3b(0.3, 0.3, 0.3);
	shininess = 80;
}

ShadeInfo::ShadeInfo(Vec3f ambientC, Vec3f diffuseC, Vec3f specularC, int shineC)
{
	hit_an_object = false;
	color = Vec3b(0, 0, 0);
	specularColor = specularC;
	diffuseColor = diffuseC;
	ambientColor = ambientC;
	shininess = shineC;
}

class Light {
public:
	Light(int type, Point3f o, Vec3f i, float attenuation);

	int type = 0; //0 - Point Light
	Point3f origin = Point3f(0, 0, 0);
	Vec3f colorIntensity = Vec3f(255, 255, 255);
	float att = 3;

};


Light::Light(int type, Point3f o, Vec3f i, float attenuation)
{
	if (type == 0)
	{
		origin = o;
		colorIntensity = i;
		att = attenuation;
	}
}



bool rayBoxIntersectionMissed(float t1, float t2, float& tNear, float& tFar)
{
	float swapTemp;
	if (t1 > t2) { swapTemp = t1; t1 = t2; t2 = swapTemp; } //swap t1 and t2
	if (t1 > tNear) { tNear = t1; }
	if (t2 < tFar) { tFar = t2; }
	if (tNear > tFar) { return true; } //Box is missed
	if (tFar < 0) { return true; } //Box is behind eyepoint
	return false;
}

//xsect_pt and t aren't being implemented right now.
std::vector<Tris> intersect_BVH(BVH* node, Ray ray, Point3f& xsect_pt, float& t)
{
	//TODO: should we combine testIntersect with this function? 
	//Instead of using this function to filter out which BVH box to use and passing those tris on
	std::vector<Tris> empty;
	std::vector<Tris> output;

	if (node->lPtr == nullptr)
	{
		//intersect geometry and return nearest xset_pt & t
		//or... just return the list of tris for testinersect to do!
		return node->obj_list;
	}
	//Intersect ray with lptr and suboxes in x, y, and z dimensions
	float tNear = -9999;
	float tFar = 9999;
	bool boxMissed1 = false;
	bool boxMissed2 = false;
	bool swapValue = false;
	float subbox1_tNear, subbox1_tFar, subbox2_tNear, subbox2_tFar, tSwapValue;

	//intersect ray with node->lptr subbox
	float t1 = (node->lPtr->bbox.bottomLeft.x - ray.o.x) / ray.d.x;
	float t2 = (node->lPtr->bbox.topRight.x - ray.o.x) / ray.d.x;
	boxMissed1 |= rayBoxIntersectionMissed(t1, t2, tNear, tFar);

	t1 = (node->lPtr->bbox.bottomLeft.y - ray.o.y) / ray.d.y;
	t2 = (node->lPtr->bbox.topRight.y - ray.o.y) / ray.d.y;
	boxMissed1 |= rayBoxIntersectionMissed(t1, t2, tNear, tFar);

	t1 = (node->lPtr->bbox.bottomLeft.z - ray.o.z) / ray.d.z;
	t2 = (node->lPtr->bbox.topRight.z - ray.o.z) / ray.d.z;
	boxMissed1 |= rayBoxIntersectionMissed(t1, t2, tNear, tFar);

	float lPtr_t;
	if (tNear < 0) { lPtr_t = tFar; }
	else { lPtr_t = tNear; }
	subbox1_tFar = tFar;
	subbox1_tNear = tNear;


	tNear = -9999;
	tFar = 9999;

	//intersect ray with node->rptr subbox
	t1 = (node->rPtr->bbox.bottomLeft.x - ray.o.x) / ray.d.x;
	t2 = (node->rPtr->bbox.topRight.x - ray.o.x) / ray.d.x;
	boxMissed2 |= rayBoxIntersectionMissed(t1, t2, tNear, tFar);

	t1 = (node->rPtr->bbox.bottomLeft.y - ray.o.y) / ray.d.y;
	t2 = (node->rPtr->bbox.topRight.y - ray.o.y) / ray.d.y;
	boxMissed2 |= rayBoxIntersectionMissed(t1, t2, tNear, tFar);

	t1 = (node->rPtr->bbox.bottomLeft.z - ray.o.z) / ray.d.z;
	t2 = (node->rPtr->bbox.topRight.z - ray.o.z) / ray.d.z;
	boxMissed2 |= rayBoxIntersectionMissed(t1, t2, tNear, tFar);

	float rPtr_t;
	if (tNear < 0) { rPtr_t = tFar; }
	else { rPtr_t = tNear; }
	subbox2_tFar = tFar;
	subbox2_tNear = tNear;

	//If not hits on either box
	if (boxMissed1 && boxMissed2) { return empty; }

	//subox with nearest t  bcomes subbox1
	//So if t from leftbox < t from rightbox, then subbox1 = lPTr
	BVH* subbox1 = node->lPtr;
	BVH* subbox2 = node->rPtr;
	if (lPtr_t > rPtr_t) {
		//if t from rightbox is larger, subbox = rPtr.
		subbox1 = node->rPtr;
		subbox2 = node->lPtr;
		//Also swap boxmissed1 and boxmissed2 for readability so boxmissed1 correlates to subbox1
		swapValue = boxMissed1; boxMissed1 = boxMissed2; boxMissed2 = swapValue;
		tSwapValue = subbox1_tFar; subbox1_tFar = subbox2_tFar; subbox2_tFar = tSwapValue;
		tSwapValue = subbox1_tNear; subbox1_tNear = subbox2_tNear; subbox2_tNear = tSwapValue;
	}

	output = mergeVectors(output, intersect_BVH(subbox1, ray, xsect_pt, t));
	//If it hit subbox2
	if (!boxMissed2)
	{
		//does subbox1 have no xsect | Are bounding boxes overlapping
		//If xsect_pt == null | t2near <= t1far //If (ray hit subbox 2)
		if (boxMissed1 || subbox2_tNear <= subbox1_tFar)
		{
			output = mergeVectors(output, intersect_BVH(subbox2, ray, xsect_pt, t));
			//Intersect BVH_(subox2, ray, xset_pt2, t2)
			//Set nearest xset_pt/t 
		}
	}

	//This function doesn't return at the end becasue it is meant to recurse until 
	//one of the subboxes on lower levels returns its whole obj_list.
	return output;
}

/*
Does ray intersect box ?
intersect_BVH(box, ray, xsect_pt, t)
If no more subboxes
Intersect geometry and return nearest xsect_pt & t
Intersect ray with both subboxes
No hits : return xsect_pt = Null;
Sort t’s
Call subbox of nearest t subbox1
intersect_BVH(subbox1, ray, xsect_pt, t)
If hit_subbox2 ?
If xsect_pt == Null || t2near <= t1far
intersect_BVH(subbox2, ray, xsect_pt, t)
Set nearest xsect_pt and t
Return
*/



class Entity {
public:
	virtual bool testIntersect(BVH* pBvh, const Ray& ray, float& t_min, ShadeInfo& si) {
		return false;
	};
	virtual int getType()
	{
		return 0;
	};
	virtual vector<Tris> getTris()
	{
		vector<Tris> t;
		return t;
	}
	virtual vector<Vec3f> getTriMeshNormals()
	{
		vector<Vec3f> t;
		return t;
	}

};



class Sphere : public Entity {
public:
	Sphere(); //Create sphere with radius 1 at origin with no transformations
	Sphere(float r, Point3f o);
	Sphere(float r, Point3f o, Matrix4f tMtx);

	bool testIntersect(BVH* pBvh, const Ray& ray, float& t_min, ShadeInfo& si);
	int getType();
	vector<Tris> getTris();
	vector<Vec3f> getNormals();

private:
	float radius;
	Point3f origin;
	Matrix4f transformMtx; //This should be moved to become an ellipsoid as it can be scaled and rotated
	vector<Point3f> normal;
	static const double kEpsilon; //This is to provide the foundation for shadows
};

int Sphere::getType()
{
	return SPHERE;
}

vector<Tris> Sphere::getTris()
{
	//This is meant for TriMesh: return empty vector
	vector<Tris> t;
	return t;
}

vector<Vec3f> Sphere::getNormals()
{
	//Sphere doesnt really have a normal... easily computable
	//TODO: member variable normals is a placeholder
	vector<Vec3f> t;
	return t;
}

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

bool Sphere::testIntersect(BVH* pBvh, const Ray& ray, float& t_min, ShadeInfo& si)
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

	//Take the smallest t as our hit point
	if (t_0 <= t_1)
	{
		t_min = t_0;
	}
	else
	{
		t_min = t_1;
	}
	//TODO: Make this line more readable
	si.local_hit_point = ray.o + (t_min * ray.d);
	//si.normal = slide 43, lecture 1 //We will get to this when we do phong shading
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
	TriMesh(string path, int owner); //Create triangle mesh using the .smf found in path parameter
	TriMesh(string path, Matrix4f tMtx, int owner); //Create a triangle mesh with transformations applied to it

	bool testIntersect(BVH* pBvh, const Ray& ray, float& t_min, ShadeInfo& si);
	int getType();
	vector<Tris> getTris();
	vector<Vec3f> getNormals();

private:
	string path;
	vector<Tris> triangles;
	vector<Vec3f> normals;
	static const double kEpsilon; //This is to provide the foundation for shadows
};

int TriMesh::getType()
{
	return TRIMESH;
}


vector<Tris> TriMesh::getTris()
{
	return triangles;
}

vector<Vec3f> TriMesh::getNormals()
{
	return normals;
}

TriMesh::TriMesh(string path, int owner)
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
			//Calculate bounding box information for this triangle
			tri.bBox_bottomLeft = saveBBoxValues(tri.vtx1, tri.vtx2, 0);
			tri.bBox_bottomLeft = saveBBoxValues(tri.bBox_bottomLeft, tri.vtx3, 0);
			tri.bBox_topRight = saveBBoxValues(tri.vtx1, tri.vtx2, 1);
			tri.bBox_topRight = saveBBoxValues(tri.bBox_topRight, tri.vtx3, 1);
			tri.centroid = Point3f((tri.bBox_bottomLeft.x + tri.bBox_topRight.x) / 2,
								   (tri.bBox_bottomLeft.y + tri.bBox_topRight.y) / 2,
								   (tri.bBox_bottomLeft.z + tri.bBox_topRight.z) / 2);
			tri.owner = owner;
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
}

TriMesh::TriMesh(string path, Matrix4f tMtx, int owner)
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

			//Calculate bounding box information for this triangle
			tri.bBox_bottomLeft = saveBBoxValues(tri.vtx1, tri.vtx2, 0);
			tri.bBox_bottomLeft = saveBBoxValues(tri.bBox_bottomLeft, tri.vtx3, 0);
			tri.bBox_topRight = saveBBoxValues(tri.vtx1, tri.vtx2, 1);
			tri.bBox_topRight = saveBBoxValues(tri.bBox_topRight, tri.vtx3, 1);
			tri.centroid = Point3f((tri.bBox_bottomLeft.x + tri.bBox_topRight.x) / 2,
				(tri.bBox_bottomLeft.y + tri.bBox_topRight.y) / 2,
				(tri.bBox_bottomLeft.z + tri.bBox_topRight.z) / 2);
			tri.owner = owner;
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
}

//I attempted use Cramers rule here. 
bool TriMesh::testIntersect(BVH* pBvh, const Ray& ray, float& t_min, ShadeInfo& si)
{

	float best_a = -1; //Barycentric coordinates of closest tri intersection
	float best_b = -1;
	float best_y = -1;
	int best_tri = -1; //Tri index for closest tri intersection 
	float best_t_min = -1; //ray's t value for closest tri intersection

	Point3f xsect_pt; float t;
	std::vector<Tris> output = intersect_BVH(pBvh, ray, xsect_pt, t);


	for (int i = 0; i < output.size(); i++)
	{
		//TODO: Do this through a library function
		//TODO: Run as its own exe outside of IDE if you use eigen?
	
		
		float a_ = output[i].vtx1.x - output[i].vtx2.x;
		float b_ = output[i].vtx1.x - output[i].vtx3.x;
		float c_ = ray.d.x;
		float d_ = output[i].vtx1.y - output[i].vtx2.y;
		float e_ = output[i].vtx1.y - output[i].vtx3.y;
		float f_ = ray.d.y;
		float g_ = output[i].vtx1.z - output[i].vtx2.z;
		float h_ = output[i].vtx1.z - output[i].vtx3.z;
		float i_ = ray.d.z;


		float detA = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);

		a_ = output[i].vtx1.x - ray.o.x;
		b_ = output[i].vtx1.x - output[i].vtx3.x;
		c_ = ray.d.x;
		d_ = output[i].vtx1.y - ray.o.y;
		e_ = output[i].vtx1.y - output[i].vtx3.y;
		f_ = ray.d.y;
		g_ = output[i].vtx1.z - ray.o.z;
		h_ = output[i].vtx1.z - output[i].vtx3.z;
		i_ = ray.d.z;

		float detB = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);
		float b = detB / detA; //beta  (barymetric weight)

		a_ = output[i].vtx1.x - output[i].vtx2.x;
		b_ = output[i].vtx1.x - ray.o.x;
		c_ = ray.d.x;
		d_ = output[i].vtx1.y - output[i].vtx2.y;
		e_ = output[i].vtx1.y - ray.o.y;
		f_ = ray.d.y;
		g_ = output[i].vtx1.z - output[i].vtx2.z;
		h_ = output[i].vtx1.z - ray.o.z;
		i_ = ray.d.z;
		Matrix3f y_mtx;
		y_mtx << a_, b_, c_,
			d_, e_, f_,
			g_, h_, i_;
		float detY = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);
		float y = detY / detA; //gamma (barymetric weight)

		a_ = output[i].vtx1.x - output[i].vtx2.x;
		b_ = output[i].vtx1.x - output[i].vtx3.x;
		c_ = output[i].vtx1.x - ray.o.x;
		d_ = output[i].vtx1.y - output[i].vtx2.y;
		e_ = output[i].vtx1.y - output[i].vtx3.y;
		f_ = output[i].vtx1.y - ray.o.y;
		g_ = output[i].vtx1.z - output[i].vtx2.z;
		h_ = output[i].vtx1.z - output[i].vtx3.z;
		i_ = output[i].vtx1.z - ray.o.z;


		float detT = fastDet(a_, b_, c_, d_, e_, f_, g_, h_, i_);
		
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
				/*
				//We have an intersection with triangle 'i'!
				si.local_hit_point = ray.o + (t_min * ray.d);
				//si.normal //We will get to this when we do phong shading
				si.hit_an_object = true;
				si.color = Vec3b{ 255, 255, 255 };
				//TODO: Optimize this. Will store the t_min of the intersection from the first found 'i'
				t_min = t;
				return true;
				*/
			}
		}

	}

	if (best_t_min != -1)
	{
		//We have an intersection with triangle 'i'!
		t_min = best_t_min;
		si.local_hit_point = ray.o + (t_min * ray.d);
		//Using the barycentric coordinates calculated in the intersection finding code above to help us get the normal
		
		//TODO: take the testintersect code out since it is dependent on other TriMeshes and requires a global variable due to BVH - not polymorphic
		vector<Vec3f> trimeshNormals = allTriMeshNormals[output[best_tri].owner];
		si.normal = Vec3f(best_a * trimeshNormals[output[best_tri].faceTopology.x] + best_b * trimeshNormals[output[best_tri].faceTopology.y] + best_y * trimeshNormals[output[best_tri].faceTopology.z]);
		si.hit_an_object = true;
		si.color = Vec3b{ 255, 255, 255 };
		return true;
	}
	si.hit_an_object = false;
	si.color = Vec3b{ 0,0,0 }; //Background color
	return false;
}



bool sortTrisByCentroidX(Tris a, Tris b)
{
	return a.centroid.x < b.centroid.x;
}

bool sortTrisByCentroidY(Tris a, Tris b)
{
	return a.centroid.y < b.centroid.y;
}

bool sortTrisByCentroidZ(Tris a, Tris b)
{
	return a.centroid.z < b.centroid.z;
}


BVH* Make_BVH(vector<Tris>& object_list, int sort_axis)
{
	BVH* data = new BVH(sort_axis);

	//Loop through all triangles in list to create the bbox of global bbox of the current scope
	for (int i = 0; i < object_list.size(); i++)
	{
		data->bbox.bottomLeft = saveBBoxValues(data->bbox.bottomLeft, object_list[i].bBox_bottomLeft, 0);
		data->bbox.topRight = saveBBoxValues(data->bbox.topRight, object_list[i].bBox_topRight, 1);
	}
	//TODO: combine intersect_BVH with test_intersect, then take the below code out
	data->obj_list = object_list;
	
	if (object_list.size() < 10)
	{
		data->obj_list = object_list;
	}
	else
	{
		if ((sort_axis % 3) == 0)
		{
			sort(object_list.begin(), object_list.end(), sortTrisByCentroidX);
		}
		else if ((sort_axis % 3) == 1)
		{
			sort(object_list.begin(), object_list.end(), sortTrisByCentroidY);
		}
		else
		{
			sort(object_list.begin(), object_list.end(), sortTrisByCentroidZ);
		}
		std::size_t const midpoint = object_list.size() / 2;
		std::vector<Tris> left_obj_list(object_list.begin(), object_list.begin() + midpoint);
		std::vector<Tris> right_obj_list(object_list.begin() + midpoint, object_list.end());

		data->lPtr = Make_BVH(left_obj_list, sort_axis+1);
		data->rPtr = Make_BVH(right_obj_list, sort_axis+1);
	}
	return data;

}

/*
sort_axis = 0;
Make_BVH(object_list, sort_axis, ptr)
struct.bbox = BoundingBox(object_list);
If # of objects < Threshold
	struct.obj_list = object_list
	Else
	If((sort_axis % 3) == 0) Sort object centroids in X
	ElseIf((sort_axis % 3) == 1)
	Sort object centroids in Y
	Else
	Sort object centroids in Z
	Split sorted list into two halves
	Make_BVH(left_obj_list, sort_axis++, lptr)
	Make_BVH(right_obj_list, sort_axis++, rptr)
	struct.lptr = lptr; struct.rptr = rptr;
ptr = &struct;
Return
*/



int main(int argc, char** argv)
{

	//Starting values
	Point3f cam_loc = Point3f(3, 0, 0);
	Point3f cam_view_out = Point3f(-1, 0, 0);
	Point3f cam_view_up = Point3f(0, 1, 0);
	float cam_horiz_angle = 56 * PI / 180;
	int xRes = IMAGE_RES_X;
	int yRes = IMAGE_RES_Y;
	float d = 3;
	cv::Mat3b image = cv::Mat3b(xRes, yRes, cv::Vec3f{ 0,0,0 });
	

	//Setup object,material, and light vectors
	vector<Entity*> worldObjects;
	vector<ShadeInfo*> objectMaterials;
	vector<Light*> worldLights;

	//Add lights to scene
	Light l1 = Light(POINT_LIGHT, Point3f(3, 2, 0), Vec3f(1.0, 1.0, 1.0), 0.5);
	worldLights.push_back(&l1);

	//Light l2 = Light(POINT_LIGHT, Point3f(3, -2, 0), Vec3f(1.0, 0.5, 0.5), 0.5);
	//worldLights.push_back(&l2);

	/* //Scene 1

	Sphere s2 = Sphere(0.75, Point3f(0, 1, 1));
	ShadeInfo randSphere1 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.7, 0.7, 0.7), Vec3f(1, 1, 1), 60.0);
	worldObjects.push_back(&s2);
	objectMaterials.push_back(randSphere1);

	TriMesh bunny = TriMesh("C:/Users/austi/Documents/School/Spring 2022/Advanced_Computer_Graphics/A2/Models/octahedron.smf");
	ShadeInfo si2 = ShadeInfo(Vec3f(0.2, 0.1, 0.01), Vec3f(0.8, 0.4, 0.01), Vec3f(1, 1, 1), 1.0);
	worldObjects.push_back(&bunny);
	objectMaterials.push_back(si2);

	Sphere s3 = Sphere(0.6, Point3f(0, -1, 1));
	ShadeInfo randSphere2 = ShadeInfo(Vec3f(0.01, 0.1, 0.3), Vec3f(0.2, 0.4, 0.8), Vec3f(1, 1, 1), 30.0);
	worldObjects.push_back(&s3);
	objectMaterials.push_back(randSphere2);

	Matrix4f translate;
	translate << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -1, 0, 0, 0, 1; //Scale cube to make it a bridge
	TriMesh cube = TriMesh("C:/Users/austi/Documents/School/Spring 2022/Advanced_Computer_Graphics/A2/Models/cube.smf", translate);
	ShadeInfo si4 = ShadeInfo(Vec3f(.09, 0.25, 1), Vec3f(0.6, 1, 1), Vec3f(1, 1, 1), 1);
	worldObjects.push_back(&cube);
	objectMaterials.push_back(si4);
	*/

	/* */
	//Add objects to scene
	//Guilty spark's center
	/*
*/	Sphere s1 = Sphere(0.4, Point3f(0, 0, 0));
	ShadeInfo si1 = ShadeInfo(Vec3f(0.3, 0.3, 0.3), Vec3f(0.5, 0.64, 0.76), Vec3f(1, 1, 1), 5.0);
	worldObjects.push_back(&s1);
	objectMaterials.push_back(&si1);

	
	//2 random spheres
	Sphere s2 = Sphere(0.25, Point3f(0, -1.2, 1.3));
	ShadeInfo randSphere1 = ShadeInfo(Vec3f(0.01, 0.1, 0.3), Vec3f(0.1, 0.2, 0.4), Vec3f(1, 1, 1), 10.0);
	worldObjects.push_back(&s2);
	objectMaterials.push_back(&randSphere1);
	
	Sphere s3 = Sphere(0.25, Point3f(0, 1.2, 1.3));
	ShadeInfo randSphere2 = ShadeInfo(Vec3f(0.01, 0.1, 0.3), Vec3f(0.6, 1, 1), Vec3f(1, 1, 1), 80.0);
	worldObjects.push_back(&s3);
	objectMaterials.push_back(&randSphere2);

	Sphere s4 = Sphere(0.3, Point3f(0, 0, 1.3));
	ShadeInfo randSphere3 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10);
	worldObjects.push_back(&s4);
	objectMaterials.push_back(&randSphere3);
	
	
	Matrix4f rotate45Mtx;
	rotate45Mtx << 0.707, -0.707, 0, 0,   0.707, 0.707, 0, 0,   0, 0, 1, 0,  0, 0, 0, 1; //Rotate 45 deg
	TriMesh monitor = TriMesh("C:/Users/austi/Documents/Raytracer/A4/monitor.smf", rotate45Mtx, 0);
	
	//TriMesh monitor = TriMesh("C:/Users/austi/Documents/School/Spring 2022/Advanced_Computer_Graphics/A2/Models/monitor.smf");
	//ShadeInfo si2 = ShadeInfo(Vec3f(0.2, 0.1, 0.01), Vec3f(0.8, 0.4, 0.01), Vec3f(1, 1, 1), 5.0); //Orange
	ShadeInfo si2 = ShadeInfo(Vec3f(0.1, 0.1, 0.1), Vec3f(0.4, 0.4, 0.4), Vec3f(1, 1, 1), 10);
	worldObjects.push_back(&monitor);
	objectMaterials.push_back(&si2);
	//lets get rid of the bottom 2 lines once we bring testintersect over to main
	allTriMeshNormals.push_back(monitor.getNormals());
	
	//TODO: Low priority: right now if I use the same Shading Info si object for two entities, one of the entities dissapears. Fix this

	Matrix4f cubeToBridge;
	cubeToBridge << 1, 0, 0, 0,   0, 3, 0, 0,   0, 0, 0.1, 0,    0, 0, 0, 1; //Scale cube to make it a bridge
	Matrix4f rotateMtx;
	rotateMtx << -0.707, -0.707, 0, 0,   0.707, -0.707, 0, 0,   0, 0, 1, 0,   0, 0, 0, 1; //Rotate 135 deg along z axis
	Matrix4f rotateMtx2;
	rotateMtx2 << 1, 0, 0, 0,    0, 0, -1, 0,    0, 1, 0, 0,     0, 0, 0, 1; //Rotate 90 deg along x axis
	//TODO: Low priority - when I lower the z-scaling, the model seems to translate up in the z-axis. 
	Matrix4f translateZDownYRight;
	translateZDownYRight << 1, 0, 0, 0,     0, 1, 0, 1.5,     0, 0, 1, -1,      0, 0, 0, 1; 
	Matrix4f product = rotateMtx * translateZDownYRight * rotateMtx2;
	TriMesh bunny = TriMesh("C:/Users/austi/Documents/Raytracer/A4/bunny200.smf", product, 1);
	//ShadeInfo si3 = ShadeInfo(Vec3f(.09, 0.25, 1), Vec3f(0.6, 1, 1), Vec3f(1, 1, 1), 10.0);
	worldObjects.push_back(&bunny);
	objectMaterials.push_back(&si2);
	//lets get rid of the bottom 2 lines once we bring testintersect over to main
	allTriMeshNormals.push_back(bunny.getNormals());


	Matrix4f translateZDownYLeft;
	translateZDownYLeft << 1, 0, 0, 0,    0, 1, 0, 1,    0, 0, 0.5, -1,    0, 0, 0, 1; 
	 product = translateZDownYLeft;
	TriMesh cylinder = TriMesh("C:/Users/austi/Documents/Raytracer/A4/cylinder.smf", translateZDownYLeft, 2);
	worldObjects.push_back(&cylinder);
	objectMaterials.push_back(&si2);
	//lets get rid of the bottom 2 lines once we bring testintersect over to main
	allTriMeshNormals.push_back(cylinder.getNormals());


	vector<Tris> allTris;
	vector<Vec3f> allNormals;
	//Make a vector with all triangles in the scene for BVH
	for (int i = 0; i < worldObjects.size(); i++)
	{
		if (worldObjects[i]->getType() == TRIMESH)
		{
			vector<Tris> objectTris = worldObjects[i]->getTris();
			allTris = mergeVectors(objectTris, allTris);
		}
	}

	//Create a Bounding Volume Heirarchy structure to optimize ray interesection calculations
	BVH* pBVH = Make_BVH(allTris, -1); // start with full list non-partitioned

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
	ShadeInfo *temp_si;
	ShadeInfo *si = new ShadeInfo;

	auto start = high_resolution_clock::now();
	for (int i = 0; i < xRes; i++)
	{
		cout << i;
		for (int j = 0;  j < yRes; j++)
		{
			vectorSum = imgPlaneW * ((float)i / (xRes - 1)) * (Vec3f)camCoordX - imgPlaneH * ((float)j / (yRes - 1)) * (Vec3f)camCoordY;
			Point3f ray_direction = (topLeftPxl + vectorSum) - cam_loc;
			ray_direction = normalize(ray_direction);
			
			currentRay.d = ray_direction;
			currentRay.o = cam_loc;
			best_t_min = -1;

			//The raytracing code:
			int temp = 0;
			for (int k = 0; k < worldObjects.size(); k++)
			{
				


				//Pull the material shader info for this object
				temp_si = objectMaterials[k];
				//the computed shader info varibles will be set in temp_si by testIntersect
				//vector<Tris> intersecting = intersect_BVH(pBVH, currentRay, xsect_pt, t_min);
				//if(!intersecting.empty())
 				if (worldObjects[k]->testIntersect(pBVH, currentRay, t_min, *temp_si))
				{
					//Sphere-Ray Intersection found
					if ((t_min < best_t_min) || (best_t_min == -1))
					{
						best_t_min = t_min;
						si = temp_si;
					}
				}

				else if(best_t_min == -1)
				{
					//No interesction from any of the world objects were found
					si = temp_si;
				}
				
			}

			//Perform shading calculation on closest point
			//(intersection with the lowest non - negative t)
			

			//Set pixel value at Image[i, j]	
			if (si->hit_an_object)
			{
				//There must be at least 1 light in the scene
				Vec3f color = Vec3f(0, 0, 0);

				for (int l = 0; l < worldLights.size(); l++)
				{
					Vec3f lightVec = p2v(worldLights[l]->origin - si->local_hit_point);
					Vec3f reflectedRay = (2 * std::max(si->normal.dot(lightVec),0.0f) * si->normal) - lightVec;
					Vec3f viewVec = p2v(cam_loc - si->local_hit_point);
					reflectedRay = normalize(reflectedRay);
					lightVec = normalize(lightVec);
					viewVec = normalize(viewVec);
					float attenuation = min(1 / (norm(Mat(si->local_hit_point) - Mat(worldLights[l]->origin)) * worldLights[l]->att), 1.0); //TODO: Make our own euclidean distance func?
					float V_R = std::max(reflectedRay.dot(viewVec), 0.0f);
					Vec3f specular = worldLights[l]->colorIntensity.mul(si->specularColor) * pow(V_R, si->shininess);
					float N_L = std::max(lightVec.dot(si->normal), 0.0f);
					Vec3f diffuse = worldLights[l]->colorIntensity.mul(si->diffuseColor) * N_L * attenuation;
					Vec3f ambient = worldLights[l]->colorIntensity.mul(si->ambientColor);

					color = color + (diffuse + specular) * 255;
					if (l == 0)
					{
						color = color + ambient * 255;
					}
					
				}
				//A1 - Any intersection will be white
				//image.at<cv::Vec3b>(i, j) = si.color;
				//A2+ - using phong model
				
				//Clamp colors to 255 (max power white)
				if (color(0) > 255) { color(0) = 255; }
				if (color(1) > 255) { color(1) = 255; }
				if (color(2) > 255) { color(2) = 255; }
				
				Vec3b charColor = Vec3b(color(2),color(1),color(0)); //TODO: use Eigen's vectors instead of opencv vectors. Also this is BGR?
				image.at<cv::Vec3b>(i, j) = charColor;
				
			}
			else
			{
				//si.color will be the background color if there no ray intersections were found
				image.at<cv::Vec3b>(i, j) = si->color;				
			}
			
		}
	}
	auto stop = high_resolution_clock::now();
	//screen
		//for each pixel
		//	if intersection - shade point

	auto duration = duration_cast<microseconds>(stop - start);
	cout << "\n" << duration.count();
	cv::imshow("generated", image);
	cv::imwrite("C:/Users/austi/Documents/Raytracer/output/scene1_image.png", image);

	//Perform supersampling algorithm
	 start = high_resolution_clock::now();
	
	cv::Mat3b newImage = cv::Mat3b(xRes / 2, yRes / 2, cv::Vec3d{ 0,0,0 });
	for (int i = 0; i < xRes / 2; i++)
	{
		for (int j = 0; j < yRes / 2; j++)
		{
			int offset_x = 2 * i;
			int offset_y = 2 * j;

			Vec3f pixel1_1 = Vec3f(image.at<cv::Vec3b>(offset_x, offset_y));
			Vec3f pixel1_2 = Vec3f(image.at<cv::Vec3b>(offset_x, offset_y + 1));
			Vec3f pixel2_1 = Vec3f(image.at<cv::Vec3b>(offset_x + 1, offset_y));
			Vec3f pixel2_2 = Vec3f(image.at<cv::Vec3b>(offset_x + 1, offset_y + 1));
			Vec3f average = (pixel1_1 + pixel1_2 + pixel2_1 + pixel2_2) / 4;
			newImage.at<cv::Vec3b>(i, j) = Vec3b(average);

		}
	}
	stop = high_resolution_clock::now();

	duration = duration_cast<microseconds>(stop - start);
	cout << "\n" << duration.count();
	cv::imshow("generated_supersampled", newImage);
	cv::imwrite("C:/Users/austi/Documents/Raytracer/output/scene1_supersampled.png", newImage);

	cv::waitKey(0);

	return 0;
}
