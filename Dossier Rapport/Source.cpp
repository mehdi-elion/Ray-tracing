#define _CRT_SECURE_NO_WARNINGS 1

#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION

#include "stb_image_write.h"
#include "stb_image.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <random>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <list>
#include <string>


std::default_random_engine random;
std::uniform_real_distribution<double> u(0, 1);



#define M_PI 3.141592653589793238


// Cours de Nicolas Bonneel


class Vector {
public:
	Vector(double x = 0, double y = 0, double z = 0) :x(x), y(y), z(z) {};
	double norm2() { return x * x + y * y + z * z; }
	void normalize() {
		double n = sqrt(norm2());
		x /= n;
		y /= n;
		z /= n;
	}
	double x, y, z;
};


class Matrix {
public:
	Matrix(Vector L1, Vector L2, Vector L3) : L1(L1), L2(L2), L3(L3) {};
	
	void transpose() {
		double t12 = L1.y;
		L1.y = L2.x;
		L2.x = t12;

		double t13 = L1.z;
		L1.z = L3.x;
		L3.x = t13;

		double t23 = L2.z;
		L2.z = L3.y;
		L3.y = t23;
	}

	Vector L1, L2, L3;
};



Matrix rotation(Vector u, double alpha) {
	double e0 = cos(alpha / 2.0);
	double e1 = u.x * sin(alpha / 2.0);
	double e2 = u.y * sin(alpha / 2.0);
	double e3 = u.z * sin(alpha / 2.0);
	Vector L1(e0*e0 + e1*e1 - e2*e2 - e3*e3,  2*(e1*e2 - e0*e3),  2*(e0*e2 + e1*e3)); 
	Vector L2(2*(e0*e3 + e1*e2),  e0*e0 - e1*e1 + e2*e2 - e3*e3,  2*(e2*e3 - e0*e1)); 
	Vector L3(2*(e1*e3 - e0*e2), 2*(e0*e1 + e2*e3), e0*e0 - e1*e1 - e2*e2 + e3*e3);
	Matrix M(L1, L2, L3);
	M.transpose();
	return M;
}


Vector operator+(const Vector& a, const Vector &b) {
	return Vector(a.x + b.x, a.y + b.y, a.z + b.z);
}


double dot(const Vector& a, const Vector& b) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector cross(const Vector& a, const Vector& b) {
	return Vector(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x) ;
}

Vector operator/(const Vector& a, double b) {
	return Vector(a.x / b, a.y / b, a.z / b);
}


Vector operator-(const Vector &a, const Vector &b) {
	return Vector(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector operator*(const Vector &a, double b) {
	return Vector(b*a.x, b*a.y, b*a.z);
}


Vector operator*(const Matrix& M, const Vector &b) {
		return Vector(dot(M.L1,b), dot(M.L2, b), dot(M.L3, b));
}



class Ray {
public:
	Ray(const Vector &C, const Vector &u) : C(C), u(u) {};
	Vector C;
	Vector u;
};




class Object {
public:
	Object() {};
	Object(const Vector A, const bool mirror, const double n) : A(A), mirror(mirror), n(n) {};
	virtual bool intersect(const Ray& r, Vector&P, Vector&N, Vector& Color) = 0;
	
	Vector A;
	bool mirror;
	double n;

};






class Triangle : public Object {

public:
	Triangle(const Vector PA, const Vector PB, const Vector PC, const Vector A, const bool mirror, const double n) : PA(PA), PB(PB), PC(PC) {
		this->mirror = mirror;
		this->A = A;
		this->n = n;
	};

	Vector normal() {
		Vector N = cross(this->PB - this->PA, this->PC - this->PA);
		N.normalize();
		return N;
	};

	Vector normal_ext() {
		Vector N = cross(this->PB - this->PA, this->PC - this->PA);
		N.normalize();
		return N;
	};

	Vector normal_int() {
		Vector N = cross(this->PB - this->PA, this->PC - this->PA);
		N.normalize();
		return N*(-1.0);
	};


		/*
		Vector u = b - a;
		Vector v = c - a;
		Vector w = P - a;

		double m11 = u.norm2();
		double m12 = dot(u, v);
		double m22 = v.norm2();
		double detm = m11 * m22 - m12 * m12;

		double b11 = dot(w, u);
		double b21 = dot(w, v);
		double detb = b11 * m22 - b21 * m12;
		double beta = detb / detm;

		double g12 = b11;
		double g22 = b21;
		double detg = m11 * g22 - m12 * g12;
		double gamma = detg / detm;

		double alpha = 1 - beta - gamma;

		if (alpha < 0 || alpha>1) { return false; }
		else if (beta < 0 || beta>1) { return false; }
		else if (gamma < 0 || gamma>1) { return false; }

		else { return true; }
		*/


	/**/
	virtual bool intersect(const Ray& r, Vector& P, Vector& N, Vector& Color) {

		// Couleur du triangle
		Color = this->A;

		// Sommets du triangle
		Vector a = this->PA;
		Vector b = this->PB;
		Vector c = this->PC;

		// Calcul de la normal au triangle
		N = this->normal() * (-1);

		// Calcul du point d'interection avec le plan
		double t = dot(a - r.C, N);
		double den = dot(r.u, N);

		if (den < 1e-10) { return false; }

		t = t / den;

		if (t < 0) { return false; }

		P = r.C +  r.u * t;

		
		// normale sortante
		if (den > 0) { N = N * (-1.0); }
		
			
		// calcul du déterminant dénominateur
		double det = (b-a).norm2() * (c-a).norm2() - std::pow(dot(b-a, c-a), 2);

		// Calcul des coeffs barycentriques avec Cramer
		double beta = dot(P - a, b - a) * (c - a).norm2() - dot(c - a, b - a) * dot(P - a, c - a);
		beta = beta / det;

		double gamma = dot((P - a), (c - a)) * (b - a).norm2() - dot(c - a, b - a) * dot(P - a, b - a);
		gamma = gamma / det;

		double alpha = 1 - beta - gamma;

		if (alpha < 0 || alpha>1) return false;
		if (beta < 0 || beta>1) return false;
		if (gamma < 0 || gamma>1) return false;

		return true;
		
		
	};
	



	/*
	virtual bool intersect_ext(const Ray& r, Vector& P, Vector& N) {

		// Sommets du triangle
		Vector a = this->PA;
		Vector b = this->PB;
		Vector c = this->PC;

		// Calcul de la normal au triangle
		N = this->normal_ext();

		// Calcul du point d'interection avec le plan
		double t = dot(a - r.C, N);
		double den = dot(r.u, N);

		if (den < 1e-9) { return false; }

		t = t / den;

		if (t < 0) { return false; }

		P = r.C +  r.u * t;


		// normale sortante
		//if (den > 0) { N = N * (-1.0); }


		// calcul du déterminant dénominateur
		double det = (b-a).norm2() * (c-a).norm2() - std::pow(dot(b-a, c-a), 2);

		// Calcul des coeffs barycentriques avec Cramer
		double beta = dot(P - a, b - a) * (c - a).norm2() - dot(c - a, b - a) * dot(P - a, c - a);
		beta = beta / det;

		double gamma = dot((P - a), (c - a)) * (b - a).norm2() - dot(c - a, b - a) * dot(P - a, b - a);
		gamma = gamma / det;

		double alpha = 1 - beta - gamma;

		if (alpha < 0 || alpha>1) return false;
		if (beta < 0 || beta>1) return false;
		if (gamma < 0 || gamma>1) return false;

		return true;


	};


	virtual bool intersect_int(const Ray& r, Vector& P, Vector& N) {

		// Sommets du triangle
		Vector a = this->PA;
		Vector b = this->PB;
		Vector c = this->PC;

		// Calcul de la normal au triangle
		N = this->normal_int();

		// Calcul du point d'interection avec le plan
		double t = dot(a - r.C, N);
		double den = dot(r.u, N);

		if (den < 1e-9) { return false; }

		t = t / den;

		if (t < 0) { return false; }

		P = r.C + r.u * t;


		// normale sortante
		if (den > 0) { N = N * (-1.0); }


		// calcul du déterminant dénominateur
		double det = (b - a).norm2() * (c - a).norm2() - std::pow(dot(b - a, c - a), 2);

		// Calcul des coeffs barycentriques avec Cramer
		double beta = dot(P - a, b - a) * (c - a).norm2() - dot(c - a, b - a) * dot(P - a, c - a);
		beta = beta / det;

		double gamma = dot((P - a), (c - a)) * (b - a).norm2() - dot(c - a, b - a) * dot(P - a, b - a);
		gamma = gamma / det;

		double alpha = 1 - beta - gamma;

		if (alpha < 0 || alpha>1) return false;
		if (beta < 0 || beta>1) return false;
		if (gamma < 0 || gamma>1) return false;

		return true;


	};

	virtual bool intersect(const Ray& r, Vector& P, Vector& N) {

		return intersect_ext(r, P, N) || intersect_int(r, P, N);

	};
	*/


	Vector PA;
	Vector PB;
	Vector PC;

};





class TriangleFace : public Object {

public:
	TriangleFace(const Vector PA, const Vector PB, const Vector PC, const Vector NA, const Vector NB, const Vector NC, const Vector uvA, const Vector uvB, const Vector uvC, std::vector<unsigned char*> textures, int textureID, int texture_w, int texture_h, const Vector A, const bool mirror, const double n) : PA(PA), PB(PB), PC(PC), NA(NA), NB(NB), NC(NC), uvA(uvA), uvB(uvB), uvC(uvC), textureID(textureID), textures(textures), texture_w(texture_w), texture_h(texture_h) {
		this->mirror = mirror;
		this->A = A;
		this->n = n;
	};

	Vector normal() {
		Vector N = cross(this->PB - this->PA, this->PC - this->PA);
		N.normalize();
		return N;
	};

	Vector normal_ext() {
		Vector N = cross(this->PB - this->PA, this->PC - this->PA);
		N.normalize();
		return N;
	};

	Vector normal_int() {
		Vector N = cross(this->PB - this->PA, this->PC - this->PA);
		N.normalize();
		return N * (-1.0);
	};


	


	virtual bool intersect(const Ray& r, Vector& P, Vector& N, Vector& Color) {

		// Couleur du triangle
		Color = this->A;

		// Sommets du triangle
		Vector a = this->PA;
		Vector b = this->PB;
		Vector c = this->PC;

		// Calcul de la normal au triangle
		N = this->normal();

		// Calcul du point d'interection avec le plan
		double t = dot(a - r.C, N);
		double den = dot(r.u, N);
		
		
		if (den < 1e-19) { return false; }

		t = t / den;

		if (t < 0) { return false; }

		P = r.C + r.u * t;


		// normale sortante
		//if (den > 0) { N = N * (-1.0); }


		/**/
		// calcul du déterminant dénominateur
		double det = (b - a).norm2() * (c - a).norm2() - std::pow(dot(b - a, c - a), 2);
		if (fabs(det) == 0.0) return false;

		// Calcul des coeffs barycentriques avec Cramer
		double beta = dot(P - a, b - a) * (c - a).norm2() - dot(c - a, b - a) * dot(P - a, c - a);
		beta = beta / det;

		double gamma = dot((P - a), (c - a)) * (b - a).norm2() - dot(c - a, b - a) * dot(P - a, b - a);
		gamma = gamma / det;

		double alpha = 1 - beta - gamma;

		if (alpha < 0 || alpha>1) return false;
		if (beta < 0 || beta>1) return false;
		if (gamma < 0 || gamma>1) return false;

		/* Interpolation de Phong*/
		N = this->NA * alpha + this->NB * beta + this->NC * gamma;
		N.normalize();
		

		// Texture 

		/**/
		int x = (( alpha*fabs(fmod(uvA.x,1)) + beta*fabs(fmod(uvB.x,1)) + gamma*fabs(fmod(uvC.x,1)) ) * (texture_w - 1));
		int y = (( alpha*fabs(fmod(1-uvA.y,1)) + beta*fabs(fmod(1-uvB.y,1)) + gamma*fabs(fmod(1-uvC.y,1))) * (texture_h - 1));

		double cr = textures[textureID][(y*texture_w + x)*3 + 0] / 255.0 ;
		double cg = textures[textureID][(y*texture_w + x)*3 + 1] / 255.0 ;
		double cb = textures[textureID][(y*texture_w + x)*3 + 2] / 255.0 ;
		
		//Vector NewColor(cr, cg, cb);
		Vector NewColor(cr*0.4, cg*0.4, cb*0.4);		// pour attébuer la brillance dde la fille
		

		//std::cout << "\n cr = " << cr << " cb = " << cb << " cg = " << cg << "\n" << std::endl;

		Color = NewColor ;
		//Color = this->A;

		return true;
	};	
	


	Vector PA;
	Vector PB;
	Vector PC;
	Vector NA;
	Vector NB;
	Vector NC;
	Vector uvA;
	Vector uvB;
	Vector uvC;

	std::vector<unsigned char*>  textures;
	int textureID;
	int texture_w;
	int texture_h;

};









class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class BBox {
public:
	BBox() {};
	BBox(const Vector& bmin, const Vector& bmax) : bmin(bmin), bmax(bmax) {};

	bool intersection(const Ray r) {

		double t1x = (bmin.x - r.C.x) / r.u.x;
		double t2x = (bmax.x - r.C.x) / r.u.x;
		double tmin_x = std::min(t1x, t2x);
		double tmax_x = std::max(t1x, t2x);

		double t1y = (bmin.y - r.C.y) / r.u.y;
		double t2y = (bmax.y - r.C.y) / r.u.y;
		double tmin_y = std::min(t1y, t2y);
		double tmax_y = std::max(t1y, t2y);

		double t1z = (bmin.z - r.C.z) / r.u.z;
		double t2z = (bmax.z - r.C.z) / r.u.z;
		double tmin_z = std::min(t1z, t2z);
		double tmax_z = std::max(t1z, t2z);

		return std::min(tmax_x, std::min(tmax_y, tmax_z)) - std::max(tmin_x, std::max(tmin_y, tmin_z)) > 0 ;
			   
	}

	Vector bmin;
	Vector bmax;
};




class BVH {
public:
	int i0, i1;
	BBox bbox;
	BVH *fg, *fd;

};





class Geometry : public Object {

public:
	~Geometry() {}
	Geometry(double scaling, const Vector& translate, const Matrix rotation, const Vector A, const bool mirror, const double n) : obj(obj), mtl_file(mtl_file), scaling(scaling), translate(translate), rotation(rotation) {
		this->mirror = mirror;
		this->A = A;
		this->n = n;

	};


	void readOBJ(const char* obj, const char* mtl) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec.x, &vec.y, &vec.z, &col.x, &col.y, &col.z) == 6) {
					col.x = std::min(1., std::max(0., col.x));
					col.y = std::min(1., std::max(0., col.y));
					col.z = std::min(1., std::max(0., col.z));

					//vertices.push_back(vec * scaling + translate);
					vertices.push_back(rotation * vec * scaling + translate);
					vertexcolors.push_back(col);
					///std::cout << "\n col.x = " << col.x << " col.y = " << col.y << " col.z = " << col.z << " \n";

				}
				else {
					sscanf(line, "v %lf %lf %lf\n", &vec.x, &vec.y, &vec.z);
					//vertices.push_back(vec * scaling + translate);
					vertices.push_back(rotation * vec * scaling + translate);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec.x, &vec.y, &vec.z);
				//normals.push_back(vec);
				normals.push_back(rotation * vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec.x, &vec.y);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				}
				else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					}
					else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						}
						else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}


				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					}
					else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						}
						else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							}
							else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								}
								else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);


		f = fopen(mtl, "r");
		//fopen_s(&f, (std::string(mtl)).c_str(), "r");
		while (!feof(f)) {
			char line[255];
			fgets(line, 255, f);
			if (line[0] == 'm' && line[4] == 'K' && line[5] == 'd') {
				char texture_file[255];
					sscanf(line, "map_Kd %100s\n", texture_file);
					add_texture(std::string(texture_file).c_str());
					///std::cout << texture_file << "\n";

			}
		}
		fclose(f);




	}


	void makeBBox() {

		bb.bmax = this->vertices[0];
		bb.bmin = this->vertices[0];

		for (int i = 1; i < this->vertices.size(); i++) {

			bb.bmin.x = std::min(bb.bmin.x, this->vertices[i].x);
			bb.bmax.x = std::max(bb.bmax.x, this->vertices[i].x);

			bb.bmin.y = std::min(bb.bmin.y, this->vertices[i].y);
			bb.bmax.y = std::max(bb.bmax.y, this->vertices[i].y);

			bb.bmin.z = std::min(bb.bmin.z, this->vertices[i].z);
			bb.bmax.z = std::max(bb.bmax.z, this->vertices[i].z);

		}
	}

	BBox build_BBox(int i0, int i1) {

		BBox result;
		TriangleIndices tri_ind_i = this->indices[i0];
		result.bmax = this->vertices[tri_ind_i.vtxi];
		result.bmin = this->vertices[tri_ind_i.vtxi];

		for (int i = i0; i < i1; i++) {

			TriangleIndices tri_ind_i = this->indices[i];

			result.bmin.x = std::min(result.bmin.x, this->vertices[tri_ind_i.vtxi].x);
			result.bmax.x = std::max(result.bmax.x, this->vertices[tri_ind_i.vtxi].x);

			result.bmin.y = std::min(result.bmin.y, this->vertices[tri_ind_i.vtxi].y);
			result.bmax.y = std::max(result.bmax.y, this->vertices[tri_ind_i.vtxi].y);

			result.bmin.z = std::min(result.bmin.z, this->vertices[tri_ind_i.vtxi].z);
			result.bmax.z = std::max(result.bmax.z, this->vertices[tri_ind_i.vtxi].z);

			result.bmin.x = std::min(result.bmin.x, this->vertices[tri_ind_i.vtxj].x);
			result.bmax.x = std::max(result.bmax.x, this->vertices[tri_ind_i.vtxj].x);

			result.bmin.y = std::min(result.bmin.y, this->vertices[tri_ind_i.vtxj].y);
			result.bmax.y = std::max(result.bmax.y, this->vertices[tri_ind_i.vtxj].y);

			result.bmin.z = std::min(result.bmin.z, this->vertices[tri_ind_i.vtxj].z);
			result.bmax.z = std::max(result.bmax.z, this->vertices[tri_ind_i.vtxj].z);

			result.bmin.x = std::min(result.bmin.x, this->vertices[tri_ind_i.vtxk].x);
			result.bmax.x = std::max(result.bmax.x, this->vertices[tri_ind_i.vtxk].x);

			result.bmin.y = std::min(result.bmin.y, this->vertices[tri_ind_i.vtxk].y);
			result.bmax.y = std::max(result.bmax.y, this->vertices[tri_ind_i.vtxk].y);

			result.bmin.z = std::min(result.bmin.z, this->vertices[tri_ind_i.vtxk].z);
			result.bmax.z = std::max(result.bmax.z, this->vertices[tri_ind_i.vtxk].z);


		}

		return result;
	}


	void makeBVH(BVH* node, int i0, int i1) {


		node->bbox = build_BBox(i0, i1);
		node->i0 = i0;
		node->i1 = i1;
		node->fg = NULL;
		node->fd = NULL;
		
		Vector diag = node->bbox.bmax - node->bbox.bmin;
		int split_dim;
		double split_val;

		if (diag.x > diag.y && diag.x > diag.z) {
			split_dim = 0;
			split_val = node->bbox.bmin.x + diag.x * 0.5;
		}
		else if (diag.y > diag.x && diag.y > diag.z) {
			split_dim = 1;
			split_val = node->bbox.bmin.y + diag.y * 0.5;
		}
		else {
			split_dim = 2;
			split_val = node->bbox.bmin.z + diag.z * 0.5;
		}

		int pivot = i0-1;
		for (int i = i0; i < i1; i++) {
			
			// On récupère les indices de la face
			TriangleIndices tri_ind_i = this->indices[i];

			// On récupère les coordonnées des sommets de la face courante
			Vector PA_i = this->vertices[tri_ind_i.vtxi];
			Vector PB_i = this->vertices[tri_ind_i.vtxj];
			Vector PC_i = this->vertices[tri_ind_i.vtxk];

			// On calcule le centre de la face et sa coordonnée selon split_dim
			Vector center = (PA_i + PB_i + PC_i) * 1.0 / 3.0;
			double center_split_dim;
			if (split_dim == 0) center_split_dim = center.x;
			else if (split_dim == 1) center_split_dim = center.y;
			else if (split_dim == 2) center_split_dim = center.z;

			// On opère la permutation (quick sort)
			if (center_split_dim < split_val) {
				pivot++;
				std::swap(this->indices[i], this->indices[pivot]);
			}
			

		}

		if (pivot < i0 || pivot >= i1 - 1 || i1 == i0 + 1)  return;

		node->fg = new BVH();
		makeBVH(node->fg, i0, pivot+1);

		node->fd = new BVH();
		makeBVH(node->fd, pivot+1, i1);

	}

	/**/
	void add_texture(const char* filename) {
		int w, h, c;
		unsigned char* data = stbi_load(filename, &w, &h, &c, 3);
		this->textures.push_back( data );
		this->texture_widths.push_back(w);
		this->texture_heigths.push_back(h);
		this->texture_colors.push_back(c);
	}
	

	virtual bool intersect(const Ray& r, Vector& P, Vector& N, Vector& Color) {
		
		bool intersect_geometry = false;
		bool intersect_face_i = false;
		double d2min = 1e99;

		if (!this->bvh.bbox.intersection(r)) return false;

		std::list<const BVH*> l;
		l.push_front(&this->bvh);

		while (!l.empty()) {

			const BVH* current = l.front();
			l.pop_front();

			if (current->fg && current->fg->bbox.intersection(r)) {
				l.push_back(current->fg);
			}
			if (current->fd && current->fd->bbox.intersection(r)) {
				l.push_back(current->fd);
			}

			if (!current->fg) {

				for (int i_face = current->i0; i_face <= current->i1; i_face++) {

					// On initialise les arguments de la routine d'intersection
					Vector Pi;
					Vector Ni;
					Vector Color_i;
					

					// On récupère les indices de la face
					TriangleIndices tri_ind_i = this->indices[i_face];
					
					// On récupère les coordonnées des sommets de la face courante
					Vector PA_i = this->vertices[tri_ind_i.vtxi];
					Vector PB_i = this->vertices[tri_ind_i.vtxj];
					Vector PC_i = this->vertices[tri_ind_i.vtxk];

					// On récupère les coordonnées des sommets de la face courante
					Vector NA_i = this->normals[tri_ind_i.ni];
					Vector NB_i = this->normals[tri_ind_i.nj];
					Vector NC_i = this->normals[tri_ind_i.nk];

					// Textures & Couleurs
					int textureID = tri_ind_i.group;
					int texture_w = this->texture_widths[textureID];
					int texture_h = this->texture_heigths[textureID];
					Vector uvA_i = this->uvs[tri_ind_i.uvi];
					Vector uvB_i = this->uvs[tri_ind_i.uvj];
					Vector uvC_i = this->uvs[tri_ind_i.uvk];


					// On instancie la face courante comme une instance de la classe Triangle
					TriangleFace Face_i(PA_i, PB_i, PC_i, NA_i, NB_i, NC_i, uvA_i, uvB_i, uvC_i, this->textures, textureID, texture_w, texture_h, this->A, mirror, n);

					// Test d'intersection avec la face courante
					if (Face_i.intersect(r, Pi, Ni, Color_i)) {
						if ((r.C - Pi).norm2() < d2min) {
							intersect_face_i = true;
							intersect_geometry = true;
							P = Pi;
							N = Ni;
							Color = Color_i;
							d2min = (r.C - P).norm2();
						}

					}
				}
			}

			

		}

		return intersect_geometry;

	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;

	std::vector<unsigned char*> textures;
	std::vector<int> texture_widths;
	std::vector<int> texture_heigths;
	std::vector<int> texture_colors;

	BBox bb;
	BVH bvh;

	const char* obj;
	const char* mtl_file;
	double scaling;
	const Vector& translate;
	const Matrix rotation;
};






class Sphere : public Object {

public:
	Sphere() {};
	Sphere(const Vector O, double R, Vector A, bool mirror, double n) : O(O), R(R) {
		this->mirror = mirror;
		this->A = A;
		this->n = n;
	};

	virtual bool intersect(const Ray& r, Vector&P, Vector&N, Vector& Color) {
		Color = this->A;
		// résolution equation du second degré
		// a*x^2 + b*x + c = 0
		double a = 1;
		double b = 2*dot(r.u, r.C - O);
		double c = (r.C - O).norm2() - R * R;
		double delta = b * b - 4 * a*c;

		if (delta >= 0) {
			double t1 = (-b - sqrt(delta)) / (2 * a);
			double t2 = (-b + sqrt(delta)) / (2 * a);

			if (t1 > 0) {
				P = r.C + r.u * t1;
			}
			else {
				if (t2 > 0) {
					P = r.C + r.u * t2;
				}
				else {
					return false;
				}
			}

			// Normale à la sphère aupoint d'intersection
			N = P - O;
			N.normalize() ;

			return true;
		}
		else
			return false;


	}

	Vector O;
	double R;
};







class Scene {
public:
	Scene() {};


	void addObject(Object* s) {
		objects.push_back(s);
	}


	Vector random_cos(const Vector& N) {
		double r1 = u(random);
		double r2 = u(random);
		double s = sqrt(1 - r2);
		Vector v(cos(2 * M_PI*r1)*s, sin(2 * M_PI*r1)*s, sqrt(r2));

		Vector T1;
		Vector T2;

		if (std::abs(N.x) <= std::abs(N.y) && std::abs(N.x) <= std::abs(N.z)) {
			T1 = Vector(0, -N.z, N.y);
		}
		else {
			if (std::abs(N.y) <= std::abs(N.x) && std::abs(N.y) <= std::abs(N.z)) {
				T1 = Vector(-N.z, 0, N.x);
			}
			else {
				T1 = Vector(-N.y, N.x, 0);
			}
		}
		T1.normalize();
		T2 = cross(N, T1);
		return  T1 * v.x + T2 * v.y + N * v.z;
	}











	Vector getColor2(const Vector &L, const double &Intensity, const Ray& I, const Vector& P, const Vector &N, const int &indSm, Vector& albedo,  int &nbRebounds) {

		// On se situe au point d'intersection P (normale N) de la sphere miroir, transparente ou diffuse spheres[indSm] avec le rayon incident I
		

		// Initialisation du vecteur de couleur (résultat) : noir si le rayon réfracté ou réfléchi n'intersecte pas la scene
		Vector Color(0, 0, 0);

		/**/
		if (indSm == 0) {
			Vector u = I.C - P;
			double d2 = u.norm2();
			//direct = albedo / (4 * M_PI * d_light2) * Intensity * std::max( 0.0, dot(N, Wi) * dot(dir_rand, Wi*(-1)) / dot(axeOP, dir_rand) ) ;
			//Color = albedo * Intensity / (0.5*1e5) ;
			Color = albedo / (4 * M_PI * d2) * Intensity ;
			return Color;
		}
		


		// Si la surface est spéculaire et numéro rebonds > 0
		else if (this->objects[indSm]->mirror  &&  nbRebounds > 0) {

			// Réflexion du rayon
			Vector RC, Ru;
			RC = P + N * 0.0000001;
			Ru = I.u - N * 2 * dot(I.u, N);
			Ray R(RC, Ru);

			// Intersection de ce nouveau rayon avec la scène
			Vector P_inter, N_inter;
			Vector albedo_inter(0.0, 0.0, 0.0);
			int indS_inter;
			bool intersect = this->intersect(R, P_inter, N_inter, indS_inter, albedo_inter);

			// Si intersection du rayon réfléchi avec la scène
			if (intersect) {
				Object* S_inter = this->objects[indS_inter];
				nbRebounds = nbRebounds - 1;
				Color = this->getColor2(L, Intensity, R, P_inter, N_inter, indS_inter, albedo_inter, nbRebounds);
			}	
		}


		// Si la surface est transparente 
		else if (this->objects[indSm]->n > 1.0  &&  nbRebounds > 0) {

			// Réfraction du rayon
			Vector RC, Ru;
			double ns = this->objects[indSm]->n ;

			/// Si le rayon vient de l'extérieur de la shpère
			if (dot(N, I.u) < 0) {
				RC = P - N * 0.0000001;
				double delta = 1 - std::pow(1 / ns, 2) * (1 - std::pow(dot(I.u, N), 2));
				if (delta >= 0) {
					Ru = I.u / ns - N * ((1 / ns)*dot(I.u, N) + sqrt(delta));
				}
				else {
					std::cout << "sphere " << indSm << " delta negatif " << "\n";
					///std::this_thread::sleep_for(std::chrono::milliseconds(2000));
					Ru = I.u - N * 2 * dot(I.u, N);
				}
			}

			/// Si le rayon vient de traverser la sphère
			else {
				RC = P + N * 0.0000001;
				double delta = 1 - std::pow(ns, 2) * (1 - std::pow(dot(I.u, N*(-1)), 2));
				if (delta >= 0) {
					Ru = I.u * ns + N * ( ns * dot(I.u, N*(-1)) + sqrt(delta));
				}
				else {
					std::cout << "sphere " << indSm << " delta negatif " << "\n";
					///std::this_thread::sleep_for(std::chrono::milliseconds(2000));
					Ru = I.u + N * 2 * dot(I.u, N*(-1));
				}
			}

			/// Calcul du rayon
			Ray R(RC, Ru);

			// Intersection de ce nouveau rayon avec la scène
			Vector P_inter, N_inter;
			Vector albedo_inter(0.0, 0.0, 0.0);
			int indS_inter;
			bool intersect = this->intersect(R, P_inter, N_inter, indS_inter, albedo_inter);

			// Si intersection du rayon réfracté avec la scène
			if (intersect) {
				Object* S_inter = this->objects[indS_inter];
				nbRebounds = nbRebounds - 1;
				Color = this->getColor2(L, Intensity, R, P_inter, N_inter, indS_inter, albedo_inter, nbRebounds);
			}	
		}


		// Si la surface est diffuse et nbRebounds > 0
		else if (nbRebounds > 0) {

			// Décrément du nombre de rebonds
			nbRebounds = nbRebounds - 1;

			// Distance point d'intersection - lumière
			Vector PL = L - P;
			double d = sqrt(PL.norm2());
			Object* S = this->objects[indSm];

			// Calcul de la contribution directe (version lumière étendue)
			Vector direct(0.0, 0.0, 0.0);

			/// Lumière étendue (sphère n°0, émissive)
			Sphere* Lum = dynamic_cast<Sphere*>(this->objects[0]);
			Vector O_lum = Lum->O;
			double R_lum = Lum->R;

			/// Axe OP normalisé
			Vector axeOP = (P - O_lum);
			axeOP.normalize();

			/// Direction aléatoire 
			Vector dir_rand = this->random_cos(axeOP);

			/// Point Aléatoire sur la sphère lumineuse
			Vector pt_rand = O_lum + dir_rand * R_lum;

			/// Rayon (aléatoire) de la contribution directe
			Vector Wi = pt_rand - P;
			Wi.normalize();
			Ray Ray_lum(P + N * 0.00001, Wi);

			/// Distance à la lumière
			double d_light2 = (pt_rand - P).norm2();

			/// Vérification de l'absence d'occlusion entre P et la source de lumière
			Vector P_light, N_light, albedo_light;
			int indS_light;
			bool intersect_lum = this->intersect(Ray_lum, P_light, N_light, indS_light, albedo_light);
			double d_intersect2 = (P - P_light).norm2();

			/// S'il n'y a pas d'occlusion	
			/*
			if (indS_light == 0) {
				direct = albedo / (4 * M_PI * d_light2) * Intensity * std::max(0.0, dot(N, Wi) * dot(dir_rand, Wi*(-1)) / dot(axeOP, dir_rand));
			}
			*/
			if (!(intersect_lum && d_intersect2 < 0.99*d_light2)) {
				direct = albedo / (4 * M_PI * d_light2) * Intensity * std::max( 0.0, dot(N, Wi) * dot(dir_rand, Wi*(-1)) / dot(axeOP, dir_rand) ) ;
			}
			/*
			if (!(intersect_lum && indS_light != 0)) {
				direct = S->A / (4 * M_PI * d_light2) * Intensity * std::max(0.0, dot(N, Wi) * dot(dir_rand, Wi*(-1)) / dot(axeOP, dir_rand));
			}
			*/
			
			
			


			// Calcul de la contribution indirecte
			Vector indirect(0,0,0);

			/// Rayon (rebond) aléatoire
			Vector RC, Ru;
			RC = P + N * 0.0000001;
			Ru = this->random_cos(N);
			Ray R(RC, Ru);

			/// Intersection du rayon rebondi avec la scene
			Vector P_inter, N_inter;
			Vector albedo_inter(0.0, 0.0, 0.0);
			int indS_inter;
			bool intersect = this->intersect(R, P_inter, N_inter, indS_inter, albedo_inter);

			/// Si intersection du nouveau rayon avec la scène
			if (intersect) {
				indirect = this->getColor2(L, Intensity, R, P_inter, N_inter, indS_inter, albedo_inter, nbRebounds);
			}


			// Sommation des deux composantes (directe et indirecte)
			Color = (direct + indirect) * 0.5 / M_PI ;

			

		}	




		/*
		// Coefficient d'extinction
		Vector CP = I.C - P;
		double t = sqrt(CP.norm2());
		double beta = 0.04;
		double int_ext = beta * t;
		double T = exp(-int_ext);


		// Contribution du milieu participant
		Vector Lv(0.0, 0.0, 0.0);
		double phase_func = 0.3 / (4 * M_PI);

		Vector random_dir(0.0, 0.0, 0.0);

		double r1 = u(random);
		double r2 = u(random);
		Vector result;
		random_dir.x = 2.0 * cos(2.0*M_PI*r1)*sqrt(r2*(1 - r2));
		random_dir.y = 2.0 * sin(2.0*M_PI*r1)*sqrt(r2*(1 - r2));
		random_dir.z = 1 - 2.0 * r2;

		double proba_dir = 1.0 / (4.0 * M_PI);

		double random_t = u(random) * t;
		double proba_t = 1.0 / t;
		double int_ext_partielle = beta * random_t;

		Ray L_ray(I.C + random_dir * random_t, random_dir);
		Vector P_lum, N_lum, lum(0.0, 0.0, 0.0);
		int indS_lum;
		Vector albedo_lum(0.0, 0.0, 0.0);
		nbRebounds = nbRebounds - 1;
		bool intersect_lum = this->intersect(L_ray, P_lum, N_lum, indS_lum, albedo_lum);
		if (intersect_lum) {
			Vector lum = getColor2(L, Intensity, L_ray, P_lum, N_lum, indS_lum, albedo_lum, nbRebounds);
		}
		
		double ext = beta;
		Lv = lum * phase_func * ext * exp(-int_ext_partielle);

		return Color * T + Lv ;
		*/

		return Color;


	}









	bool intersect(const Ray& r, Vector &P, Vector &N, int &indS, Vector& Color) {

		// Intersection de la scene par le rayon
		bool inter_bol = false;
		bool inter_bol_i;
		
		// Initialisation de la disatance min
		double dmin = 1000000000000000000000000000000000000000.0;

		// Initialisation de Pi et Ni
		Vector Pi, Ni, Color_i;

		// Comparaison aux autres spheres (on ignore la sphere 0 qui est lumineuse)
		for (int i = 0; i < objects.size(); i++) {

			// Récupération de la sphere i
			Object* S_i = objects[i];

			// Initialisation du booléen
			bool inter_bol_i = S_i->intersect(r, Pi, Ni, Color_i);

			// Indice de la sphere la plus proche
			if (inter_bol_i == true) {
				inter_bol = true;
				if ((Pi - r.C).norm2() < dmin) {
					indS = i;
					P = Pi;
					N = Ni;
					Color = Color_i;
					dmin = (P - r.C).norm2();
				}
			}
		}


		return inter_bol;

	}

	std::vector <Object*> objects;
};










/* ----------------------- Image Poly Sphères avec albedo ------------------------- */

// Rotation ==> M * v = s * R * v + t
// ==> M^(-1) * v = 1/s * R.t * (v - t)

int main() {

	// Taille de l'image
	int W = 512;
	int H = 512;

	// Nombre de chemins aléatoires pour intégration
	int nbRays = 10;

	// Nombre de rebonds par chemin
	int nbRebounds = 4;

	// Caméra
	double fov = 65 * M_PI / 180;
	Vector C(0, 0, 55);
	//Vector C(-10.0, 2, 47);
	double focus = 55-20;
	Matrix RotCam = rotation(Vector(0.0, 1.0, 0.0), M_PI / 5.0);

	// Lumière
	///double I = 70000000000;
	///double I = 15000000000;
	double I = 25000000000;
	Vector L(12,12,45);
	//Vector L(-15, 20, 2);


	// Sphère 0 (lumineuse)
	Vector O_lum = L;
	double R_lum = 4;
	Vector A_lum(1.0, 1.0, 1.0);
	//Vector A_lum(0.0, 0.0, 1.0);
	Sphere S_lum(O_lum, R_lum, A_lum, false, 1);


	// Sphère miroir
	//Vector O_miroir(-12, 0, 0);
	Vector O_miroir(-12, 0, 8);
	double R_miroir = 10;
	Vector A_miroir(0.0, 0.25, 1.0);
	Sphere S_miroir(O_miroir, R_miroir, A_miroir, true, 1.0);

	// Sphère sol
	Vector O_sol(0, -1000, 0);
	double R_sol = 990;
	Vector A_sol(0, 0.2, 0.2);
	Sphere S_sol(O_sol, R_sol, A_sol, false, 1);

	// Sphère plafond
	Vector O_top(0, 1000, 0);
	double R_top = 940;
	Vector A_top(0.5, 0.0, 0.5);
	Sphere S_top(O_top, R_top, A_top, false, 1);

	// Sphère mur fond
	Vector O_murBack(0, 0, -1000);
	double R_murBack = 940;
	Vector A_murBack(0.0, 1.0, 0.0);
	Sphere S_murBack(O_murBack, R_murBack, A_murBack, false, 1);

	// Sphère mur front
	Vector O_murFront(0, 0, 1000);
	double R_murFront = 940;
	Vector A_murFront(0.5, 1.0, 0.5);
	Sphere S_murFront(O_murFront, R_murFront, A_murFront, false, 1);

	// Sphère mur droit
	Vector O_murD(1000, 0, 0);
	double R_murD = 940;
	Vector A_murD(0.5, 0.5, 0.5);
	Sphere S_murD(O_murD, R_murD, A_murD, false,  1);

	// Sphère mur gauche
	Vector O_murG(-1000, 0, 0);
	double R_murG = 940;
	Vector A_murG(0.5, 0.5, 0.0);
	Sphere S_murG(O_murG, R_murG, A_murG, false, 1);

	// Sphère transparente
	//Vector O_trans(12, 0, 0);
	//double R_trans = 10;
	Vector O_trans(12, 10, 20);
	double R_trans = 7;
	Vector A_trans(0.85, 0.25, 0.85);
	Sphere S_trans(O_trans, R_trans, A_trans, false, 1);
	//Sphere S_trans(O_trans, R_trans, A_trans, false, 1.4);

	// Sphère diffuse
	Vector O_diff(4, 2, 20);
	double R_diff = 5;
	Vector A_diff(0.15, 0.95, 0.95);
	Sphere S_diff(O_diff, R_diff, A_diff, false, 1);

	// Sphère diffuse 1
	Vector O_diff1(-7, 2, 30);
	double R_diff1 = 5;
	Vector A_diff1(0.95, 0.95, 0.15);
	Sphere S_diff1(O_diff1, R_diff1, A_diff1, false, 1);

	// Sphère diffuse 2
	Vector O_diff2(18, 2, 10);
	double R_diff2 = 5;
	Vector A_diff2(0.95, 0.15, 0.95);
	Sphere S_diff2(O_diff2, R_diff2, A_diff2, false, 1);

	// Triangle 
	/*
	Vector PA(-5.0, 0.0, 11.0);
	Vector PB(5.0, 0.0, 11.0);
	Vector PC(0.0, 5.0, 11.0);
	*/
	/*
	Vector PA(0.0, -7.0, 10.0);
	Vector PB(-6.0, -7.0, 15.0);
	Vector PC(6.0, -7.0, 15.0);
	*/
	/*
	Vector PA(0.0, 5.0, 12.0);
	Vector PB(-6.0, -7.0, 12.0);
	Vector PC(6.0, -7.0, 12.0);
	
	Vector A9(1.0, 0.0, 0.0);
	Triangle T9(PA, PB, PC, A9, false, 1.0);
	*/

	/* Beautiful Girl */
	const char* obj_file = "Beautiful Girl.obj";
	double scaling = 15.0;
	//const Vector translate(0.0, 20.0, -10.0);  /// y et z permutés
	const Vector translate(0.0, 26.0, -10.0);  /// y et z permutés
	//Matrix Rot = rotation(Vector(0.0, 0.0, 1.0), M_PI / 4.0);  /// y et z permutés
	Matrix Rot = rotation(Vector(0.0, 0.0, 1.0), 0.0);  /// y et z permutés
	bool mirror = false;
	double n = 1;
	const char* mtl_file = "Beautiful Girl.mtl";
	//Vector A10(0.20, 0.20, 0.20);
	Vector A10(0.50, 0.50, 0.50);

	Geometry Girl(scaling, translate, Rot, A10, mirror, n);
	Girl.readOBJ(obj_file, mtl_file);
	
	/// Pivotage de la figure Girl   (car 2,0,1 // dragon 0,1,2)
	for (int ind = 0; ind < Girl.vertices.size(); ind++) {
		/// Pivotage des coordonnées de vertex
		Vector v_old = Girl.vertices[ind];
		Vector v_new(v_old.x, v_old.z, v_old.y);
		Girl.vertices[ind] = v_new;
		/// Pivotage des coordonnées des normales
		Vector n_old = Girl.normals[ind];
		Vector n_new(n_old.x, n_old.z, n_old.y);
		Girl.normals[ind] = n_new;
	}
	/// Calcul de la BBox
	Girl.makeBBox();
	/// Calcul de la BVH
	Girl.makeBVH(&Girl.bvh, 0, Girl.indices.size());
	



	// Scène
	Scene Sc;
	Sc.addObject(& S_lum);			// 0
	Sc.addObject(& S_top);			// 1
	Sc.addObject(& S_sol);			// 2
	Sc.addObject(& S_murD);			// 3
	Sc.addObject(& S_murG);			// 4
	Sc.addObject(& S_murFront);		// 5
	Sc.addObject(& S_murBack);		// 6
	Sc.addObject(& S_trans);		// 7
	Sc.addObject(& S_miroir);		// 8	
	//Sc.addObject(& S_diff);		// 9
	//Sc.addObject(& S_diff1);		// 10
	//Sc.addObject(& S_diff2);		// 11
	Sc.addObject(& Girl);			// 12

	
	/* Farandole
	double N = 70.0;
	int N_iter = (int)N;
	double x0 = 0;
	double z0 = 15;
	double theta_min = 0.0;
	double theta_max = 9.0 * M_PI;
	double alt_max = 20.0;
	double alt_min = -8.0;
	double r = 10.0;
	double t, theta, alt;
	std::vector <Sphere> farandole;

	for (int k = 0; k < N_iter; k++) {
		t = 0.0 + k * (N - 0) / N; 
		theta = theta_min + k * (theta_max - theta_min) / N;
		alt = alt_min + k * (alt_max - alt_min) / N;
		
		if (k > 1) {
			//Sphere* Skk = dynamic_cast<Sphere*>(Sc.objects[6+k]);
			//std::cout << "objects[k].O.z = " << Skk->O.z << std::endl;
			//std::cout << "objects.size() = " << Sc.objects.size() << std::endl;
		}
		

		Vector O_k(x0 + r*exp(-t/(10*N)) * cos(theta),   alt,   z0 + r*exp(-t/(10*N)) * sin(theta));
		double R_k = 1;
		Vector A_k(1.0, 0.25, 0.25);
		Sphere S_k;
		farandole.push_back( Sphere(O_k, R_k, A_k, false, 1) );

		

	}
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	for (int k = 0; k < N_iter; k++) {
		Sc.addObject(& farandole[k] );
	}
	 */



	// Sphere Lumineuse
	Sphere* Lum = dynamic_cast<Sphere*>(Sc.objects[0]);

	// Pour Chaque pixel de l'image
	std::vector<unsigned char> image(W*H * 3, 0);

	// On lance le chrono
	int temps_sec, temps_min, temps_h;
	clock_t clock1, clock2;
	clock1 = clock();

	#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

			// Initialisation du vecteur de la couleur du pixel (i,j)
			Vector pixelColor(0.0, 0.0, 0.0);

			// Sommation MonteCarlo
			double countRays = 0.0;
			for (int ray = 0; ray < nbRays; ray++) {

				// Initialisation du compteur de rebonds
				int nbRebounds_ij = nbRebounds;

				// Incrément compteur
				countRays = countRays + 1.0;

				/* Rayon initial (version anti-aliasing) */
				double theta_rand = 2*M_PI*u(random);
				double R_rand = u(random);
				double u_rand = R_rand * cos(theta_rand);
				double v_rand = R_rand * sin(theta_rand);
				Vector V( j - W / 2 + u_rand + 0.5 ,   -i + H / 2 + v_rand + 0.5,   -W / (2 * tan(fov / 2)) );
				V.normalize();
				//V = RotCam * V;
				Ray Rayon_ij(C, V);
				 


				/* Rayon initial (version anti-aliasing + depth-of-field) 
				double theta_rand = 2 * M_PI*u(random);
				double R_rand = u(random);
				double u_rand = R_rand * cos(theta_rand);
				double v_rand = R_rand * sin(theta_rand);
				double x_open = (u(random) - 0.5) * 5.0 ;
				double y_open = (u(random) - 0.5) * 5.0 ;
				Vector direction(j - W / 2 + u_rand + 0.5, -i + H / 2 + v_rand + 0.5, -W / (2 * tan(fov / 2)));
				direction.normalize();

				Vector end = C + direction * focus;
				Vector start = C + Vector(x_open, y_open, 0.0);
				Vector new_direction = end - start;
				new_direction.normalize();
				Ray Rayon_ij(start, new_direction);
				*/
				


				/* Rayon initial (version sans anti-aliasing)
				Vector V(j - W / 2 - 0.5 , -i + H / 2 - 0.5 , -H / (2 * tan(fov / 2)));
				V.normalize();
				Ray Rayon_ij(C, V);
				*/


				// Intersection P et Normale à la sphère indS en P
				Vector P, N;
				Vector albedo(0.0, 0.0, 0.0);
				int indS;
				bool intersect_scene = Sc.intersect(Rayon_ij, P, N, indS, albedo);

				if (intersect_scene) {

					/* Sommation de la couleur du pixel (i,j) - version source étendue */
					pixelColor = pixelColor + Sc.getColor2(Lum->O, I, Rayon_ij, P, N, indS, albedo, nbRebounds_ij);
					
					/*
					if (indS == 0) {
						pixelColor + pixelColor + Sc.objects[indS]->A * I;
					}
					else {
						pixelColor = pixelColor + Sc.getColor2(Lum->O, I, Rayon_ij, P, N, indS, albedo, nbRebounds_ij);
					}
					*/

				}
			}

			// Moyennage de la somme des chemins
			pixelColor = pixelColor / countRays;
			/*std::cout << pixelColor.x << ' '<< pixelColor.y << ' ' << pixelColor.z << '\n';*/

			// Coloration de l'image avec correction gamma
			image[(i*W + j) * 3 + 0] = std::min(255.0, std::max(0.0, std::pow(pixelColor.x, 0.45)));
			image[(i*W + j) * 3 + 1] = std::min(255.0, std::max(0.0, std::pow(pixelColor.y, 0.45)));
			image[(i*W + j) * 3 + 2] = std::min(255.0, std::max(0.0, std::pow(pixelColor.z, 0.45)));


		}
	}

	// On arrête le chrono
	clock2 = clock();

	// On calcule le temps d'exécution
	temps_sec = (float)(clock2 - clock1) / CLOCKS_PER_SEC;
	temps_min = temps_sec / 60;
	temps_h = temps_min / 60;

	// Nom de l'image
	/// nombre de rayons
	char name[100] = "nbRays=" ;
	char nbRays_char[100]; 
	_itoa(nbRays, nbRays_char, 10);
	strcat(name, nbRays_char);
	/// nombre de rebonds
	char nbReb_char1[100] = " nbReb=";
	char nbReb_char2[100] ;
	_itoa(nbRebounds, nbReb_char2, 10);
	strcat(name, nbReb_char1);
	strcat(name, nbReb_char2);
	/// temps d'exécution 
	char t_char[100] = " time=";
	strcat(name, t_char);
	/// temps d'exécution - h
	char th1[100] ;
	_itoa(temps_h, th1, 10);
	char th2[100] = "h";
	strcat(name, th1);
	strcat(name, th2);
	/// temps d'exécution - min
	char tm1[100];
	_itoa(temps_min-60*temps_h, tm1, 10);
	char tm2[100] = "m";
	strcat(name, tm1);
	strcat(name, tm2);
	/// temps d'exécution - sec
	char ts1[100] ;
	_itoa(temps_sec-60*temps_min, ts1, 10);
	char ts2[100] = "s";
	strcat(name, ts1);
	strcat(name, ts2);
	// extention fichier
	char ext[100] = ".png";
	strcat(name, ext);

	//std::cout << "name = " << name << std::endl;
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	stbi_write_png(name, W, H, 3, &image[0], 0);
	stbi_write_png("image_recovery.png", W, H, 3, &image[0], 0);


	/* Test de la classe matrice
	Vector v_test(1.0, 2.0, 3.0);
	Vector L1(1.0, 0.0, 0.0);
	Vector L2(0.0, 2.0, 0.0);
	Vector L3(0.0, 0.0, 3.0);
	Matrix M(L1, L2, L3);
	Vector v_res;
	v_res = M * v_test;
	std::cout << "v_res_1 = " << v_res.x << std::endl;
	std::cout << "v_res_2 = " << v_res.y << std::endl;
	std::cout << "v_res_3 = " << v_res.z << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	*/


	return 0;
}


























