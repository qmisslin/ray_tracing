// scene.h: interface for the scene class.
//
// scene defines a set of 3D meshes
// Main operations are: raytracing and visibility computation
// 
// By JMD 10/8/06
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_SCENE_H__EC87CD56_6E08_4390_B876_CEC6D44EEA86__INCLUDED_)
#define AFX_SCENE_H__EC87CD56_6E08_4390_B876_CEC6D44EEA86__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "gmath/transform.h"
#include "gmath/perspective.h"

#include "mesh.h"
#include "bbh.h"
#include "phongbrdf.h"

const int MAX_MESHES=100;
const int MAX_OBJECTS=1000;
const int MAX_BRDF=100;
const int MAX_LIGHTS=100;
const int MAX_BOUNCE=10;
const unsigned int MAX_IN_BOX_FACE=6; // for triangles BVH
const unsigned int MAX_IN_BOX_OBJ=6;  // for objects BVH
const float SCENE_PRECISION=0.000001f;

const float LENSE = 0.84f;

const bool LISSAGE = false;
const bool WBBH = true;
const bool WTOPOLOGY = false;
const bool WSPECULAR = false;
const bool WSHADOW = false;

const double PI=3.141592653589793;



// CLASS scenepoint: DEFINES A POINT IN A SCENE
///////////////////////////////////////////////
template <class T> class scenepoint
{
protected:
		int iobj, iface;		// if point belongs to mesh
		intersection3<T> it;
		triangle3<T> tri;
public:
	scenepoint<T>() { iobj=-1; iface=-1; }
	scenepoint<T>(int io, int ifa, intersection3<T> &ii, triangle3<T> &tt) { iobj=io; iface=ifa; it=ii; tri=tt; }
	scenepoint<T>(const vec3<T> &x):it(x) { iobj=-1; iface=-1; }
	scenepoint<T>(int io, int ifa, triangle3<T> &tt):it(tt.inPoint()) { iobj=io; iface=ifa; tri=tt; }

	int getObj() const { return iobj; }
	int getFace() const { return iface; }
	T distance() { return it.distance(0); }
	vec3<T> getPoint() { return it.valueOf(); }
	triangle3<T> getTriangle() { return tri; }
};

// CLASS scene: DEFINES A set of 3D triangular Objects, Lighting and a Camera
///////////////////////////////////////////////
template <class T> class scene
{
public:
	enum	VISIBILITY_TYPE		{ CLOSEST, OCCLUDER, COMPLETE };

protected:

	// GEOMETRY DATA
	std::vector<unsigned int>			meshid;    // 3D objects list, object=index in mesh list
	std::vector<transform<T> >			pos;		// position of object
	std::vector<unsigned int>			brdfid;		// brdf of object, index in brdf list
	std::vector<bool>					liss;		// normal interpolation activated or not
	std::vector<bbh<T>	*>				bbhobj;		// bounding box hierarchy for triangles

	bbh<T>								*bbhscene;				// bounding box hierarchy of the objects of the entire scene

	std::vector<mesh<T>	*>				mm;		// list of meshes
	std::vector<brdf *>					refl;		// list of brdfs

	// CAMERA DATA
	int						W,H;					// image resolution
	vec3<T>					eyepos, lookpos;		// for perspective

	// LIGHTING DATA
	luminance				back;		// background color
	std::vector<vec4<T>	>	lights;		// point light sources or directional light sources (sun)
	std::vector<luminance>	lightit;	// energy and color of lights


public:
	// default constructor
	scene<T>(int resX, int resY): back(1.0f) 
	{ 
		W=resX; H=resY; 
		lookpos=vec3<T>(0.0);
		eyepos=vec3<T>(-20.0, -20.0, 20.0);
		// create an empty scene
		bbhscene=0; 
	}

	// add a point/directional light source to the scene
	// ll.W()==0 means directional, ll.W()==1 means point light
	void pushLight(const vec4<T> ll, const luminance &it)
	{
		if (lights.size()>=MAX_LIGHTS) return;
		lights.push_back(ll); 
		lightit.push_back(it); 
	}

	// set perspective data
	void lookAt(vec3<float> from, vec3<float> at)
	{
		eyepos = from;  
		lookpos = at;
	}

	// Normal vector of a scenepoint (only if scenepoint is on a triangle)
	// if liss[] of object is true the normal is interpolated
	// the normal is oriented according to vector ref, so that dot product ref normal is positive
	vec3<T> getOrientedNormal(vec3<T> ref, scenepoint<T> *ip)
	{
		mesh<T> *m = mm[meshid[ip->getObj()]];
		transform<T> tr = pos[ip->getObj()];
		int faceid = ip->getFace();
		vec3<T> no = m->normalFace(faceid);
		no = tr.applyVec3(no);
		if (ref.dot(no)<0.0f) no.reverse();
		if (!liss[ip->getObj()]) return no; // here, no interpolation is applied
		vec3<T> nn[3];
		nn[0] = m->getFaceVertexNormal(faceid, 0); nn[0]=tr.applyVec3(nn[0]); if (nn[0].dot(no)<0.0) nn[0].reverse();
		nn[1] = m->getFaceVertexNormal(faceid, 1); nn[1]=tr.applyVec3(nn[1]); if (nn[1].dot(no)<0.0) nn[1].reverse();
		nn[2] = m->getFaceVertexNormal(faceid, 2); nn[2]=tr.applyVec3(nn[2]); if (nn[2].dot(no)<0.0) nn[2].reverse();
		no=ip->getTriangle().interpolate(nn);
		no.normalize(no.norm());
		return no;
	}
	// Normal vector of a scenepoint (only if scenepoint is on a triangle)
	// No interpolation is applied
	vec3<T> getNormal(scenepoint<T> *ip)
	{
		mesh<T> *m = mm[meshid[ip->getObj()]];
		transform<T> tr = pos[ip->getObj()];
		int faceid = ip->getFace();
		vec3<T> no = m->normalFace(faceid);
		no = tr.applyVec3(no);
		return no;
	}

	// Builds the bounding box hierarchy of the objects and the scene
	void buildBBH()
	{
		int i,j;
		box3<T> *blistobj = new box3<T> [meshid.size()];
		// compute a hierarchy for each object 
		for (i=0; i<meshid.size(); i++)
		{
			mesh<T> *m = mm[meshid[i]];
			transform<T> tr = pos[i];
			box3<T> *blist=new box3<T> [m->nFaces()];
			// compute boxes of all triangles
			for (j=0; j<m->nFaces(); j++)
			{
				vec3<T> a=m->getFaceVertex((unsigned int)j, 0); a=tr.apply(a);
				vec3<T> b=m->getFaceVertex((unsigned int)j, 1); b=tr.apply(b);
				vec3<T> c=m->getFaceVertex((unsigned int)j, 2); c=tr.apply(c);
				box3<T> bb(a); bb.addPoint(b); bb.addPoint(c);
				blist[j]=bb;
			}
			bbhobj.push_back(new bbh<T>((unsigned int)m->nFaces(),blist, MAX_IN_BOX_FACE));
			delete [] blist;
			blistobj[i] = bbhobj[i]->getBox();
		}
		// compute the global scene hierarchy 
		bbhscene = new bbh<T>((unsigned int)meshid.size(),blistobj, MAX_IN_BOX_OBJ);
		delete [] blistobj;
		printf("scene hierarchy contains %ld objects\n", meshid.size());
		vec3<T> min=bbhscene->getBox().minPoint();
		vec3<T> max=bbhscene->getBox().maxPoint();
		printf("box: %g,%g,%g - %g,%g,%g\n",min.X(), min.Y(), min.Z(), max.X(), max.Y(), max.Z());
	}

	// importing an OBJ file with MTL file
	//////////////////////////////////////////////
	void importOBJ(const char *name)
	{
		FILE *fd;
		int nobj=0;
		int nbrdf = 0;
		bool first=false;
		vec3<T> min,max;
		float xx,yy,zz;
		const unsigned int FNAME_LEN = 256;
		char fname[FNAME_LEN], oname[FNAME_LEN], brdfname[MAX_BRDF][FNAME_LEN];
		char buff[FNAME_LEN];

		// load materials in .mtl file (if it exists)
		strcpy(fname, name); strcat(fname,".mtl");
		fd = fopen(fname, "r");
		if (fd==0) 
		{ 
			printf("no mtl file for %s", fname);
			phongbrdf *r = new phongbrdf("default", luminance(1.0), luminance(0.0), 0.0f);
			refl.push_back(r);
			nbrdf=1;
		}
		else
		{
			luminance Kd(1.0f), Ks(0.0f), Kr(0.0f), Kt(0.0f); float nn=0.0f, irefract=1.0f;
			// parse the file
			do { fgets(buff, 254, fd); if (strlen(buff)>=255) printf("error in file: line too long!"); } while (buff[0]=='#' || buff[0]=='\n'); 
			do {
					if (strncmp(buff,"newmtl",6)==0)
					{
						if (first) // found yet another material
						{
							printf("loaded material %d: %s\n", nbrdf, brdfname[nbrdf]);
							if (nn<=1.0f) nn=1.0f;
							// create  material
							phongbrdf *r = new phongbrdf(brdfname[nbrdf], Kd, Ks, nn);
							r->setMirrorReflection(Kr);
							r->setTransparency(Kt);
							r->setRefractionIndex(irefract);
							// add it to the scene
							refl.push_back(r);
							nbrdf+=1;
							// reset data for next material
							Kd=luminance(1.0f); Ks=luminance(0.0f); Kr=luminance(0.0f); Kt=luminance(0.0f); nn=0.0f; irefract=1.0f;
							strcpy(brdfname[nbrdf],buff+7); brdfname[nbrdf][strlen(brdfname[nbrdf])-1]=0;
						}
						else
						{
							if (strlen(buff) > 8) { strcpy(brdfname[nbrdf], buff + 7); brdfname[nbrdf][strlen(brdfname[nbrdf]) - 1] = 0; }
							else strcpy(brdfname[nbrdf], "default");
							printf("loading materials file...\n");
							first=true;
						}
					}
					else if (buff[0]=='N'  && buff[1]=='s')
					{
						sscanf(buff+3,"%g", &nn);
					}
					else if (buff[0]=='K'  && buff[1]=='d')
					{
						sscanf(buff+3,"%g %g %g", &xx, &yy, &zz);
						Kd=luminance(xx,yy,zz);
					}
					else if (buff[0]=='K'  && buff[1]=='s')
					{
						sscanf(buff+3,"%g %g %g", &xx, &yy, &zz);
						Ks=luminance(xx,yy,zz);
					}
					else if (buff[0]=='K'  && buff[1]=='r')
					{
						sscanf(buff+3,"%g %g %g", &xx, &yy, &zz);
						Kr=luminance(xx,yy,zz);
					}
					else if (buff[0]=='K'  && buff[1]=='t')
					{
						sscanf(buff+3,"%g %g %g", &xx, &yy, &zz);
						Kt=luminance(xx,yy,zz);
					}
					else if (buff[0]=='I'  && buff[1]=='R')
					{
						sscanf(buff+3,"%g", &irefract);
					}
					do { fgets(buff, 254, fd); if (strlen(buff)>=255) printf("error in file: line too long!"); } while (!feof(fd) && (buff[0]=='#' || buff[0]=='\n'));
			} while (!feof(fd));

			printf("loaded material: %s\n", brdfname[nbrdf]);
			if (nn<=1.0f) nn=1.0f;
			// create material
			phongbrdf *r = new phongbrdf(brdfname[nbrdf], Kd, Ks, nn);
			r->setMirrorReflection(Kr);
			r->setTransparency(Kt);
			r->setRefractionIndex(irefract);
			// add it to the scene
			refl.push_back(r);
			nbrdf+=1;
			printf("Stored %d materials\n", nbrdf);
		}

		// load geometry in .obj file
		first=false;
		strcpy(fname, name); strcat(fname,".obj");
		fd = fopen(fname, "r");
		if (fd==0) { printf("cannot open file %s.obj", fname); return; }
		
		const int INITV = 500;
		int ns=0, nf=0, nno=0, nt=0, oldns=0;
		int ibrdf=0;
		int maxns= INITV, maxnf= INITV, maxno= INITV, maxnt= INITV;
		unsigned int *lface;
		T *vertices, *normals, *tcoord; 
		int i,j, indv[4], indno[4], indt[4];
		
		vertices = (T *)malloc(sizeof(T)*INITV *3);
		lface = (unsigned int *)malloc(sizeof(unsigned int)*INITV *3);
		normals = (T *)malloc(sizeof(T)*INITV *3);
		tcoord = (T *)malloc(sizeof(T)*INITV *2);

		do { fgets(buff, FNAME_LEN-1, fd); if (strlen(buff)>= FNAME_LEN) printf("error in file: line too long!"); } while (buff[0]=='#' || buff[0]=='\n');
		do {
				if (buff[0]=='o'  && buff[1]==' ')
				{
					if (first) // found yet another mesh in file
					{ 
						if (nno==0) { free(normals); normals=0; }
						mm.push_back(new mesh<T>(ns,nf, lface, vertices, normals));
						// update bounding box of scene
						if (nobj==0) { min=mm[nobj]->getMin(); max=mm[nobj]->getMax(); }
						else { min.keepMin(min,mm[nobj]->getMin()); max.keepMax(max,mm[nobj]->getMax()); }
						// object has texture coordinates
						if (nt>0) 
						{
							mm[nobj]->pushAttribute(tcoord, 2);
						}
						else free(tcoord);
						// add the object to the scene: its position is identity transform
						meshid.push_back(nobj);
						transform<T> tr; tr.setIdentity();
						pos.push_back(tr);
						brdfid.push_back(ibrdf);
						liss.push_back(LISSAGE);
						nobj++;
						// reset data for next object
						printf("loaded object %s: %d v, %d f, material %d\n", oname, ns, nf, ibrdf);
						strcpy(oname, buff+2); oname[strlen(oname)-1]=0;
						oldns=oldns+ns; ns=0; nf=0; nno=0; nt=0; ibrdf=0;
						maxns= INITV; maxnf= INITV; maxno= INITV; maxnt= INITV;
						vertices = (T *)malloc(sizeof(T)*INITV *3);
						lface = (unsigned int *)malloc(sizeof(unsigned int)*INITV *3);
						normals = (T *)malloc(sizeof(T)*INITV *3);
						tcoord = (T *)malloc(sizeof(T)*INITV *2);
					}
					else { strcpy(oname, buff+2); oname[strlen(oname)-1]=0; first=true; }
				}
				else if (strncmp(buff,"usemtl",6)==0)
				{
					buff[strlen(buff)-1]=0;
					for (i=0; i<nbrdf; i++) if (strlen(brdfname[i])>0 && strcmp(brdfname[i],buff+7)==0) break;
					if (i<nbrdf) ibrdf=i; else ibrdf=0;
				}
				else if (buff[0]=='v' && buff[1]==' ') // load geometry data : 3D points
				{
						sscanf(buff+2,"%g %g %g", &xx, &yy, &zz);
						if (ns==maxns-1) // array needs to be enlarged
							{
								T *nvert = (T *)malloc(sizeof(T)*(maxns+ INITV)*3);
								for (i=0; i<3*ns; i++) nvert[i]=vertices[i];
								free(vertices);
								vertices=nvert; maxns+= INITV;
							}
						vertices[ns*3]=T(xx); vertices[ns*3+1]=T(zz); vertices[ns*3+2]=T(yy);
						ns++;
				}
				else if (buff[0]=='v' && buff[1]=='t') // load texture coordinates
				{
						sscanf(buff+2,"%g %g", &xx, &yy);
						if (nt==maxnt) // array needs to be enlarged
							{
								T *nvert = (T *)malloc(sizeof(T)*(maxnt+ INITV)*2);
								for (i=0; i<2*maxnt; i++) nvert[i]=tcoord[i];
								free(tcoord);
								tcoord=nvert; maxnt+= INITV;
							}
						tcoord[nt*2]=T(xx); tcoord[nt*2+1]=T(yy); nt++;
				}
				else if (buff[0]=='v' && buff[1]=='n') // load normals
				{
						sscanf(buff+2,"%g %g %g", &xx, &yy, &zz);
						if (nno==maxno) // array needs to be enlarged
							{
								T *nvert = (T *)malloc(sizeof(T)*(maxno+ INITV)*3);
								for (i=0; i<3*maxno; i++) nvert[i]=normals[i];
								free(normals);
								normals=nvert; maxno+= INITV;
							}
						normals[nno*3]=T(xx); normals[nno*3+1]=T(yy); normals[nno*3+2]=T(zz); nno++;
				}
				else if (buff[0]=='f' && buff[1]==' ') // load topoly: faces
				{
						if (ns>0 && nno>0 && nt>0) // with normals and with texture coordinates
						{
							int nfe=3;
							indv[0]=0; indv[1]=0; indv[2]=0; indv[3]=0;
							indt[0]=0; indt[1]=0; indt[2]=0; indt[3]=0;
							indno[0]=0; indno[1]=0; indno[2]=0; indno[3]=0;
							sscanf(buff+2,"%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d", indv, indt, indno, indv+1, indt+1, indno+1, indv+2, indt+2, indno+2, indv+3, indt+3, indno+3);
							if (indv[3]!=0) { nfe=4; }
							indv[0]-=oldns; indv[1]-=oldns; indv[2]-=oldns; indv[3]-=oldns;
							for (j=0; j<nfe; j++)
							{
								if (indv[j]<=0||indv[j]>ns) { printf("vertex index out of range"); return; }
								if (indt[j]<=0||indt[j]>nt) { printf("texture vertex index out of range"); return;}
								if (indno[j]<=0||indno[j]>nno) { printf("normal index out of range"); return; }
							}
							lface[nf*3]=indv[0]-1; lface[nf*3+1]=indv[1]-1; lface[nf*3+2]=indv[2]-1; nf++;
							if (nfe==4) { lface[nf*3]=indv[0]-1; lface[nf*3+1]=indv[2]-1; lface[nf*3+2]=indv[3]-1; nf++; }
							if (nf>=maxnf-4) // array needs to be enlarged
								{
									unsigned int *nfa = (unsigned int *)malloc(sizeof(T)*(maxnf+ INITV)*3);
									for (i=0; i<3*maxnf; i++) nfa[i]=lface[i];
									lface=nfa; maxnf+= INITV;
								}
						}
						else if (ns>0 && nno==0 && nt>0) // with texture coordinates but no normals
						{
							int nfe=3;
							indv[0]=0; indv[1]=0; indv[2]=0; indv[3]=0;
							indt[0]=0; indt[1]=0; indt[2]=0; indt[3]=0;
							sscanf(buff+2,"%d/%d %d/%d %d/%d %d/%d", indv, indt, indv+1, indt+1, indv+2, indt+2, indv+3, indt+3);
							if (indv[3]!=0) { nfe=4; }
							indv[0]-=oldns; indv[1]-=oldns; indv[2]-=oldns; indv[3]-=oldns;
							for (j=0; j<nfe; j++)
							{
								if (indv[j]<=0||indv[j]>ns) { printf("vertex index out of range"); return;}
								if (indt[j]<=0||indt[j]>nt) { printf("texture vertex index out of range"); return; }
							}
							printf("face %d:%d,%d,%d\n", nf, indv[0],indv[1],indv[2]);
							lface[nf*3]=indv[0]-1; lface[nf*3+1]=indv[1]-1; lface[nf*3+2]=indv[2]-1; nf++;
							if (nfe==4) { lface[nf*3]=indv[0]-1; lface[nf*3+1]=indv[2]-1; lface[nf*3+2]=indv[3]-1; nf++; }
							if (nf>=maxnf-4) // array must be enlarged
								{
									unsigned int *nfa = (unsigned int *)malloc(sizeof(T)*(maxnf+ INITV)*3);
									for (i=0; i<3*maxnf; i++) nfa[i]=lface[i];
									lface=nfa; maxnf+= INITV;
								}
						}
						else if (ns>0 && nno==0 && nt==0) // with no texture coordinates and no normals
						{
							int nfe=3;
							indv[0]=0; indv[1]=0; indv[2]=0; indv[3]=0;
							sscanf(buff+2,"%d %d %d %d", indv, indv+1, indv+2, indv+3);
							if (indv[3]!=0) { nfe=4; }
							indv[0]-=oldns; indv[1]-=oldns; indv[2]-=oldns; indv[3]-=oldns;
							for (j=0; j<nfe; j++)
							{
								if (indv[j]<=0||indv[j]>ns) { printf("vertex index out of range face %d ind=%d (old=%d)",nf,indv[j], oldns); return;}
							}
							lface[nf*3]=indv[0]-1; lface[nf*3+1]=indv[1]-1; lface[nf*3+2]=indv[2]-1; nf++;
							if (nfe==4) { lface[nf*3]=indv[0]-1; lface[nf*3+1]=indv[2]-1; lface[nf*3+2]=indv[3]-1; nf++; }
							if (nf>=maxnf-4) // array must be enlarged
								{
									unsigned int *nfa = (unsigned int *)malloc(sizeof(T)*(maxnf+ INITV)*3);
									for (i=0; i<3*maxnf; i++) nfa[i]=lface[i];
									free(lface);
									lface=nfa; maxnf+= INITV;
								}
						}
						else printf("cannot read face!\n");
				}
			do { fgets(buff, FNAME_LEN-1, fd); if (strlen(buff)>= FNAME_LEN) printf("error in file: line too long!"); } while (!feof(fd) && (buff[0]=='#' || buff[0]=='\n'));
		} while (!feof(fd));
		if (nno==0) { free(normals); normals=0; }
		// add the object to the scene
		mm.push_back(new mesh<T>(ns,nf, lface, vertices, normals));
		min.keepMin(min,mm[nobj]->getMin()); max.keepMax(max,mm[nobj]->getMax());
		if (nt>0) 
		{
			mm[nobj]->pushAttribute(tcoord, 2);
		}
		else free(tcoord);
		meshid.push_back(nobj);
		transform<T> tr; tr.setIdentity();
		pos.push_back(tr);
		liss.push_back(LISSAGE);
		brdfid.push_back(ibrdf);
		nobj++;
		printf("loaded object %s: %d v, %d f, material %d\n", oname, ns, nf, ibrdf);
		// eventually build the bounding box hierarchy
		if (WBBH) buildBBH();
		else for (int i = 0; i < meshid.size(); i++) bbhobj.push_back(0);
	}



//  RAYTRACING
////////////////////////////////////////////////////////////

// Apply classical raytracing algorithm
// save resulting PPM image in file pname
	virtual void raytracing(const char *pname)
	{
			int i,j;

			// compute perspective projection according to camera data
			vec3<float> dir,up; 
			dir.PVec(eyepos, lookpos);
			dir.normalize(dir.norm());
			up.orthogonalInPlan(dir,vec3<float>(0.0,0.0,1.0));
			perspective<float> persp(eyepos,lookpos, up, LENSE, (float)(LENSE*H/W), 1.0f, 50.0f);
			
			// allocate a picture
			unsigned char *picture=(unsigned char *)malloc(sizeof(unsigned char)*W*H*4);
			for (i=0; i<W*H*4; i++) picture[i]=0;
			
			// do raytracing: for each pixel of the screen
			for (j=0; j<H; j++) 
			{
				if (j % 10 == 0) printf("line %d\n", j);
					for (i=0; i<W; i++) 
					{
						// compute line
						line3<float> dd = persp.reverse(vec3<float>(1.0-2.0*((float)i+0.5)/(float)W, -1.0+2.0*((float)j+0.5)/(float)H, 0.0));
						// cast the ray
						luminance col = primaryRay(dd);
						// store returned color into image
						col.clamp(0.0f, 1.0f);
						picture[4*(i+j*W)]=(unsigned char)(col.RED()*255.0);
						picture[4*(i+j*W)+1]=(unsigned char)(col.GREEN()*255.0);
						picture[4*(i+j*W)+2]=(unsigned char)(col.BLUE()*255.0);
					}
			}
			// saving picture as PPM
			FILE * fp = fopen(pname, "wb");
			fprintf(fp, "P6\n");
			fprintf(fp, "%d %d\n", W, H);
			fprintf(fp, "255\n");
			for (i = 0; i<H; i++)
				for (j = 0; j<W; j++)
				{
					unsigned char v;
					v = (unsigned char)((unsigned char)picture[4 * (j + (H-i-1)*W)]);
					fwrite(&v, 1, sizeof(unsigned char), fp);
					v = (unsigned char)((unsigned char)picture[4 * (j + (H - i - 1)*W)+1]);
					fwrite(&v, 1, sizeof(unsigned char), fp);
					v = (unsigned char)((unsigned char)picture[4 * (j + (H - i - 1)*W)+2]);
					fwrite(&v, 1, sizeof(unsigned char), fp);
				}
			fclose(fp);
	}


	// cast primary rays
	luminance primaryRay(line3<float> dd)
	{
		int count;
		scenepoint<T>  ip;
		// check if an object is intersected, gets closest intersection scenepoint in ip
		int first=this->trace(bbhscene, dd, &ip, 0, 0, 0);
		// if no intersection return back color
		if (first==0) return back;
		// compute shading for closest point ip
		return localIllumination(dd, &ip, 0);
	}

	// cast secondary rays
	// only difference with primary ray cast is that there is a "start" scenepoint
	luminance secondaryRay(scenepoint<T> *start, line3<float> dd, int depth)
	{
		int count;
		scenepoint<T>  ip;
		// check if an object is intersected, gets closest intersection scenepoint in ip
		int first=this->trace(bbhscene, dd, &ip, start, 0, 0);
		// if no intersection return back color
		if (first==0) return back;
		// compute shading for closest point ip
		return localIllumination(dd, &ip, depth);
	}

	// compute local illumination using lighting data on intersection scenepoint ip
	luminance localIllumination(line3<float> dd, scenepoint<T> *ip, int depth)
	{
		// get BRDF and Normal
		brdf *r = refl[brdfid[ip->getObj()]];
		vec3<T> obsdir = dd.direction(); obsdir.reverse();
		vec3<T> no = getOrientedNormal(obsdir, ip);
		vec3<T> uu; uu.orthogonal(no);
		vec3<T> vv; vv.cross(no, uu); vv.normalize(vv.norm());

		int i;
		luminance col(0.0f);
		// for all light sources
		for (i=0; i<lights.size(); i++)
		{
			// compute light position and direction
			vec3<T> ll=vec3<T>(lights[i].X(), lights[i].Y(), lights[i].Z());
			luminance itin=lightit[i];
			luminance res(0.0f);
			vec3<T> dir;
			if (lights[i].W()!=0.0) dir.PVec(ip->getPoint(), ll); else dir=ll;
			dir.normalize(dir.norm());
			// compute shadow ray direction
			line3<T> dd(ip->getPoint(), dir);
			scenepoint<T> shadow;
			scenepoint<T> end(ll);
			int count;

			float cc = dir.dot(no);
			// if light source on the right side compute its illumination
			if (cc>0.0f)
			{
				itin.scale(cc);
				// cast the shadow ray
				if (!WSHADOW || this->occluder(bbhscene, dd, ip, lights[i].W()==0.0?0:&end, 0)==0)
				{
					// if no shadow apply the brdf, result is in res
					r->apply(itin, no, dir, obsdir, uu, vv, 0, res);
					col+=res;
				}
			}
		}
		// mirror secondary ray
		if (WSPECULAR && r->isMirror() && depth<MAX_BOUNCE)
		{
			// ray direction
			vec3<T> rdir; rdir.reflection(obsdir, no);
			line3<T> rdd(ip->getPoint(), rdir);
			// cast secondary ray and add returned luminance
			luminance res, refl=secondaryRay(ip, rdd, depth+1);
			r->applyMirror(refl, 0, res);
			col += res;
		}
		// transparency secondary ray
		if (WSPECULAR && r->isTransparent() && depth<MAX_BOUNCE)
		{
			vec3<T> rdir; 
			// ray direction
			if (!rdir.refraction(obsdir, no, (double)r->getRefractionIndex()))
			{
				printf("error in refraction: index is complex, you are probably inside the transparent object\n");
			}
			else
			{
				bool isok = false;
				line3<T> rdd(ip->getPoint(), rdir);
				scenepoint<T>  out;
				int objid = ip->getObj();
				mesh<T> *m=mm[meshid[objid]];
				transform<T> tr = pos[objid];
				int count;
				// closest intersection with scene -> entry point
				//if (intersectRayObj(bbhobj[objid], objid, m, tr, rdd, CLOSEST, count, &out, 1, ip, 0, liss[objid], 0)>0)
				if (intersectRayObj(bbhobj[objid], objid, m, tr, rdd, CLOSEST, count, &out, 1, ip, 0, false, 0)>0)
				{
					rdir.reverse();
					vec3<T> noout = getOrientedNormal(rdir, &out);
					vec3<T> uuout; uuout.orthogonal(noout);
					vec3<T> vvout; vvout.cross(noout, uuout); vvout.normalize(vvout.norm());
					vec3<T> odir; 
					// compute outgoing point, if it exists by inverting the refraction index
					if (!odir.refraction(rdir, noout, 1.0/(double)r->getRefractionIndex()))
					{
						printf("error in refraction while computing outgoing ray: the ray is locked inside the object\n");
						//col = luminance(1.0f);
					}
					else
					{
						line3<T> rddout(out.getPoint(), odir);
						luminance res, refl=secondaryRay(&out, rddout, depth+1);
						r->applyTransparency(refl, 0, res);
						col += res;
						isok = true;
					}
				}
				else
				{
					printf("error in refraction while computing outgoing ray: the ray has no intersection\n");
					//col = luminance(1.0f,0.0f,0.0f);
				}
				if (!isok)
				{
					obsdir.reverse();
					line3<T> rdd(ip->getPoint(), obsdir);
					luminance res, refl=secondaryRay(ip, rdd, depth+1);
					col += res;

				}
			}
		}
		return col;
	}


// intersections with scene objects and visibility computation
/////////////////////////////////////////

	// finds intersection with closest point between start and end. (if end=0 then end=infinity)
	// Returns 0 (no intersection) or 1. Resulting scenepoint (if existing) is in ip.
	int trace(bbh<T> *bb, line3<T> dd, scenepoint<T> *ip, scenepoint<T> *start, scenepoint<T> *end = 0, bool *onoff = 0)
	{
		int countip=0, maxit = 1;
		return this->visibility(bb, dd, CLOSEST, countip, ip, maxit, start, end, onoff);
	}
	// tests if there is an occluderbetween start and end. Returns 0 (no occluder) or 1. 
	int occluder(bbh<T> *bb, line3<T> dd, scenepoint<T> *start, scenepoint<T> *end = 0, bool *onoff = 0)
	{
		int countip = 0, maxit = 1;
		scenepoint<T> ip;
		return this->visibility(bb, dd, OCCLUDER, countip, &ip, maxit, start, end, onoff);
	}
	// finds all intersections between start and end. (if end=0 then end=infinity)
	// The number is limited to maxit. ip must be an array. Returns the number of intersections
	int traceall(bbh<T> *bb, line3<T> dd, int &countip, scenepoint<T> ip[], int maxit, scenepoint<T> *start, scenepoint<T> *end = 0, bool *onoff = 0)
	{
		return this->visibility(bb, dd, COMPLETE, countip, ip, maxit, start, end, onoff);
	}

	// The three previous functions do all use this "visibility" function
	// it computes visibility between start and end according to type of visibility (if end=0 then end=infinity
	// vtype=CLOSEST means a single closest point returned in ip (return is 0 or 1)
	// vtype=OCCLUDER means there is an occlusion between start and end (stops once a single occluder is found), return is 0 or 1
	// vtype=COMPLETE means all intersections are stored in array ip (not more than maxit), return is number of intersections
	
	// all visibility is computed only for objects that have "true" in onoff table, if onoff==0 all objects are true
	int visibility(bbh<T> *bb, line3<T> dd, VISIBILITY_TYPE vtype, int &countip, scenepoint<T> *ip, int maxit, scenepoint<T> *start, scenepoint<T> *end=0, bool *onoff=0)
	{
		int res=0;
		int i;
		intersection3<T> it;
		// there is no object hierarchy, all objects are tested individually
		if (bb==0) 
		{ 
			for (i=0; i<meshid.size(); i++) 
			{
				mesh<T> *m=mm[meshid[i]];
				transform<T> tr = pos[i];
				int tst = this->intersectRayObj(bbhobj[i], i, m, tr, dd, vtype, countip, ip, maxit, start, end, liss[i], onoff);
				if (vtype==OCCLUDER && tst>=1) return 1;
				if (vtype==CLOSEST && tst>=1) res=1;
				else res+=tst;
			}
			return res;
		}
		// there is a hierarchy, so do a depth first walkthrough the hierarchy
		box3<T> bo=bb->getBox();
		int id;
		T distmax=T(-1);
		// check if ray origin is inside the box
		bool startinside=bo.isInside(dd.origin(),SCENE_PRECISION);
		bool ok=startinside;
		if (!startinside)
		{
				id=it.intersectRay(dd,bo.minPoint(), bo.maxPoint());
				if (id>=1) ok=true;
		}
		if (!ok) return 0;
		// if intersection with box is further than current closest intersection, then leave 
		if (vtype==CLOSEST && !startinside && id>=1 && ip->getObj()!=-1)
			{
			if (id==2) { if (it.distance(0)>ip->distance() && it.distance(1)>ip->distance()) return 0; }
			else { if (it.distance(0)>ip->distance()) return 0; }
			}
		if (end!=0)
		{
			vec3<T> se; se.PVec(start->getPoint(), end->getPoint());
			distmax = se.norm();
			if (!startinside) 
				{
					if (id==2) { if (it.distance(0)>distmax && it.distance(1)>distmax) return 0; }
					else { if (it.distance(0)>distmax) return 0; }
				}
		}
		// check intersections with hierarchy leaves, each leaf is an object of the scene
		if (bb->nLeafs()>0)
		{
			for (i=0; i<bb->nLeafs(); i++) 
			{
				int objid = bb->getLeaf(i);
				mesh<T> *m=mm[meshid[objid]];
				transform<T> tr = pos[objid];
				// test visibility for the individual object
				int tst = this->intersectRayObj(bbhobj[objid], objid, m, tr, dd, vtype, countip, ip, maxit, start, end, liss[objid], onoff);
				if (vtype==OCCLUDER && tst>=1) return 1;
				if (vtype==CLOSEST && tst>=1) res=1;
				else res+=tst;
			}
		}
		// check also nodes of the hierarchy
		// first, sort the nodes according to distance
		int j,k;
		int nbb=0; int bid[8]; T dists[8];
		for (i=0; i<8; i++) if (bb->getNode(i)!=0)
		{
			vec3<T> fr; fr.PVec(bb->getNode(i)->getBox().middle(), dd.origin());
			T ds=fr.norm();
			for (j=0; j<nbb; j++) if (ds<dists[j]) break;
			for (k=nbb-1; k>=j; k--) { bid[k+1]=bid[k]; dists[k+1]=dists[k]; }
			bid[j]=i; dists[j]=ds; nbb++;
		}
		// now check the nodes recursively
		for (i=0; i<nbb; i++)
			if (bb->getNode(bid[i])!=0) 
			{
				int tst = this->visibility(bb->getNode(bid[i]), dd, vtype, countip, ip, maxit, start, end, onoff);
				if (vtype==OCCLUDER && tst>=1) return 1;
				if (vtype==CLOSEST && tst>=1) res=1;
				else res+=tst;
			}
		return res;
	}

	// same as previous "visibility" operation, but for only one single object "objid" of the scene
	int intersectRayObj(bbh<T> *bb, int objid, mesh<T> *m, const transform<T> &tr, line3<T> dd, VISIBILITY_TYPE vtype, int &countip, scenepoint<T> *ip, int maxit, scenepoint<T> *start, scenepoint<T> *end=0, bool lissage=true, bool *onoff=0)
	{
		T distmax=T(-1);
		int i;
		int res=0;
		intersection3<T> it;
		// test intersection only if object has onoff[] value true
		if (onoff!=0) if (!onoff[objid]) return 0;
		// there is no face hierarchy, all faces (triangles) are tested individually
		if (bb==0) 
		{ 
			for (i = 0; i < m->nFaces(); i++)
			{
				int faceid = i;
				bool tst = true;
				// avoid intersection with the same face or with a connected face for start and end if liss[] is active
				if (WTOPOLOGY && start != 0)
				{
					if (start->getObj() == objid && start->getFace() == faceid) tst = false;
					if (tst && lissage && start->getObj() == objid)
					{
						if (m->areConnected(start->getFace(), faceid)) tst = false;
					}
				}
				if (WTOPOLOGY && end != 0)
				{
					if (end->getObj() == objid && end->getFace() == faceid) tst = false;
					if (tst && lissage && end->getObj() == objid)
					{
						if (m->areConnected(end->getFace(), faceid)) tst = false;
					}
				}
				// compute the intersection with the triangle
				if (tst)
				{
					vec3<T> aa = m->getFaceVertex(i, 0); aa = tr.apply(aa);
					vec3<T> bb = m->getFaceVertex(i, 1); bb = tr.apply(bb);
					vec3<T> cc = m->getFaceVertex(i, 2); cc = tr.apply(cc);
					triangle3<T> tt(aa, bb, cc);
					intersection3<T> it;
					int id = tt.intersectRay(dd, SCENE_PRECISION, it);
					if (vtype == OCCLUDER && id > 0)
					{
						*ip = scenepoint<T>(objid, i, it, tt);
						return 1;
					}
					if (vtype == CLOSEST && id > 0)
					{
						bool ctst = (ip->getObj() >= 0);
						if (ctst) ctst = (ip->distance() > it.distance(0)); else ctst = true;
						if (ctst) *ip = scenepoint<T>(objid, i, it, tt);
						if (ctst) res = 1;
					}
					if (vtype == COMPLETE && id > 0)
					{
						*(ip + countip) = scenepoint<T>(objid, i, it, tt);
						countip++;
						if (countip > maxit) { countip--; }
						res++;
					}
				}
			}
			return res;
		}
		// there is a hierarchy, do a depth first walkthrough the hierarchy
		box3<T> bo=bb->getBox();
		int id;
		bool startinside=bo.isInside(dd.origin(),SCENE_PRECISION);
		bool ok=startinside;
		if (!startinside)
		{
				id=it.intersectRay(dd,bo.minPoint(), bo.maxPoint());
				if (id>=1) ok=true;
		}
		if (!ok) return 0;
		if (vtype==CLOSEST && !startinside && id>=1 && ip->getObj()!=-1)
			{
			if (id==2) { if (it.distance(0)>ip->distance() && it.distance(1)>ip->distance()) return 0; }
			else { if (it.distance(0)>ip->distance()) return 0; }
			}
		if (end!=0)
		{
			vec3<T> se; se.PVec(start->getPoint(), end->getPoint());
			distmax = se.norm();
			if (!startinside) 
				{
					if (id==2) { if (it.distance(0)>distmax && it.distance(1)>distmax) return 0; }
					else { if (it.distance(0)>distmax) return 0; }
				}
		}
		// check intersections with hierarchy leaves, each leaf is a triangle of the object
		if (bb->nLeafs()>0)
		{
			for (i=0; i<bb->nLeafs(); i++) 
			{
				int faceid = bb->getLeaf(i);
				bool tst=true;
				// avoid intersection with the same face or with a connected face for start and end if liss[] is active
				if (WTOPOLOGY && start!=0)
				{
					if (start->getObj()==objid && start->getFace()==faceid) tst=false;
					if (tst && lissage && start->getObj()==objid)
					{
						if (m->areConnected(start->getFace(),faceid)) tst=false;
					}
				}
				if (WTOPOLOGY && end!=0)
				{
					if (end->getObj()==objid && end->getFace()==faceid) tst=false;
					if (tst && lissage && end->getObj()==objid)
					{
						if (m->areConnected(end->getFace(),faceid)) tst=false;
					}
				}
				// compute the intersection with the triangle
				if (tst)
				{
					vec3<T> aa=m->getFaceVertex(faceid,0); aa=tr.apply(aa);
					vec3<T> bb=m->getFaceVertex(faceid,1); bb=tr.apply(bb);
					vec3<T> cc=m->getFaceVertex(faceid,2); cc=tr.apply(cc);
					triangle3<T> tt(aa,bb,cc);
					intersection3<T> it;
					int id=tt.intersectRay(dd, SCENE_PRECISION, it);
					// if there is an intersection, update information according to type of visibility computation
					if (vtype==OCCLUDER && id>0) 
					{ 
						*ip=scenepoint<T>(objid, faceid, it, tt); 
						return 1; 
					}
					if (vtype==CLOSEST && id>0)
					{
						tst=(ip->getObj()>=0);
						if (tst) tst = (ip->distance()>it.distance(0)); else tst=true;
						if (tst) *ip=scenepoint<T>(objid, faceid, it, tt);
						if (tst) res=1;
					}
					if (vtype==COMPLETE && id>0)
					{
						*(ip+countip)=scenepoint<T>(objid, faceid, it, tt);
						countip++;
						if (countip>maxit) { countip--; }
						res++;
					}
				}
			}
		}
		// check also nodes of the hierarchy
		// first, sort the nodes according to distance
		int j,k;
		int nbb=0; int bid[8]; T dists[8];
		for (i=0; i<8; i++) if (bb->getNode(i)!=0)
		{
			vec3<T> fr; fr.PVec(bb->getNode(i)->getBox().middle(), dd.origin());
			T ds=fr.norm();
			for (j=0; j<nbb; j++) if (ds<dists[j]) break;
			for (k=nbb-1; k>=j; k--) { bid[k+1]=bid[k]; dists[k+1]=dists[k]; }
			bid[j]=i; dists[j]=ds; nbb++;
		}
		// now, check the nodes recursively
		for (i=0; i<nbb; i++)
			if (bb->getNode(bid[i])!=0) 
			{
				int tst = this->intersectRayObj(bb->getNode(bid[i]), objid, m, tr, dd, vtype , countip, ip, maxit, start, end, lissage, onoff);
				if (vtype==OCCLUDER && tst>=1) return 1;
				if (vtype==CLOSEST && tst>=1) res=1;
				else res+=tst;
			}
		return res;
	}

};

#endif // !defined(AFX_LINE3_H__EC87CD56_6E08_4390_B876_CEC6D44EEA86__INCLUDED_)
