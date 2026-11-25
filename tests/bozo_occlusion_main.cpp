#include "native_occlusion.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <string>
namespace {
using W=vp2s::OcclusionWords;
void check(bool ok,const std::string &what){if(!ok)throw std::runtime_error(what);}
uint32_t w(float v){return std::bit_cast<uint32_t>(v);}
double f(uint32_t v){return std::bit_cast<float>(v);}
W vec(double x,double y,double z,double a){return {w(float(x)),w(float(y)),w(float(z)),w(float(a))};}
bool near(double got,double want,double scale){return std::fabs(got-want)<=1e-4*std::max(1.0,scale);}
struct V{double x,y,z;};
V operator-(V a,V b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
V crossD(V a,V b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double dotD(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
V unitD(V a){const double l=std::sqrt(dotD(a,a));return {a.x/l,a.y/l,a.z/l};}
V v3(const W &a){return {f(a[0]),f(a[1]),f(a[2])};}
// The leaf's record as 222d70 stores it: corners, normals (negated when the clip face normal's z > 0).
std::array<W,9> recordOf(const vp2s::NativeOccluder &o){
    std::array<W,9> r;for(unsigned k=0;k<4;++k)r[k]=o.corners[k];
    const bool flip=f(o.face[2])>0;
    for(unsigned k=0;k<5;++k){r[4+k]=o.normals[k];if(flip)for(unsigned i=0;i<3;++i)r[4+k][i]^=0x80000000u;}
    return r;
}
}
int main(){try{
    std::mt19937 random(0x0cc1);std::uniform_real_distribution<double> u(-1,1);
    const std::array<W,4> identity={vec(1,0,0,0),vec(0,1,0,0),vec(0,0,1,0),vec(0,0,0,1)};
    unsigned leaves=0,behindCases=0;
    for(unsigned c=0;c<200;++c){
        // A rotation about y and a translation as the world rows (+90: rows, translation in w); a quad in its xy plane.
        const double a=u(random)*3,ca=std::cos(a),sa=std::sin(a),tx=u(random)*50,ty=u(random)*20,tz=100+u(random)*50;
        const std::array<W,4> world={vec(ca,0,sa,tx),vec(0,1,0,ty),vec(-sa,0,ca,tz),vec(0,0,0,1)};
        const double hw=5+10*std::fabs(u(random)),hh=5+10*std::fabs(u(random));
        const std::array<W,4> local={vec(-hw,-hh,0,1),vec(hw,-hh,0,1),vec(hw,hh,0,1),vec(-hw,hh,0,1)};
        const W eye=vec(u(random)*5,u(random)*5,u(random)*5,1);
        const std::array<W,4> rows={vec(2,0,0,0),vec(0,2,0,0),vec(0,0,1,1),vec(0,0,-1,0)};   // a clip-like projection
        const auto o=vp2s::nativeOccluder(world,local,eye,rows);
        V p[4];for(unsigned k=0;k<4;++k){const double lx=f(local[k][0]),ly=f(local[k][1]);
            p[k]={ca*lx+tx,ly+ty,-sa*lx+tz};
            check(near(f(o.corners[k][0]),p[k].x,200) && near(f(o.corners[k][1]),p[k].y,200) && near(f(o.corners[k][2]),p[k].z,200) && f(o.corners[k][3])==1,"corner");}
        const V e=v3(eye);
        const V n[5]={crossD(p[1]-p[0],p[0]-e),crossD(p[2]-p[1],p[1]-e),crossD(p[3]-p[2],p[2]-e),crossD(p[0]-p[3],p[3]-e),crossD(p[0]-p[1],p[2]-p[0])};
        for(unsigned k=0;k<5;++k){const V want=unitD(n[k]),got=v3(o.normals[k]);
            check(near(got.x,want.x,1) && near(got.y,want.y,1) && near(got.z,want.z,1),"normal "+std::to_string(k));}
        for(unsigned k=0;k<4;++k){const double x=2*p[k].x,y=2*p[k].y,z=p[k].z-1,ww=p[k].z;
            check(near(f(o.clip[k][0]),x,400) && near(f(o.clip[k][1]),y,400) && near(f(o.clip[k][2]),z,400) && near(f(o.clip[k][3]),ww,400),"clip corner");
            const auto q=vp2s::nativeOccluderProjection(o.clip[k]);
            check(near(f(q[0]),x/z,4) && near(f(q[1]),y/z,4) && q[2]==o.clip[k][2] && q[3]==o.clip[k][3],"projection");}
        {const V c0=v3(o.clip[0]),c1=v3(o.clip[1]),c2=v3(o.clip[2]);const V face=crossD(c1-c0,c2-c0),got=v3(o.face);
         check(near(got.z,face.z,std::fabs(face.z)) && near(got.x,face.x,std::fabs(face.x)+std::fabs(face.z)),"face normal");}
        // Program 0xd58 against the restated distances; a sphere straight behind the quad's centre (seen from the eye)
        // and small enough is occluded; one far to the side or between the eye and the quad is not.
        const auto record=recordOf(o);
        const V centre={(p[0].x+p[2].x)/2,(p[0].y+p[2].y)/2,(p[0].z+p[2].z)/2},ray=unitD(centre-e);
        const auto verdict=[&](V s,double r){
            const bool flip=f(o.face[2])>0;bool any=false;
            for(unsigned k=0;k<5;++k){const V d=k<4?s-p[k]:s-p[0];V nn=unitD(n[k]);if(flip)nn={-nn.x,-nn.y,-nn.z};any=any || dotD(d,nn)-r<0;}
            return !any;};
        const auto run=[&](V s,double r){return vp2s::nativeOccluded(vec(s.x,s.y,s.z,r),record);};
        const double depth=std::sqrt(dotD(centre-e,centre-e));
        const V behind={centre.x+ray.x*30,centre.y+ray.y*30,centre.z+ray.z*30},front={e.x+ray.x*depth/2,e.y+ray.y*depth/2,e.z+ray.z*depth/2};
        const V side={centre.x+200,centre.y,centre.z+60};
        check(run(behind,0.5)==verdict(behind,0.5) && run(front,0.5)==verdict(front,0.5) && run(side,0.5)==verdict(side,0.5),"0xd58 against the restatement");
        check(!run(front,0.5) && !run(side,0.5),"a sphere in front of or beside the occluder is not occluded");
        // The leaf orients the planes by the undivided clip face normal (an orthographic winding): the geometric
        // expectation holds when the face points along the view axis.
        const bool facing=std::fabs(unitD(n[4]).z)>0.5 && std::fabs(dotD(ray,unitD(n[4])))>0.5 && dotD(ray,unitD(n[4]))*unitD(n[4]).z>0;
        behindCases+=facing;
        if(facing && !run(behind,0.5)){
            std::fprintf(stderr,"case %u: a=%g face=(%g %g %g) ray.n4=%g n4z=%g\n",c,a,f(o.face[0]),f(o.face[1]),f(o.face[2]),dotD(ray,unitD(n[4])),unitD(n[4]).z);
            for(unsigned k=0;k<5;++k){const V d=k<4?behind-p[k]:behind-p[0];std::fprintf(stderr,"  d%u=%g\n",k,dotD(d,unitD(n[k])));}
            check(false,"a small sphere straight behind the occluder is occluded");}
        for(unsigned t=0;t<20;++t){const V s={centre.x+u(random)*40,centre.y+u(random)*40,centre.z+u(random)*80};const double r=std::fabs(u(random))*5;
            check(run(s,r)==verdict(s,r),"random sphere against the restatement");}
        ++leaves;
    }
    (void)identity;check(behindCases>40,"facing-occluder coverage");
    std::printf("%u occluders: corners, unit normals, clip corners, projections and the face normal match the listing restated;"
                " program 0xd58 matches its restatement (behind, in front, beside, 4000 random spheres);"
                " %u spheres straight behind a facing occluder are occluded\n",leaves,behindCases);
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}
