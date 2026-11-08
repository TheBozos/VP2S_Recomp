#include "native_math.h"
#include "bozo/ee/dispatcher.h"
#include "bozo/port.h"
#include "bones_reference_functions.h"
#include "bones_adapted_functions.h"
#include <bit>
#include <cstdio>
#include <cstring>
#include <immintrin.h>
#include <source_location>
#include <stdexcept>
namespace {
void check(bool ok,std::source_location at=std::source_location::current()){
    if(!ok)throw std::runtime_error("bone matrix assertion at "+std::to_string(at.line()));
}
uint32_t product(uint32_t a,uint32_t b){
    const auto sign=(a^b)&0x80000000u;unsigned ea=(a>>23)&255,eb=(b>>23)&255;if(!ea || !eb)return sign;
    uint64_t p=uint64_t((a&0x7fffff)|0x800000)*((b&0x7fffff)|0x800000);int e=int(ea)+int(eb)-127;unsigned shift=23;
    if(p&(1ull<<47)){++e;++shift;}if(e<=0)return sign;check(e<255);return sign|uint32_t(e<<23)|uint32_t((p>>shift)&0x7fffff);
}
uint32_t sum(uint32_t a,uint32_t b){
    if((a&0x7fffffff)<(b&0x7fffffff))std::swap(a,b);
    if(!(b&0x7fffffff))return (a&0x7fffffff)?a:(a&b&0x80000000u);
    int e=(a>>23)&255;unsigned distance=unsigned(e)-((b>>23)&255);
    const int64_t large=(a&0x7fffff)|0x800000,small=distance<=23?(((b&0x7fffff)|0x800000)>>distance):0;
    int64_t value=(a>>31?-large:large)+(b>>31?-small:small);if(!value)return 0;
    const auto sign=value<0?0x80000000u:0u;uint64_t m=value<0?-value:value;
    while(m>=0x1000000){m>>=1;++e;}while(m<0x800000){m<<=1;--e;}
    if(e<=0)return sign;check(e<255);return sign|uint32_t(e<<23)|uint32_t(m&0x7fffff);
}
uint32_t root(uint32_t bits){
    const double target=std::bit_cast<float>(bits);uint32_t low=0,high=0x7f7fffff;
    while(low<high){const auto mid=low+(high-low+1)/2;const double value=std::bit_cast<float>(mid);
        if(value*value<=target)low=mid;else high=mid-1;}return low;
}
template<class T>void put(Runtime &r,uint32_t p,T v){std::memcpy(r.memory.hostPointer(p,sizeof(v)),&v,sizeof(v));}
uint32_t word(Runtime &r,uint32_t p){uint32_t v;std::memcpy(&v,r.memory.hostPointer(p,4),4);return v;}
void referenceTransform(const float matrix[4][4],const float vector[4],unsigned weights,uint32_t out[4]){
    const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
    for(unsigned lane=0;lane<4;++lane){volatile float p=matrix[0][lane]*vector[0];float total=normal(p);
        for(unsigned i=1;i<weights;++i){volatile float product=matrix[i][lane]*vector[i];volatile float next=total+normal(product);total=normal(next);}
        out[lane]=std::bit_cast<uint32_t>(total);}
}
float referencePairDistance(const float a[4],const float b[4],float direction[3]){
    const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
    float squares[3];for(unsigned i=0;i<3;++i){volatile float difference=b[i]-a[i];direction[i]=normal(difference);volatile float square=direction[i]*direction[i];squares[i]=normal(square);}
    volatile float xy=squares[0]+squares[1];volatile float z=1.f*squares[2];volatile float squared=normal(xy)+normal(z);
    volatile float distance=static_cast<float>(std::sqrt(double(normal(squared))));volatile float positive=0.f+distance;return normal(positive);
}
void oracle(Runtime &r,R5900Context *c){
    uint32_t base[4][4],bone[4][4],local[4][4],relative[4][4],result[4][4],output[4][4];
    const auto owner=c->r[20].uw[0],i=c->r[6].uw[0]-1;
    std::memcpy(base,r.memory.hostPointer(c->r[29].uw[0]+0x60,64),64);
    std::memcpy(bone,r.memory.hostPointer(word(r,c->r[17].uw[0]+i*4)+0x90,64),64);
    std::memcpy(local,r.memory.hostPointer(word(r,owner+0x40)+i*64,64),64);
    auto transform=[](uint32_t out[4],const uint32_t m[4][4],const uint32_t v[4]){
        for(unsigned lane=0;lane<4;++lane){auto s=product(m[0][lane],v[0]);
            for(unsigned j=1;j<4;++j)s=sum(s,product(m[j][lane],v[j]));out[lane]=s;}
    };
    for(unsigned j=0;j<3;++j)transform(relative[j],bone,base[j]);std::memcpy(relative[3],bone[3],16);
    for(unsigned j=0;j<3;++j)transform(result[j],relative,local[j]);std::memcpy(result[3],relative[3],16);
    for(unsigned row=0;row<4;++row)for(unsigned col=0;col<4;++col){
        const auto value=result[col][row];output[row][col]=col==row?value:col==3?product(value,0x3f800000):sum(0,value);
    }
    std::memcpy(r.memory.hostPointer(c->r[19].uw[0],64),output,64);
}
struct Host:bozo::ee::Host,bozo::port::Host,vp2s::MotionOwner{
    vp2s::NativeMotion motion;
    vp2s::NativeMotion &nativeMotion()override{return motion;}
    bool qualifiedMotionCallback(Runtime *rt,uint32_t target)const override{
        return target==0x1fcb40 && rt->functions.generated(target)==bones_adapted_motion_setter_0x1fcb40;
    }
    std::map<uint32_t,std::array<uint32_t,4>> referenceMotion;
    unsigned symmetricPairs=0,weightedPairs=0,boxPublications=0;
    bool reference=false;uint32_t selectedInverse=0,translationW=0;
    bool callbackActive=false;uint32_t callbackObject=0;uint8_t callbackFlags=0;unsigned callbacks=0;
    std::array<uint32_t,24> callbackData{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(reference && pc==0x1fcdac){
            const auto stack=c->r[29].uw[0],point=word(*rt,c->r[21].uw[0]+0x38);float matrix[4][4],points[2][4],directions[2][3];
            std::memcpy(matrix,rt->memory.hostPointer(stack+0x150,64),64);std::memcpy(points[0],rt->memory.hostPointer(stack+0x190,16),16);std::memcpy(points[1],rt->memory.hostPointer(point+0x30,16),16);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned which=0;which<2;++which){float value[3],squares[3];
                for(unsigned lane=0;lane<3;++lane){volatile float first=matrix[0][lane]*points[which][0];float total=normal(first);
                    for(unsigned row=1;row<3;++row){volatile float term=matrix[row][lane]*points[which][row];volatile float sum=total+normal(term);total=normal(sum);}
                    volatile float translation=matrix[3][lane]*1.f;volatile float result=total+normal(translation);value[lane]=normal(result);
                    volatile float square=value[lane]*value[lane];squares[lane]=normal(square);}
                volatile float xy=squares[0]+squares[1],z=1.f*squares[2],xyz=normal(xy)+normal(z);volatile float length=static_cast<float>(std::sqrt(double(normal(xyz))));
                volatile float reciprocal=length==0?std::numeric_limits<float>::max():static_cast<float>(1.0/double(length));
                for(unsigned i=0;i<3;++i){volatile float valueScaled=value[i]*normal(reciprocal);directions[which][i]=normal(valueScaled);}
            }
            volatile float x=directions[0][0]*directions[1][0],y=directions[0][1]*directions[1][1],z=directions[0][2]*directions[1][2];
            volatile float xy=normal(x)+normal(y),xyz=normal(xy)+normal(z);const auto result=normal(xyz);std::memcpy(rt->memory.hostPointer(stack+0x140,4),&result,4);return true;
        }
        if(reference && pc==0x216e04){
            float matrix[4][4],point[4];uint32_t out[4];std::memcpy(matrix,rt->memory.hostPointer(c->r[5].uw[0],64),64);
            std::memcpy(point,rt->memory.hostPointer(c->r[29].uw[0],16),16);std::memcpy(out,point,16);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned lane=0;lane<3;++lane){volatile float x=matrix[lane][0]*point[0],y=matrix[lane][1]*point[1],z=matrix[lane][2]*point[2];
                volatile float xy=normal(x)+normal(y),xyz=normal(xy)+normal(z);out[lane]=std::bit_cast<uint32_t>(normal(xyz));}
            std::memcpy(rt->memory.hostPointer(c->r[29].uw[0],16),out,16);return true;
        }
        if(pc==0x1fbf9c)++boxPublications;
        if(pc==0x1fc984)++symmetricPairs;if(pc==0x1fc9cc)++weightedPairs;
        if(reference && (pc==0x1fc908 || pc==0x1fc984 || pc==0x1fc9cc)){
            float a[4],b[4],delta[3];const auto first=pc==0x1fc908?c->r[4].uw[0]+0x30:c->r[5].uw[0],second=pc==0x1fc908?c->r[6].uw[0]+0x30:c->r[4].uw[0];
            std::memcpy(a,rt->memory.hostPointer(first,16),16);std::memcpy(b,rt->memory.hostPointer(second,16),16);const float distance=referencePairDistance(a,b,delta);
            if(pc==0x1fc908){std::memcpy(rt->memory.hostPointer(c->r[29].uw[0],4),&distance,4);return true;}
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            volatile float reciprocal=distance==0?std::numeric_limits<float>::max():1.f/distance;
            uint32_t out[2][4];std::memcpy(out[0],a,16);std::memcpy(out[1],b,16);
            for(unsigned i=0;i<3;++i){volatile float direction=delta[i]*normal(reciprocal);volatile float firstDelta=normal(direction)*(pc==0x1fc984?c->f[0]:c->f[1]);
                volatile float secondDelta=normal(direction)*c->f[0];volatile float baseA=a[i]*1.f,baseB=b[i]*1.f;
                volatile float resultA=normal(baseA)+normal(firstDelta),resultB=normal(baseB)-normal(secondDelta);
                out[0][i]=std::bit_cast<uint32_t>(normal(resultA));out[1][i]=std::bit_cast<uint32_t>(normal(resultB));}
            std::memcpy(rt->memory.hostPointer(first,16),out[0],16);std::memcpy(rt->memory.hostPointer(second,16),out[1],16);return true;
        }
        if(reference && pc==0x1fcb08){
            const auto object=c->r[16].uw[0];const auto flags=*rt->memory.hostPointer(object+1,1);
            float position[4],velocity[4],acceleration[4]{},gravity[4]{};std::array<uint32_t,4> out;
            std::memcpy(position,rt->memory.hostPointer(object+0x30,16),16);std::memcpy(out.data(),position,16);
            std::memcpy(velocity,rt->memory.hostPointer(object+0x50,16),16);
            if(flags&0x40)std::memcpy(acceleration,rt->memory.hostPointer(c->r[5].uw[0],16),16);
            if(flags&1)std::memcpy(gravity,rt->memory.hostPointer(0x359100,16),16);
            const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
            volatile float dtSquared=c->f[12]*c->f[12];const float squared=normal(dtSquared);
            float acc[3];for(unsigned i=0;i<3;++i){volatile float initial=position[i]+0.f;acc[i]=normal(initial);}
            if(flags&1){volatile float force=gravity[1]*squared;volatile float value=acc[1]+normal(force);acc[1]=normal(value);}
            if(flags&0x40)for(unsigned i=0;i<3;++i){volatile float force=acceleration[i]*c->f[12];volatile float value=acc[i]+normal(force);acc[i]=normal(value);}
            if(flags&2){volatile float coefficient=c->f[0]*c->f[12];const float drag=normal(coefficient);
                for(unsigned i=0;i<3;++i){volatile float force=velocity[i]*drag;volatile float value=acc[i]-normal(force);acc[i]=normal(value);}}
            for(unsigned i=0;i<3;++i){volatile float displacement=velocity[i]*c->f[12];volatile float result=acc[i]+normal(displacement);out[i]=std::bit_cast<uint32_t>(normal(result));}
            check(referenceMotion.emplace(c->r[29].uw[0],out).second);return true;
        }
        if(reference && pc==0x1fcb24){
            const auto it=referenceMotion.find(c->r[29].uw[0]);check(it!=referenceMotion.end());
            std::memcpy(rt->memory.hostPointer(c->r[16].uw[0]+0x30,16),it->second.data(),16);referenceMotion.erase(it);return true;
        }
        if(reference && (pc==0x1fb418 || pc==0x1fb610 || pc==0x1fb8a0 || pc==0x1fb200 || pc==0x1fb318 || pc==0x1fb448 || pc==0x1fb640 || pc==0x1fbdf0 || pc==0x1fbe90 || pc==0x1fbf9c)){
            const bool linear=pc==0x1fb200 || pc==0x1fb318 || pc==0x1fb448 || pc==0x1fb640;
            const bool local=pc==0x1fbdf0 || pc==0x1fbe90,world=pc==0x1fbf9c;
            float matrix[4][4]{},vector[4];uint32_t out[4];std::memcpy(matrix,rt->memory.hostPointer(c->r[local || world?3:5].uw[0],linear?48:64),linear?48:64);
            std::memcpy(vector,rt->memory.hostPointer(c->r[local?5:world?6:4].uw[0],16),16);referenceTransform(matrix,vector,linear?3:4,out);
            std::memcpy(rt->memory.hostPointer(c->r[local?2:world?6:3].uw[0],16),out,16);return true;
        }
        if(reference && pc==0x1fb79c){
            float matrix[4][4],vectors[2][4];uint32_t out[2][4];
            std::memcpy(matrix,rt->memory.hostPointer(c->r[6].uw[0],64),64);
            std::memcpy(vectors[0],rt->memory.hostPointer(c->r[5].uw[0],16),16);std::memcpy(vectors[1],rt->memory.hostPointer(c->r[4].uw[0],16),16);
            for(unsigned which=0;which<2;++which)referenceTransform(matrix,vectors[which],which?3:4,out[which]);
            std::memcpy(rt->memory.hostPointer(c->r[5].uw[0],16),out[0],16);std::memcpy(rt->memory.hostPointer(c->r[4].uw[0],16),out[1],16);return true;
        }
        if(pc==0x216cb0){selectedInverse=pc;return false;}
        if(reference && pc==0x216d5c){
            float rows[3][4],point[4];uint32_t out[4];std::memcpy(rows,rt->memory.hostPointer(c->r[5].uw[0],48),48);
            std::memcpy(point,rt->memory.hostPointer(c->r[4].uw[0]+0x30,16),16);std::memcpy(out,point,16);translationW=out[3];
            const auto normal=[](float v){const auto b=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((b&0x7f800000)?b:(b&0x80000000));};
            for(unsigned lane=0;lane<3;++lane){volatile float a=rows[0][lane]*point[0],b=rows[1][lane]*point[1],d=rows[2][lane]*point[2];
                volatile float ab=normal(a)+normal(b);volatile float total=normal(ab)+normal(d);
                out[lane]=std::bit_cast<uint32_t>(normal(total))^0x80000000u;}
            std::memcpy(rt->memory.hostPointer(c->r[5].uw[0]+0x30,16),out,16);return true;
        }
        if(reference && pc==0x2485b4){
            const auto saved=_mm_getcsr();uint32_t basis[3][4],out[3];std::memcpy(basis,rt->memory.hostPointer(c->r[5].uw[0]+0x90,48),48);
            for(unsigned axis=0;axis<3;++axis){const auto a=product(basis[0][axis],basis[0][axis]),b=product(basis[1][axis],basis[1][axis]),d=product(basis[2][axis],basis[2][axis]);
                out[axis]=sum(root(sum(sum(a,b),d)),0xbf800000u)&0x7fffffffu;}
            std::memcpy(rt->memory.hostPointer(c->r[2].uw[0],12),out,12);_mm_setcsr(saved);return true;
        }
        if(reference && (pc==0x23c270 || pc==0x23c2a8 || pc==0x23c2c8)){
            const bool combined=pc!=0x23c2c8;uint32_t first[4],second[4],offset[4]{},out[4];
            std::memcpy(first,rt->memory.hostPointer(c->r[combined?6:5].uw[0],16),16);
            std::memcpy(second,rt->memory.hostPointer(c->r[combined?5:4].uw[0],16),16);
            if(combined)std::memcpy(offset,rt->memory.hostPointer(c->r[3].uw[0],16),16);
            for(unsigned i=0;i<4;++i){out[i]=product(first[i],second[i]);if(combined)out[i]=sum(out[i],offset[i]);}
            std::memcpy(rt->memory.hostPointer(c->r[combined?4:3].uw[0],16),out,16);return true;
        }
        if(reference && (pc==0x20f26c || pc==0x20f2ac)){
            uint32_t matrix[4][4],scale[4],out[4][4];
            std::memcpy(matrix,rt->memory.hostPointer(c->r[4].uw[0],64),64);std::memcpy(scale,rt->memory.hostPointer(c->r[30].uw[0],16),16);
            for(unsigned row=0;row<4;++row)for(unsigned lane=0;lane<4;++lane)out[row][lane]=product(matrix[row][lane],scale[row]);
            std::memcpy(rt->memory.hostPointer(c->r[3].uw[0],64),out,64);return true;
        }
        if(reference && pc==0x1fedfc){
            const auto saved=_mm_getcsr();uint32_t first[4],second[4],out[4];
            std::memcpy(first,rt->memory.hostPointer(c->r[5].uw[0],16),16);std::memcpy(second,rt->memory.hostPointer(c->r[2].uw[0],16),16);
            for(unsigned i=0;i<3;++i){const auto d=sum(first[i],second[i]^0x80000000u);out[i]=product(d,d);}
            out[0]=root(sum(sum(out[0],out[1]),out[2]));out[3]=first[3];
            std::memcpy(rt->memory.hostPointer(c->r[4].uw[0],16),out,16);_mm_setcsr(saved);return true;
        }
        if(reference){check(pc==0x237284 || pc==0x2372bc);oracle(*rt,c);return true;}
        check(vp2s::nativeMathInstruction(r,c,rt,pc));return true;
    }
    void frameEnd(Runtime *,R5900Context *)override{throw std::runtime_error("frame");}
    bool callLibrary(Runtime *,R5900Context *,uint32_t)override{return false;}
    void libraryCall(bozo::ee::Dispatcher &d,R5900Context *c,uint32_t pc)override{
        if(pc==0x172000){
            check(callbackActive && c->r[4].uw[0]==callbackObject);auto &r=d.runtime();
            std::memcpy(r.memory.hostPointer(callbackObject+0x90,64),callbackData.data(),64);
            std::memcpy(r.memory.hostPointer(callbackObject+0x230,32),callbackData.data()+16,32);
            *r.memory.hostPointer(callbackObject+0x20e,1)=callbackFlags;++callbacks;c->r[2].ud[0]=0;c->pc=c->r[31].uw[0];return;
        }
        if(pc!=0x216f50)throw std::runtime_error("library");
        check(c->r[5].uw[0]==c->r[4].uw[0]+0x40);selectedInverse=pc;
        // The general inverse remains a bounded record contract. The rigid
        // branch now executes the actual compiled scalar builder and consumer.
        uint32_t marker[16]{};for(unsigned i=0;i<4;++i)marker[i*5]=0x3f800000;
        marker[0]=0x40000000;
        std::memcpy(d.runtime().memory.hostPointer(c->r[5].uw[0],64),marker,64);
        c->r[2].sd[0]=int32_t(c->r[5].uw[0]);c->pc=c->r[31].uw[0];
    }
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
}
int main(){try{
    Host h;bozo::ee::Dispatcher a(h),b(h);
    a.setFunctions(bones_reference_function_table,bones_reference_function_count,bones_reference_resume_table,bones_reference_resume_count);
    b.setFunctions(bones_adapted_function_table,bones_adapted_function_count,bones_adapted_resume_table,bones_adapted_resume_count);
    a.runtime().port=&h;b.runtime().port=&h;uint32_t random=51;unsigned cases=0;
    for(auto *d:{&a,&b}){d->bindLibrary(0x216f50);d->bindLibrary(0x172000);}
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        constexpr uint32_t owner=0x180000,list=0x190000,bind=0x1a0000,output=0x1b0000,metadata=0x1c0000;
        const uint32_t count=1+seed%11,stack=seed&1?0x70003e00:0x1fff000,selector=(seed>>1)&1;
        put(x,owner+0x34,metadata);put(x,metadata+4,count);put(x,owner+0x3c,list);put(x,owner+0x40,bind);put(x,owner+0x4c,output);put<uint8_t>(x,owner+0x50,selector);
        const auto matrix=[&](uint32_t at){uint32_t values[16];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((112+random%17)<<23);}
            if(seed%8==0)for(unsigned i=0;i<16;++i)values[i]=std::bit_cast<uint32_t>(i%5?0.f:1.f);
            if(seed%8==1)for(unsigned i=0;i<16;++i)values[i]=(i&1)?0x80000000u:0;
            std::memcpy(x.memory.hostPointer(at,64),values,64);};
        matrix(stack+0x60);
        for(unsigned i=0;i<count;++i){const auto bone=0x200000+i*0x100;put(x,list+i*4,bone);matrix(bone+0x90);matrix(bind+i*64);}
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0xd3,sizeof(p.vu0_vf));p.vu0_status=0x1245;p.r[17].ud[0]=list;p.r[20].ud[0]=owner;
        p.r[19].ud[0]=(output+0x40+selector*count*64)|0x30000000;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x2371dc)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x2371dc)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++cases;
    }
    auto &r=b.runtime();constexpr uint32_t owner=0x180000,list=0x190000,output=0x1b0000;
    const auto count=word(r,0x1c0004);const auto selector=*r.memory.hostPointer(owner+0x50,1);
    R5900Context c{};c.r[20].ud[0]=owner;c.r[17].ud[0]=list;c.r[6].ud[0]=count;c.r[7].ud[0]=count;
    c.r[19].ud[0]=output+0x40+selector*count*64+(count-1)*64;c.r[29].ud[0]=0x70003e00;
    auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
        try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x2372bc);}catch(const std::runtime_error &){bad=true;}
        check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
    c.r[6].ud[0]=0;reject();c.r[6].ud[0]=count;c.r[19].ud[0]+=16;reject();c.r[19].ud[0]-=16;
    const auto bind=word(r,owner+0x40);put(r,owner+0x40,output+0x40+selector*count*64);reject();put(r,owner+0x40,bind);
    const auto bone=word(r,list);put(r,list,output+0x40+selector*count*64-0x90);reject();put(r,list,bone);
    put(r,bind+(count-1)*64,0x7fc00000u);reject();
    unsigned distances=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003e00:0x1fff000,destination=stack+0x60;
        const uint32_t first=seed%3==0?destination:0x180000,second=seed%3==1?destination:0x190000;
        uint32_t values[8];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((100+random%40)<<23);}
        if(seed%8==0)for(unsigned i=0;i<3;++i)values[i+4]=values[i];
        if(seed%8==1)for(unsigned i=0;i<8;++i)values[i]=i&1?0x80000000u:0;
        values[3]=0x7fc01234u; // W is carried verbatim, never an XYZ input.
        std::memcpy(x.memory.hostPointer(first,16),values,16);std::memcpy(x.memory.hostPointer(second,16),values+4,16);
        constexpr uint32_t item=0x1a0000,table=0x1b0000;
        const auto index=seed%3;put(x,item+0xc,table);put(x,table+index*64+0x38,second-0x30);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x91,sizeof(p.vu0_vf));p.vu0_status=0x9876;
        p.r[4].ud[0]=destination;p.r[5].ud[0]=first;p.r[6].ud[0]=index;p.r[8].ud[0]=item;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fedc0)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fedc0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));
        check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++distances;
    }
    unsigned scales=0;
    for(unsigned branch=0;branch<2;++branch)for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t object=seed&1?0x70000100:0x180000,source=object+(branch?0x150:0xd0),destination=object+0x1d0;
        const uint32_t coefficient=seed%4==0?destination:seed%4==1?source+16:seed%4==2?0x190000:0x70000800;
        uint32_t values[20];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%30)<<23);}
        if(seed%8==0)for(unsigned i=0;i<20;++i)values[i]=(i&1)?0x80000000u:0;
        if(seed%8==2){for(unsigned i=0;i<16;++i)values[i]=0x00800000u|(i&1?0x80000000u:0);for(unsigned i=16;i<20;++i)values[i]=0x3f000000;}
        std::memcpy(x.memory.hostPointer(source,64),values,64);std::memcpy(x.memory.hostPointer(coefficient,16),values+16,16);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x81,sizeof(p.vu0_vf));p.vu0_status=0x9843;p.r[21].ud[0]=object;p.r[30].ud[0]=coefficient;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);const auto entry=branch?0x20f280:0x20f240;
        h.reference=true;x.functions.generated(entry)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(entry)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++scales;
    }
    {auto &r=b.runtime();constexpr uint32_t object=0x180000;R5900Context c{};c.r[21].ud[0]=object;c.r[4].ud[0]=object+0x150;c.r[3].ud[0]=object+0x1d0;c.r[30].ud[0]=0x190000;
        uint32_t one=0x3f800000;for(unsigned i=0;i<16;++i)put(r,object+0x150+i*4,one);for(unsigned i=0;i<4;++i)put(r,0x190000+i*4,one);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x20f2ac);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;
        put(r,0x190000,0x7fc00000u);reject();put(r,0x190000,1u);reject();put(r,0x190000,0x40000000u);put(r,object+0x150,0x7f7fffffu);reject();
    }
    unsigned parameters=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x71,bozo::kFlatBytes);
        const uint32_t global=seed&1?0x70000100:0x180000,model=seed&2?0x70001000:0x190000,flags=seed%4;
        uint32_t values[16];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%30)<<23);}
        if(seed%8==0)for(unsigned i=0;i<16;++i)values[i]=i&1?0x80000000u:0;
        if(seed%8==1)for(unsigned i=0;i<4;++i){values[i]=std::bit_cast<uint32_t>(i&1?-1.f:1.f);values[i+4]=values[i+8]=values[i]^0x80000000u;values[i+12]=0x3f800000;}
        if(seed%8==2)for(unsigned i=0;i<12;++i){values[i]=0x00800000u|(i&1?0x80000000u:0);values[12+i%4]=0x3f000000;}
        std::memcpy(x.memory.hostPointer(global+0x310,16),values,16);std::memcpy(x.memory.hostPointer(global+0x390,16),values+4,16);
        std::memcpy(x.memory.hostPointer(global+0x3c0,16),values+8,16);std::memcpy(x.memory.hostPointer(model+0x7e0,16),values+12,16);
        *x.memory.hostPointer(global+0x518,1)=flags&1?7:0;*x.memory.hostPointer(global+0x517,1)=flags&2?9:0;
        const uint32_t coefficient=global+(flags&1?0x3c0:flags&2?0x390:0x310);
        const uint32_t stack=seed%4==0?global+0x310-0x50:seed%4==1?coefficient-0x50:seed%4==2?model+0x7e0-0x50:seed&4?0x70003e00:0x1fff000;
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x61,sizeof(p.vu0_vf));p.vu0_status=0x1254;p.r[18].ud[0]=global;p.r[19].ud[0]=model;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x23c240)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x23c240)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));++parameters;
    }
    {auto &r=b.runtime();constexpr uint32_t global=0x180000,model=0x190000;R5900Context c{};
        c.r[18].ud[0]=global;c.r[19].ud[0]=model;c.r[29].ud[0]=0x1fff000;c.r[6].ud[0]=global+0x3c0;c.r[5].ud[0]=model+0x7e0;c.r[3].ud[0]=global+0x310;c.r[4].ud[0]=0x1fff050;
        for(unsigned i=0;i<4;++i){put(r,global+0x310+i*4,0x3f800000u);put(r,global+0x3c0+i*4,0x3f800000u);put(r,model+0x7e0+i*4,0x3f800000u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x23c270);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;
        for(uint32_t bad:{0x7fc00000u,1u}){put(r,global+0x3c0,bad);reject();}
        put(r,global+0x3c0,0x7f7fffffu);put(r,model+0x7e0,0x40000000u);reject();
        put(r,model+0x7e0,0x3f800000u);put(r,global+0x310,0x7f7fffffu);reject();
    }
    unsigned selectors=0,rigid=0,general=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x71,bozo::kFlatBytes);
        const uint32_t object=seed&1?0x70000100:0x180000,stack=seed&2?0x70003e00:0x1fff000;
        float basis[3][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0}};
        switch(seed%10){
        case 1:std::memset(basis,0,sizeof(basis));break;
        case 2:basis[0][0]=2;basis[1][1]=.5f;break;
        case 3:basis[0][0]=std::bit_cast<float>(0x3f800001u);break;
        case 4:basis[0][0]=std::bit_cast<float>(0x3f800008u);break;
        case 5:basis[0][0]=std::bit_cast<float>(0x3f80000au);break;
        case 6:basis[0][0]=-1;basis[2][2]=-1;break;
        case 7:basis[0][1]=.5f;break;
        case 8:basis[0][0]=std::bit_cast<float>(0x00800000u);break;
        case 9:basis[0][0]=basis[0][1]=basis[1][1]=.70710677f;basis[1][0]=-.70710677f;break;
        }
        for(unsigned row=0;row<3;++row)basis[row][3]=std::bit_cast<float>(0x7fc01234u+row);
        std::memcpy(x.memory.hostPointer(object+0x90,48),basis,48);*x.memory.hostPointer(object+0x20e,1)=uint8_t(seed*17);
        uint32_t inverseInput[16]{};for(unsigned i=0;i<4;++i)inverseInput[i*5]=0x3f800000;
        inverseInput[12]=0x3f800000;inverseInput[13]=0xc0000000;inverseInput[14]=0x40400000;inverseInput[15]=0x7fc01234;
        std::memcpy(x.memory.hostPointer(object+0x150,64),inverseInput,64);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        const auto fourth=word(x,stack-4);R5900Context p{};
        for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x83,sizeof(p.vu0_vf));p.vu0_status=0x1245;p.r[4].ud[0]=object;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;h.selectedInverse=0;x.functions.generated(0x248560)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr(),selected=h.selectedInverse;_mm_setcsr(control);
        h.reference=false;h.selectedInverse=0;y.functions.generated(0x248560)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp && h.selectedInverse==selected);_mm_setcsr(saved);
        check(selected==0x216cb0 || selected==0x216f50);rigid+=selected==0x216cb0;general+=selected==0x216f50;
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(word(y,stack-4)==fourth && *y.memory.hostPointer(object+0x20e,1)==uint8_t(seed*17|0x20));++selectors;
    }
    check(rigid && general);
    {auto &r=b.runtime();constexpr uint32_t object=0x180000,stack=0x1fff000;R5900Context c{};c.r[5].ud[0]=object;c.r[29].ud[0]=stack;c.r[2].ud[0]=stack+0x10;
        for(unsigned i=0;i<12;++i)put(r,object+0x90+i*4,i%5==0?0x3f800000u:0u);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x2485b4);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[2].ud[0]+=16;reject();c.r[2].ud[0]-=16;c.r[5].ud[0]=stack;reject();c.r[5].ud[0]=object;
        for(uint32_t bad:{0x7fc00000u,1u,0x7f7fffffu}){put(r,object+0x90,bad);reject();}
    }
    unsigned columns=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t source=seed&1?0x70000100:0x180000;
        const int offsets[8]={0,0,16,32,48,-16,-32,-48};
        const uint32_t destination=seed%8?source+offsets[seed%8]:0x190000;
        uint32_t values[16];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed%8==0)for(unsigned i=0;i<16;++i)values[i]=i%5?0:0x3f800000;
        if(seed%8==1)for(unsigned i=0;i<16;++i)values[i]=i&1?0x80000000u:0;
        const uint32_t payloads[6]={0x7fc01234,0x7f800001,0xffc05678,0x80000000,0xdeadbeef,0x3f800000};
        for(unsigned i=0;i<4;++i)values[i*4+3]=payloads[(seed+i)%6];
        std::memcpy(x.memory.hostPointer(source,64),values,64);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x92,sizeof(p.vu0_vf));p.vu0_status=0x5634;p.r[4].ud[0]=source;p.r[5].ud[0]=destination;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x216cb0)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr(),expectedW=h.translationW;_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x216cb0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(word(y,destination+0x3c)==expectedW);++columns;
    }
    {auto &r=b.runtime();constexpr uint32_t source=0x180000,destination=0x190000;R5900Context c{};c.r[4].ud[0]=source;c.r[5].ud[0]=destination;
        for(unsigned i=0;i<16;++i){put(r,source+i*4,i%5==0?0x3f800000u:0u);put(r,destination+i*4,i%5==0?0x3f800000u:0u);}
        put(r,source+0x30,0x40000000u);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x216d5c);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        for(uint32_t bad:{0x7fc00000u,1u,0x7f7fffffu}){put(r,destination,bad);reject();}
    }
    unsigned pairs=0,pairedCallers=0,calculated=0,cached=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x51,bozo::kFlatBytes);
        const uint32_t matrix=seed&1?0x70000100:0x180000;
        uint32_t point=0x190000,direction=0x1a0000;
        switch(seed%8){case 1:point=matrix;direction=matrix+16;break;case 2:point=matrix+16;direction=matrix+48;break;
            case 3:direction=point;break;case 4:direction=matrix+48;break;case 5:point=matrix+48;break;
            case 6:direction=point+16;break;case 7:point=direction-16;break;}
        uint32_t values[24];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed==0){std::memset(values,0,sizeof(values));for(unsigned i=0;i<4;++i)values[i*5]=0x3f800000;
            values[3]=0x40000000;values[7]=0xc0400000;values[11]=0x40800000;values[15]=0x40a00000;
            values[16]=values[20]=0x3f800000;values[17]=values[21]=0x40000000;values[18]=values[22]=0x40400000;values[19]=0x40000000;}
        if(seed%16==8)for(unsigned i=0;i<24;++i)values[i]=i<16?0x00800000u:0x3f000000u;
        std::memcpy(x.memory.hostPointer(matrix,64),values,64);std::memcpy(x.memory.hostPointer(point,16),values+16,16);std::memcpy(x.memory.hostPointer(direction,16),values+20,16);
        const auto w=direction+12;const bool activeW=(w>=matrix && w<matrix+64) || (w>=point && w<point+16);
        if(!activeW)put(x,w,seed&2?0x7f800001u:0x7fc01234u);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x48,sizeof(p.vu0_vf));p.vu0_status=0x1256;p.r[6].ud[0]=matrix;p.r[5].ud[0]=point;p.r[4].ud[0]=direction;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;check(h.nativeInstruction(x.memory.flat(),&p,&x,0x1fb79c));const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;check(h.nativeInstruction(y.memory.flat(),&q,&y,0x1fb79c));check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(seed==0)check(word(y,point+12)==0x41900000 && word(y,direction+12)==0x41000000);++pairs;
    }
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t object=seed&1?0x70000100:0x180000,stack=seed&2?0x70003e00:0x1fff000;
        constexpr uint32_t vtable=0x1e0000;put(x,object,vtable);put(x,vtable+0x64,0x172000u);
        h.callbackActive=true;h.callbackObject=object;h.callbackFlags=uint8_t((seed*17&~8u)|(seed&4?8:0));
        for(auto &v:h.callbackData){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed%8==0){h.callbackData.fill(0);for(unsigned i=0;i<4;++i)h.callbackData[i*5]=0x3f800000;}
        h.callbackData[23]=0x7fc01234;std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x82,sizeof(p.vu0_vf));p.vu0_status=0x4512;p.r[4].ud[0]=object;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;h.callbacks=0;x.functions.generated(0x1fb6a0)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();check(h.callbacks==1);_mm_setcsr(control);
        h.reference=false;h.callbacks=0;y.functions.generated(0x1fb6a0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp && h.callbacks==1);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(h.callbackFlags&8){++cached;check(*y.memory.hostPointer(object+0x20e,1)==h.callbackFlags);}
        else {++calculated;check(*y.memory.hostPointer(object+0x20e,1)==uint8_t((h.callbackFlags&~0x10)|8));}
        ++pairedCallers;
    }
    h.callbackActive=false;check(calculated && cached);
    {auto &r=b.runtime();constexpr uint32_t matrix=0x180000,point=0x190000,direction=0x1a0000;R5900Context c{};c.r[6].ud[0]=matrix;c.r[5].ud[0]=point;c.r[4].ud[0]=direction;
        for(unsigned i=0;i<16;++i)put(r,matrix+i*4,i%5==0?0x3f800000u:0u);
        for(unsigned i=0;i<4;++i){put(r,point+i*4,0x3f800000u);put(r,direction+i*4,0x3f800000u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fb79c);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[6].ud[0]+=4;reject();c.r[6].ud[0]-=4;put(r,point+12,0x7fc00000u);reject();put(r,point+12,0x3f800000u);
        put(r,matrix+12,0x7fc00000u);reject();put(r,matrix+12,0u);put(r,point,1u);reject();
        put(r,point,0x40000000u);put(r,matrix,0x7f7fffffu);reject();
    }
    unsigned transforms=0,linearTransforms=0;
    for(uint32_t entry:{0x1fb3f4u,0x1fb5ecu,0x1fb87cu,0x1fb1e4u,0x1fb2fcu,0x1fb42cu,0x1fb624u,0x1fbdccu})for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        const bool linear=entry==0x1fb1e4 || entry==0x1fb2fc || entry==0x1fb42c || entry==0x1fb624;
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t matrix=seed&1?0x70000100:0x180000,vector=seed%8==3?matrix+16:seed%8==5?matrix+32:0x190000;
        const uint32_t destination=seed%4==0?matrix:seed%4==1?matrix+32:seed%4==2?vector:0x1a0000;
        uint32_t values[20];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed%16==8)for(unsigned i=0;i<20;++i)values[i]=i<16?0x00800000u:0x3f000000u;
        if(seed%16==0){std::memset(values,0,sizeof(values));for(unsigned i=0;i<4;++i)values[i*5]=0x3f800000;
            values[3]=0x40000000;values[7]=0xc0400000;values[11]=0x40800000;values[15]=0x40a00000;
            values[16]=0x3f800000;values[17]=0x40000000;values[18]=0x40400000;values[19]=0x40000000;}
        if(linear && vector==0x190000)values[19]=seed&2?0xff801234u:0x7fc01234u;
        std::memcpy(x.memory.hostPointer(matrix,64),values,64);std::memcpy(x.memory.hostPointer(vector,16),values+16,16);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x83,sizeof(p.vu0_vf));p.vu0_status=0x1234;
        p.r[entry==0x1fbdcc?3:5].ud[0]=matrix;p.r[entry==0x1fbdcc?5:4].ud[0]=vector;p.r[entry==0x1fbdcc?2:3].ud[0]=destination;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(entry)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(entry)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(seed%16==0)check(word(y,destination+12)==(linear?0x41000000u:0x41900000u));if(linear)++linearTransforms;else ++transforms;
    }
    {auto &r=b.runtime();constexpr uint32_t matrix=0x180000,vector=0x190000,destination=0x1a0000;R5900Context c{};c.r[5].ud[0]=matrix;c.r[4].ud[0]=vector;c.r[3].ud[0]=destination;
        for(unsigned i=0;i<16;++i)put(r,matrix+i*4,i%5==0?0x3f800000u:0u);for(unsigned i=0;i<4;++i)put(r,vector+i*4,0x3f800000u);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fb610);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[3].ud[0]+=4;reject();c.r[3].ud[0]-=4;put(r,vector+12,0x7fc00000u);reject();put(r,vector+12,0x3f800000u);
        put(r,matrix+12,0x7fc00000u);reject();put(r,matrix+12,0u);put(r,vector,1u);reject();
        put(r,vector,0x40000000u);put(r,matrix,0x7f7fffffu);reject();
    }
    {auto &r=b.runtime();constexpr uint32_t matrix=0x180000,vector=0x190000,destination=0x1a0000;R5900Context c{};c.r[5].ud[0]=matrix;c.r[4].ud[0]=vector;c.r[3].ud[0]=destination;
        for(unsigned i=0;i<12;++i)put(r,matrix+i*4,i%5==0?0x3f800000u:0u);for(unsigned i=0;i<4;++i)put(r,vector+i*4,0x3f800000u);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fb640);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[3].ud[0]+=4;reject();c.r[3].ud[0]-=4;put(r,vector,0x7fc00000u);reject();put(r,vector,0x3f800000u);
        put(r,matrix+12,0x7fc00000u);reject();put(r,matrix+12,0u);put(r,vector,1u);reject();
        put(r,vector,0x40000000u);put(r,matrix,0x7f7fffffu);reject();
    }
    unsigned motions=0;std::array<unsigned,8> motionBranches{};
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t object=seed&1?0x70000100:0x180000,owner=seed&8?object:0x1b0000,stack=seed&2?0x70003e00:0x1fff000;
        const uint8_t flags=(seed&1?1:0)|(seed&2?2:0)|(seed&4?0x40:0)|(seed&0x30);
        const uint32_t table=0x1e0000|(uint32_t(flags)<<8),acceleration=seed&16?object+0x50:0x190000;
        for(auto at:{object+0x30,object+0x50,object+0x60,acceleration,0x359100u}){
            uint32_t values[4];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%25)<<23);}
            if(seed%16==0)for(auto &v:values)v=0;std::memcpy(x.memory.hostPointer(at,16),values,16);
        }
        put(x,object+0x3c,seed&1?0xff801234u:0x7fc01234u);put(x,object+0x5c,0x7fc05678u);put(x,acceleration+12,0x7fc05678u);
        put(x,owner,table);put(x,table+0x34,0x1fcb40u);put(x,object+4,owner);*x.memory.hostPointer(object,1)=0;*x.memory.hostPointer(object+1,1)=flags;
        put(x,object+0x28,0x3e800000u);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x35,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=object;p.r[5].ud[0]=acceleration;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;
        p.f[12]=seed%16==0?std::bit_cast<float>(seed&16?0x80000000u:0u):std::bit_cast<float>(0x3d000000u+(seed%8)*0x100000);auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fc9e0)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fc9e0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(h.motion.outstanding()==0 && h.referenceMotion.empty());++motions;++motionBranches[(flags&3)|((flags&0x40)>>4)];
    }
    for(auto count:motionBranches)check(count==64);
    {auto &r=b.runtime();constexpr uint32_t object=0x180000,owner=0x1b0000,table=0x1e0000,acceleration=0x190000;R5900Context c{};
        c.r[16].ud[0]=object;c.r[29].ud[0]=0x1fff000;c.r[6].ud[0]=object+0x30;c.r[3].ud[0]=object+0x50;c.r[4].ud[0]=0x359100;c.r[5].ud[0]=acceleration;c.f[12]=0.5f;c.f[0]=0.25f;
        put(r,owner,table);put(r,table+0x34,0x1fcb40u);put(r,object+4,owner);*r.memory.hostPointer(object+1,1)=0x43;
        for(auto at:{object+0x30,object+0x50,acceleration,0x359100u})for(unsigned i=0;i<4;++i)put(r,at+i*4,0x3f800000u);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fcb08);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()) && h.motion.outstanding()==0);};
        c.r[6].ud[0]+=16;reject();c.r[6].ud[0]-=16;c.f[12]=std::bit_cast<float>(0x7fc00000u);reject();c.f[12]=0.5f;
        put(r,object+0x50,1u);reject();put(r,object+0x50,0x3f800000u);put(r,0x359104,0x7fc00000u);reject();put(r,0x359104,0x3f800000u);
        put(r,table+0x34,0x1fcb44u);reject();put(r,table+0x34,0x1fcb40u);c.f[12]=std::numeric_limits<float>::max();reject();
    }
    {vp2s::NativeMotion updates;const std::array<uint32_t,4> first{1,2,3,0x7fc01234},second{4,5,6,0xff801234};
        updates.prepare(0x1fff000,0x180000,first);updates.prepare(0x1ffef00,0x190000,second);
        bool duplicate=false,wrong=false,stale=false;
        try{updates.prepare(0x1fff000,0x190000,second);}catch(const std::runtime_error &){duplicate=true;}
        try{updates.finish(0x1fff000,0x190000);}catch(const std::runtime_error &){wrong=true;}
        check(updates.outstanding()==2 && updates.finish(0x1ffef00,0x190000)==second && updates.finish(0x1fff000,0x180000)==first);
        try{updates.finish(0x1fff000,0x180000);}catch(const std::runtime_error &){stale=true;}check(duplicate && wrong && stale && updates.outstanding()==0);
    }
    unsigned positionPairs=0,fixedPairs=0;h.symmetricPairs=h.weightedPairs=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t first=seed&1?0x70000100:0x180000,second=seed%8==4?first+16:seed%8==6?first:0x190000;
        const uint32_t constraint=(seed&32) && seed%16?first+16:0x1a0000,stack=seed&2?0x70003e00:0x1fff000;
        for(auto at:{first+0x30,second+0x30}){uint32_t values[4];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((120+random%10)<<23);}
            std::memcpy(x.memory.hostPointer(at,16),values,16);}
        *x.memory.hostPointer(first,1)=seed&1?1:0;*x.memory.hostPointer(second,1)=seed&2?1:0;
        put(x,first+0x24,std::bit_cast<uint32_t>(seed%8==0?0.f:float(seed%7+1)));put(x,second+0x24,std::bit_cast<uint32_t>(seed%8==0?0.f:float(seed%5+1)));
        put(x,constraint+0x28,std::bit_cast<uint32_t>(float(seed%9+1)));put(x,first+0x3c,0x7fc01234u);put(x,second+0x3c,0xff801234u);
        if(seed%16==0){for(unsigned i=0;i<3;++i){put(x,first+0x30+i*4,0u);put(x,second+0x30+i*4,i==0?0x41000000u:0u);}put(x,constraint+0x28,0x40800000u);}
        if(seed%16==8){for(unsigned i=0;i<3;++i){put(x,first+0x30+i*4,0u);put(x,second+0x30+i*4,0u);}}
        const bool fixed=*x.memory.hostPointer(first,1)==1 && *x.memory.hostPointer(second,1)==1;
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x37,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=first;p.r[5].ud[0]=constraint;p.r[6].ud[0]=second;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fc860)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fc860)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(seed%16==0)check(word(y,first+0x30)==0x40000000 && word(y,second+0x30)==0x40c00000);
        if(fixed)++fixedPairs;else for(unsigned i=1;i<4;++i)check(word(y,stack-16+i*4)==0x61616161);
        ++positionPairs;
    }
    check(h.symmetricPairs && h.weightedPairs && fixedPairs);
    {auto &r=b.runtime();constexpr uint32_t first=0x180000,second=0x190000,constraint=0x1a0000,stack=0x1fff000;R5900Context c{};
        c.r[4].ud[0]=first;c.r[5].ud[0]=constraint;c.r[6].ud[0]=second;c.r[29].ud[0]=stack;c.r[3].ud[0]=stack;
        for(unsigned i=0;i<4;++i){put(r,first+0x30+i*4,0u);put(r,second+0x30+i*4,0x3f800000u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fc908);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[5].ud[0]=stack;reject();c.r[5].ud[0]=constraint;c.r[4].ud[0]=stack-0x30;reject();c.r[4].ud[0]=first;
        put(r,first+0x30,0x7fc00000u);reject();put(r,first+0x30,1u);reject();put(r,first+0x30,0x7f7fffffu);reject();
    }
    unsigned boxes=0,clamped=0,unclamped=0;h.boxPublications=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t object=seed&1?0x70000100:0x180000,stack=seed&2?0x70003e00:0x1fff000;
        const uint32_t input=(seed&16) && seed%16?object+0x150:0x190000;
        const uint32_t output=seed%4==0?input:seed%4==1?object+0x150:seed%4==2?object+0x190:0x1a0000;
        for(auto at:{object+0x150,object+0x190}){
            uint32_t values[16];for(auto &v:values){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((120+random%10)<<23);}
            if(seed%16==0)for(unsigned i=0;i<16;++i)values[i]=i%5==0?0x3f800000u:0u;
            std::memcpy(x.memory.hostPointer(at,64),values,64);
        }
        uint32_t point[4];for(auto &v:point){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((120+random%10)<<23);}
        if(seed%16==0){point[0]=seed%32==0?0xc1200000u:0u;point[1]=0;point[2]=seed%32==0?0x41200000u:0u;point[3]=0x3f800000;}
        std::memcpy(x.memory.hostPointer(input,16),point,16);
        for(unsigned i=0;i<3;++i)put(x,object+0x220+i*4,std::bit_cast<uint32_t>(float(5+i)));
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x39,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=object;p.r[5].ud[0]=input;p.r[6].ud[0]=output;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;p.f[12]=1.f;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fbe60)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fbe60)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(q.r[2].ud[0])++clamped;else ++unclamped;
        if(seed%32==0)check(word(y,output)==0xc0800000 && word(y,output+8)==0x40c00000 && word(y,output+12)==0x3f800000);
        if(seed%32==16)check(q.r[2].ud[0]==0 && word(y,output+12)==0x3f800000);++boxes;
    }
    check(clamped && unclamped && h.boxPublications==clamped*2);
    unsigned transposedInverses=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t source=seed&1?0x70000100:0x180000,stack=seed&2?0x70003e00:0x1fff000;
        const uint32_t output=seed%8==0?0x1a0000:seed%8==1?source:seed%8==2?source+16:seed%8==3?source-16:seed%8==4?source+32:seed%8==5?source-32:seed%8==6?source+48:source-48;
        uint32_t matrix[16];for(auto &v:matrix){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed%16==0)for(unsigned i=0;i<16;++i)matrix[i]=std::bit_cast<uint32_t>(float(i+1));
        for(unsigned i=12;i<16;++i)matrix[i]=0x7f800001u+i;
        std::memcpy(x.memory.hostPointer(source,64),matrix,64);put(x,stack-4,0xff801234u);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x41,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=source;p.r[5].ud[0]=output;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x216d60)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x216d60)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(word(y,stack-4)==0xff801234u);
        if(seed%16==0)check(word(y,output+12)==0xc3180000 && word(y,output+28)==0xc3300000 && word(y,output+44)==0xc3480000);++transposedInverses;
    }
    {auto &r=b.runtime();constexpr uint32_t matrix=0x180000,stack=0x1fff000;R5900Context c{};c.r[5].ud[0]=matrix;c.r[29].ud[0]=stack;c.r[3].ud[0]=stack;
        for(unsigned i=0;i<16;++i)put(r,matrix+i*4,i%5==0?0x3f800000u:0u);for(unsigned i=0;i<4;++i)put(r,stack+i*4,0x3f800000u);
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x216e04);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[5].ud[0]=stack-16;reject();c.r[5].ud[0]=matrix;put(r,matrix,0x7fc00000u);reject();put(r,matrix,1u);reject();put(r,matrix,0x7f7fffffu);put(r,stack,0x40000000u);reject();
    }
    unsigned angularCosines=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1ffe000,child=seed&2?0x70000100:0x1b0000,parent=0x1b1000,group=0x1c0000,part=0x1d0000,pointRecord=0x1e0000;
        put(x,part+0x38,pointRecord);uint32_t matrix[16],points[2][4];
        for(auto &v:matrix){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((116+random%15)<<23);}
        for(auto &point:points)for(auto &v:point){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((116+random%15)<<23);}
        if(seed%16==0 || seed%16==8){std::memset(matrix,0,sizeof(matrix));for(unsigned i=0;i<3;++i)matrix[i*5]=0x3f800000;
            std::memset(points,0,sizeof(points));points[0][0]=points[1][0]=0x3f800000;
            if(seed%64==16)points[1][0]=0xbf800000;
            if(seed%64==32){points[1][0]=0;points[1][1]=0x3f800000;}
            if(seed%64==48)points[0][0]=0;
            if(seed%16==8)points[0][0]=points[1][0]=0x17800000;
        }
        for(unsigned i=0;i<4;++i)matrix[i*4+3]=0xff800011u+i;points[0][3]=0xff801234;points[1][3]=0x7fc05678;
        std::memcpy(x.memory.hostPointer(stack+0x150,64),matrix,64);std::memcpy(x.memory.hostPointer(stack+0x190,16),points[0],16);std::memcpy(x.memory.hostPointer(pointRecord+0x30,16),points[1],16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x43,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[16].ud[0]=child;p.r[17].ud[0]=parent;p.r[19].ud[0]=group;p.r[21].ud[0]=part;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fccf8)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fccf8)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        const auto cosine=word(y,stack+0x140);if(seed%64==0)check(cosine==0x3f800000);if(seed%64==16)check(cosine==0xbf800000);
        if(seed%64==32 || seed%64==48)check((cosine&0x7fffffff)==0);if(seed%16==8)check(cosine>0x3f800000 && cosine<0x7f800000);
        for(unsigned i=1;i<4;++i)check(word(y,stack+0x140+i*4)==0x61616161);++angularCosines;
    }
    {auto &r=b.runtime();constexpr uint32_t stack=0x1ffe000,part=0x1d0000,point=0x1e0000;R5900Context c{};
        c.r[16].ud[0]=0x1b0000;c.r[17].ud[0]=0x1b1000;c.r[19].ud[0]=0x1c0000;c.r[21].ud[0]=part;c.r[29].ud[0]=stack;c.r[3].ud[0]=stack+0x140;c.r[4].ud[0]=stack+0x190;c.r[5].ud[0]=stack+0x150;
        put(r,part+0x38,point);for(unsigned i=0;i<16;++i)put(r,stack+0x150+i*4,i%5==0?0x3f800000u:0u);
        for(unsigned i=0;i<4;++i){put(r,stack+0x190+i*4,0x3f800000u);put(r,point+0x30+i*4,0x3f800000u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fcdac);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[4].ud[0]+=16;reject();c.r[4].ud[0]-=16;put(r,part+0x38,stack+0x100);reject();put(r,part+0x38,point);
        put(r,stack+0x150,0x7fc00000u);reject();put(r,stack+0x150,0x3f800000u);put(r,point+0x30,1u);reject();put(r,point+0x30,0x7f7fffffu);reject();
        for(unsigned i=0;i<3;++i){put(r,stack+0x190+i*4,0x1f800000u);put(r,point+0x30+i*4,0x1f800000u);}reject();
    }
    std::printf("%u bone,%u distance,%u scale,%u parameters,%u selectors (%u rigid/%u general),%u column inverses,%u paired aliases,%u paired callers,%u full/%u linear transforms,%u motion callers,%u position pairs (%u fixed),%u boxes (%u clamped/%u unchanged),%u transposed inverses,%u angular cosines: CPU/RAM, four modes, live XYZW; no VU state\n",cases,distances,scales,parameters,selectors,rigid,general,columns,pairs,pairedCallers,transforms,linearTransforms,motions,positionPairs,fixedPairs,boxes,clamped,unclamped,transposedInverses,angularCosines);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}
