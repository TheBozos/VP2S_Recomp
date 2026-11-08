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
    unsigned symmetricPairs=0,weightedPairs=0,boxPublications=0,angularQuaternions=0,angularCompositions=0,velocityPublications=0,collisionNormals=0,responsePairs=0,responseFirsts=0,responseSeconds=0,responseFallbacks=0,planeEvaluations=0,planePublications=0;
    bool reference=false;uint32_t selectedInverse=0,translationW=0;
    bool callbackActive=false;uint32_t callbackObject=0;uint8_t callbackFlags=0;unsigned callbacks=0;
    std::array<uint32_t,24> callbackData{};
    bozo::libs::Libraries &libraries()override{throw std::runtime_error("libraries");}
    bool nativeGraphics()const override{return true;}
    bool nativeInstruction(uint8_t *r,R5900Context *c,Runtime *rt,uint32_t pc)override{
        if(pc==0x1fbff0)++planeEvaluations;if(pc==0x1fc010)++planePublications;
        if(reference && (pc==0x1fbff0 || pc==0x1fc010)){
            float point[4],origin[4],normal[4],products[3];std::memcpy(point,rt->memory.hostPointer(c->r[5].uw[0],16),16);std::memcpy(origin,rt->memory.hostPointer(c->r[4].uw[0]+0x230,16),16);std::memcpy(normal,rt->memory.hostPointer(c->r[4].uw[0]+0x220,16),16);
            const auto reduce=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned i=0;i<3;++i){volatile float difference=point[i]-origin[i],product=reduce(difference)*normal[i];products[i]=reduce(product);}
            volatile float xy=products[0]+products[1],z=1.f*products[2],dot=reduce(xy)+reduce(z),difference=reduce(dot)-c->f[12];const auto distance=reduce(difference);
            if(pc==0x1fbff0)put(*rt,c->r[29].uw[0],std::bit_cast<uint32_t>(distance));
            else{for(unsigned i=0;i<3;++i){volatile float base=point[i]+0.f,product=normal[i]*distance,value=reduce(base)-reduce(product);point[i]=reduce(value);}std::memcpy(rt->memory.hostPointer(c->r[6].uw[0],16),point,16);}return true;
        }
        if(reference && pc>=0x211fb0 && pc<0x2120a0){
            float segment[2][4],point[4],delta[3],products[3],squares[3];std::memcpy(segment,rt->memory.hostPointer(c->r[4].uw[0],32),32);std::memcpy(point,rt->memory.hostPointer(c->r[5].uw[0],16),16);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned i=0;i<3;++i){volatile float d=point[i]-segment[0][i];delta[i]=normal(d);volatile float p=delta[i]*segment[1][i],s=segment[1][i]*segment[1][i];products[i]=normal(p);squares[i]=normal(s);}
            const auto total=[&](const float *p){volatile float xy=p[0]+p[1],z=p[2]*1.f,value=normal(xy)+normal(z);return normal(value);};
            const float projection=total(products),length=total(squares);float result;
            if(pc==0x211fe8)result=projection;
            else if(pc==0x212020)result=length;
            else if(pc==0x212064){volatile float ratio=projection/length,value=0.f+normal(ratio);result=normal(value);}
            else if(pc==0x212084){
                if(projection>0){if(projection>=length){for(unsigned i=0;i<3;++i){volatile float d=delta[i]-segment[1][i];delta[i]=normal(d);}}
                    else{volatile float ratio=projection/length;for(unsigned i=0;i<3;++i){volatile float base=delta[i]+0.f,p=segment[1][i]*normal(ratio),d=normal(base)-normal(p);delta[i]=normal(d);}}}
                for(unsigned i=0;i<3;++i){volatile float s=delta[i]*delta[i];squares[i]=normal(s);}result=total(squares);
            }else throw std::runtime_error("unexpected reference point/segment site");
            put(*rt,c->r[2].uw[0],std::bit_cast<uint32_t>(result));return true;
        }
        if(pc==0x1fc0c4)++responsePairs;if(pc==0x1fc128)++responseSeconds;if(pc==0x1fc174)++responseFirsts;if(pc==0x1fc108 || pc==0x1fc15c)++responseFallbacks;
        if(reference && (pc==0x1fc0c4 || pc==0x1fc128 || pc==0x1fc174)){
            const bool pair=pc==0x1fc0c4,secondOnly=pc==0x1fc128;const auto first=c->r[4].uw[0]+(pair?0:0x30),second=c->r[6].uw[0]+0x30;
            float n[4],a[4],b[4],coefficient;std::memcpy(n,rt->memory.hostPointer(c->r[29].uw[0]+0x210,16),16);std::memcpy(a,rt->memory.hostPointer(first,16),16);std::memcpy(b,rt->memory.hostPointer(second,16),16);std::memcpy(&coefficient,rt->memory.hostPointer(0x35910c,4),4);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            volatile float scaled=c->f[13]*coefficient;float correction[3];for(unsigned i=0;i<3;++i){volatile float d=n[i]*normal(scaled);correction[i]=normal(d);}
            float weights[2];if(pair){volatile float oneMinus=1.f-c->f[12];weights[0]=normal(oneMinus);weights[1]=c->f[12];}
            else if(secondOnly?c->f[12]<std::bit_cast<float>(0x3c23d70au):c->f[12]>std::bit_cast<float>(0x3f7d70a4u))std::memcpy(weights,rt->memory.hostPointer(0x359108,4),4);
            else{volatile float inverse=c->f[12]==0?std::copysign(std::numeric_limits<float>::max(),c->f[12]):1.f/c->f[12];weights[0]=normal(inverse);}
            float output[2][4];for(unsigned which=0;which<(pair?2u:1u);++which){const auto *input=pair?(which?b:a):secondOnly?b:a;std::memcpy(output[which],input,16);
                for(unsigned i=0;i<3;++i){volatile float base=pair?input[i]+0.f:input[i]*1.f,d=correction[i]*weights[which],value=normal(base)+normal(d);output[which][i]=normal(value);}}
            if(pair){std::memcpy(rt->memory.hostPointer(first,16),output[0],16);std::memcpy(rt->memory.hostPointer(second,16),output[1],16);}
            else std::memcpy(rt->memory.hostPointer(secondOnly?second:first,16),output[0],16);return true;
        }
        if(pc==0x1fc690)++collisionNormals;
        if(reference && pc==0x1fc690){
            float a[2][4],b[2][4],delta[3],squares[3],normal[3];std::memcpy(a,rt->memory.hostPointer(c->r[19].uw[0]+0x250,32),32);std::memcpy(b,rt->memory.hostPointer(c->r[20].uw[0]+0x230,32),32);
            const auto reduce=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned i=0;i<3;++i){volatile float start=b[0][i]+0.f,p=b[1][i]*c->f[1],end=reduce(start)+reduce(p),first=a[0][i]*1.f,d=reduce(end)-reduce(first),last=a[1][i]*c->f[2],difference=reduce(d)-reduce(last);
                delta[i]=reduce(difference);volatile float square=delta[i]*delta[i];squares[i]=reduce(square);}
            volatile float xy=squares[0]+squares[1],z=1.f*squares[2],squared=reduce(xy)+reduce(z),length=static_cast<float>(std::sqrt(std::abs(double(reduce(squared)))));
            volatile float reciprocal=length==0?std::numeric_limits<float>::max():static_cast<float>(1.0/double(length));
            for(unsigned i=0;i<3;++i){volatile float n=delta[i]*reduce(reciprocal);normal[i]=reduce(n);}
            std::memcpy(rt->memory.hostPointer(c->r[18].uw[0],12),normal,12);return true;
        }
        if(reference && pc>=0x2120a0 && pc<0x212950){
            float first[2][4],second[2][4];std::memcpy(first,rt->memory.hostPointer(c->r[4].uw[0],32),32);std::memcpy(second,rt->memory.hostPointer(c->r[5].uw[0],32),32);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            float delta[3],negative[3];for(unsigned i=0;i<3;++i){volatile float d=first[0][i]-second[0][i],n=second[1][i]*-1.f;delta[i]=normal(d);negative[i]=normal(n);}
            const auto dot=[&](const float *a,const float *b){float product[3];for(unsigned i=0;i<3;++i){volatile float p=a[i]*b[i];product[i]=normal(p);}
                volatile float xy=product[0]+product[1],z=1.f*product[2],xyz=normal(xy)+normal(z);return normal(xyz);};
            const float a=dot(first[1],first[1]),cross=dot(first[1],second[1]),cc=dot(second[1],second[1]),d=dot(delta,first[1]),f=dot(delta,delta),e=dot(delta,negative);
            volatile float negativeCross=cross*-1.f,ac=a*cc,bb=normal(negativeCross)*normal(negativeCross),det=normal(ac)-normal(bb);
            volatile float bd=normal(negativeCross)*d,ae=a*e,be=normal(negativeCross)*e,cd=cc*d,tn=normal(bd)-normal(ae),sn=normal(be)-normal(cd);
            if(pc==0x21211c){std::memcpy(rt->memory.hostPointer(c->r[29].uw[0],12),delta,12);return true;}
            float value;switch(pc){case 0x212120:value=a;break;case 0x212128:value=normal(negativeCross);break;case 0x212130:value=cc;break;
            case 0x21213c:value=d;break;case 0x212144:value=f;break;case 0x21214c:value=std::abs(normal(det));break;
            case 0x212188:value=e;break;case 0x212190:value=normal(tn);break;case 0x212198:value=normal(sn);break;default:throw std::runtime_error("unexpected reference segment site");}
            c->r[pc==0x21214c?3:2].uw[0]=std::bit_cast<uint32_t>(value);return true;
        }
        if(reference && pc==0x1fde6c){
            const auto part=c->r[17].uw[0],current=c->r[18].uw[0],stack=c->r[29].uw[0];float first[4],second[4];
            std::memcpy(first,rt->memory.hostPointer(word(*rt,part+0x38)+0x30,16),16);std::memcpy(second,rt->memory.hostPointer(current+0x30,16),16);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            if(*rt->memory.hostPointer(part+0x20,1)&4){
                float basis[4][4]{},local[4];uint32_t offset[4];std::memcpy(basis,rt->memory.hostPointer(word(*rt,current+4)+0x150,48),48);std::memcpy(local,rt->memory.hostPointer(part,16),16);
                // Opaque W is carried separately and never enters geometry.
                for(auto &row:basis)row[3]=0;referenceTransform(basis,local,3,offset);
                for(unsigned i=0;i<3;++i){volatile float a=first[i]+std::bit_cast<float>(offset[i]),b=second[i]+std::bit_cast<float>(offset[i]);first[i]=normal(a);second[i]=normal(b);}
            }
            for(unsigned i=0;i<3;++i){volatile float difference=first[i]-second[i];first[i]=normal(difference);}
            std::memcpy(rt->memory.hostPointer(stack+0x460,16),second,16);std::memcpy(rt->memory.hostPointer(stack+0x220,16),first,16);return true;
        }
        if(pc==0x1fe934)++velocityPublications;
        if(reference && pc==0x1fe934){
            const auto object=c->r[5].uw[0];float current[4],previous[4],contact;uint32_t out[4];
            std::memcpy(current,rt->memory.hostPointer(object+0x30,16),16);std::memcpy(previous,rt->memory.hostPointer(object+0x40,16),16);std::memcpy(out,current,16);std::memcpy(&contact,rt->memory.hostPointer(object+0x1c,4),4);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned i=0;i<3;++i){volatile float a=current[i]*c->f[20],b=previous[i]*c->f[20],difference=normal(a)-normal(b);float result=normal(difference);
                if(contact>0){volatile float scaled=result*c->f[0];result=normal(scaled);}out[i]=std::bit_cast<uint32_t>(result);}
            std::memcpy(rt->memory.hostPointer(object+0x50,16),out,16);return true;
        }
        if(pc==0x1fce30)++angularCompositions;
        if(reference && pc==0x1fce30){
            float a[4],b[4],products[4],out[4];std::memcpy(a,rt->memory.hostPointer(c->r[29].uw[0]+0x130,16),16);std::memcpy(b,rt->memory.hostPointer(c->r[17].uw[0]+0x30,16),16);
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            for(unsigned i=0;i<4;++i){volatile float product=a[i]*b[i];products[i]=normal(product);}
            const unsigned next[3]={1,2,0},last[3]={2,0,1};
            for(unsigned i=0;i<3;++i){volatile float positive=a[next[i]]*b[last[i]],negative=b[next[i]]*a[last[i]];
                volatile float cross=normal(positive)-normal(negative);volatile float first=b[i]*a[3],second=a[i]*b[3],base=normal(first)+normal(second),one=normal(cross)*1.f;
                volatile float result=normal(base)+normal(one);out[i]=normal(result);}
            volatile float wz=products[3]-products[2],wy=normal(wz)-products[1],wx=normal(wy)-products[0];out[3]=normal(wx);
            std::memcpy(rt->memory.hostPointer(c->r[29].uw[0]+0x130,16),out,16);return true;
        }
        if(pc==0x1fcdf0)++angularQuaternions;
        if(reference && (pc==0x1fcdac || pc==0x1fcdf0)){
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
            volatile float xy=normal(x)+normal(y),xyz=normal(xy)+normal(z);const auto result=normal(xyz);
            if(pc==0x1fcdac){std::memcpy(rt->memory.hostPointer(stack+0x140,4),&result,4);return true;}
            volatile float twice=result*2.f,coefficient=2.f+normal(twice);volatile float root=static_cast<float>(std::sqrt(std::abs(double(normal(coefficient)))));
            volatile float reciprocal=root==0?std::numeric_limits<float>::max():1.f/root;
            float cross[3];volatile float positiveX=directions[1][1]*directions[0][2],negativeX=directions[0][1]*directions[1][2];
            volatile float positiveY=directions[1][2]*directions[0][0],negativeY=directions[0][2]*directions[1][0];
            volatile float positiveZ=directions[1][0]*directions[0][1],negativeZ=directions[0][0]*directions[1][1];
            volatile float crossX=normal(positiveX)-normal(negativeX),crossY=normal(positiveY)-normal(negativeY),crossZ=normal(positiveZ)-normal(negativeZ);
            cross[0]=normal(crossX);cross[1]=normal(crossY);cross[2]=normal(crossZ);float quaternion[4];
            for(unsigned i=0;i<3;++i){volatile float component=cross[i]*normal(reciprocal);quaternion[i]=normal(component);}volatile float w=root*0.5f;quaternion[3]=normal(w);
            std::memcpy(rt->memory.hostPointer(stack+0x130,16),quaternion,16);return true;
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
        if(pc==0x1fc984 || pc==0x1fc7f8)++symmetricPairs;if(pc==0x1fc9cc || pc==0x1fc844)++weightedPairs;
        if(reference && (pc==0x1fc908 || pc==0x1fc984 || pc==0x1fc9cc || pc==0x1fc79c || pc==0x1fc7f8 || pc==0x1fc844)){
            const bool isDistance=pc==0x1fc908 || pc==0x1fc79c,symmetric=pc==0x1fc984 || pc==0x1fc7f8;
            float a[4],b[4],delta[3];const auto first=isDistance?c->r[4].uw[0]+0x30:c->r[5].uw[0],second=isDistance?c->r[6].uw[0]+0x30:c->r[4].uw[0];
            std::memcpy(a,rt->memory.hostPointer(first,16),16);std::memcpy(b,rt->memory.hostPointer(second,16),16);const float distance=referencePairDistance(a,b,delta);
            if(pc==0x1fc908 || pc==0x1fc79c){std::memcpy(rt->memory.hostPointer(c->r[29].uw[0],4),&distance,4);return true;}
            const auto normal=[](float v){const auto bits=std::bit_cast<uint32_t>(v);return std::bit_cast<float>((bits&0x7f800000)?bits:(bits&0x80000000));};
            volatile float reciprocal=distance==0?std::numeric_limits<float>::max():1.f/distance;
            uint32_t out[2][4];std::memcpy(out[0],a,16);std::memcpy(out[1],b,16);
            for(unsigned i=0;i<3;++i){volatile float direction=delta[i]*normal(reciprocal);volatile float firstDelta=normal(direction)*(symmetric?c->f[0]:c->f[1]);
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
    for(uint32_t entry:{0x1fc860u,0x1fc6d0u})for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
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
        if(entry==0x1fc6d0){const float gain=seed%16==0?1.f:seed%5==0?0.f:seed%5==1?.25f:seed%5==2?.5f:seed%5==3?1.5f:-.25f;
            // This field can alias a point's otherwise inactive W. It is an
            // active gain in this caller and must then contain a valid float.
            put(x,constraint+0x2c,std::bit_cast<uint32_t>(gain));}
        const bool fixed=*x.memory.hostPointer(first,1)==1 && *x.memory.hostPointer(second,1)==1;
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x37,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=first;p.r[5].ud[0]=constraint;p.r[6].ud[0]=second;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;p.f[12]=seed%16==0?1.f:seed&16?0.5f:seed&8?0.f:1.25f;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(entry)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(entry)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
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
    unsigned angularCosines=0,angularRotations=0,angularAligned=0;h.angularQuaternions=h.angularCompositions=0;
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
            if(seed%16==8 && (seed&16))points[1][0]|=0x80000000;
        }
        for(unsigned i=0;i<4;++i)matrix[i*4+3]=0xff800011u+i;points[0][3]=0xff801234;points[1][3]=0x7fc05678;
        uint32_t joint[4];for(auto &v:joint){random=random*1664525u+1013904223u;v=(random&0x807fffff)|((104+random%25)<<23);}
        if(seed%16==0){joint[0]=joint[1]=joint[2]=0;joint[3]=0x3f800000;}std::memcpy(x.memory.hostPointer(parent+0x30,16),joint,16);
        std::memcpy(x.memory.hostPointer(stack+0x150,64),matrix,64);std::memcpy(x.memory.hostPointer(stack+0x190,16),points[0],16);std::memcpy(x.memory.hostPointer(pointRecord+0x30,16),points[1],16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x43,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[16].ud[0]=child;p.r[17].ud[0]=parent;p.r[19].ud[0]=group;p.r[21].ud[0]=part;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fccf8)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fccf8)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        const auto cosine=word(y,stack+0x140);if(seed%64==0)check(cosine==0x3f800000);if(seed%64==16)check(cosine==0xbf800000);
        if(seed%64==32 || seed%64==48)check((cosine&0x7fffffff)==0);if(seed%16==8)check((cosine&0x7fffffff)>0x3f800000 && (cosine&0x7fffffff)<0x7f800000);
        if(q.r[2].uw[0]==stack+0x130){++angularRotations;check(std::bit_cast<float>(cosine)<std::bit_cast<float>(0x3f7ffffeu));}
        else {++angularAligned;check(!(std::bit_cast<float>(cosine)<std::bit_cast<float>(0x3f7ffffeu)));}
        if(seed%64==16)for(unsigned i=0;i<4;++i)check((word(y,stack+0x130+i*4)&0x7fffffff)==0);
        if(seed%64==32)check((word(y,stack+0x130)&0x7fffffff)==0 && (word(y,stack+0x134)&0x7fffffff)==0 &&
            (word(y,stack+0x138)&0x80000000) && word(y,stack+0x13c)>0 && word(y,stack+0x13c)<0x7f800000);
        for(unsigned i=1;i<4;++i)check(word(y,stack+0x140+i*4)==0x61616161);++angularCosines;
    }
    check(angularRotations && angularAligned && h.angularQuaternions==angularRotations*2 && h.angularCompositions==angularRotations*2);
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
        for(unsigned i=0;i<3;++i){put(r,stack+0x190+i*4,i==0?0x3f800000u:0u);put(r,point+0x30+i*4,i==1?0x3f800000u:0u);}
        check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fcdac));c.r[2].ud[0]=stack+0x130;c.f[0]=std::bit_cast<float>(word(r,stack+0x140));c.f[1]=std::bit_cast<float>(0x3f7ffffeu);
        auto rejectQuaternion=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fcdf0);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[2].ud[0]+=16;rejectQuaternion();c.r[2].ud[0]-=16;c.f[1]=1.f;rejectQuaternion();c.f[1]=std::bit_cast<float>(0x3f7ffffeu);c.f[0]=1.f;rejectQuaternion();c.f[0]=std::bit_cast<float>(word(r,stack+0x140));
        put(r,point+0x30,0xbf800000u);put(r,point+0x34,0u);rejectQuaternion();put(r,point+0x30,0u);put(r,point+0x34,0x3f800000u);
        put(r,stack+0x190,0x1fb00000u);put(r,point+0x34,0x1fb00000u);check(vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fcdac));c.f[0]=std::bit_cast<float>(word(r,stack+0x140));rejectQuaternion();
    }
    unsigned quaternionProducts=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t first=seed&1?0x70000300:0x180000,second=seed%4==0?0x190000:seed%4==1?first:seed%4==2?(seed&1?first:first|0x20000000u):first+16;
        uint32_t aa[4],bb[4];for(auto *vector:{aa,bb})for(unsigned i=0;i<4;++i){random=random*1664525u+1013904223u;vector[i]=(random&0x807fffff)|((104+random%26)<<23);}
        if(seed%16==0)for(unsigned i=0;i<4;++i){aa[i]=std::bit_cast<uint32_t>(float(i+1));bb[i]=std::bit_cast<uint32_t>(float(i+5));}
        if(seed%16==8)for(unsigned i=0;i<4;++i){aa[i]=0x00800000;bb[i]=0x3f000000;}
        if(seed%16==4)for(unsigned i=0;i<4;++i){aa[i]=i&1?0x80000000u:0u;bb[i]=i&1?0u:0x80000000u;}
        std::memcpy(x.memory.hostPointer(first,16),aa,16);std::memcpy(x.memory.hostPointer(second,16),bb,16);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x45,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[2].ud[0]=first;p.r[3].ud[0]=second;p.r[17].ud[0]=second-0x30;p.r[29].ud[0]=first-0x130;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;check(h.nativeInstruction(x.memory.flat(),&p,&x,0x1fce30));const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;check(h.nativeInstruction(y.memory.flat(),&q,&y,0x1fce30));check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        if(seed%16==0)check(word(y,first)==0x41c00000 && word(y,first+4)==0x42400000 && word(y,first+8)==0x42400000 && word(y,first+12)==0xc0c00000);++quaternionProducts;
    }
    {auto &r=b.runtime();constexpr uint32_t first=0x180000,second=0x190000;R5900Context c{};c.r[2].ud[0]=first;c.r[3].ud[0]=second;c.r[17].ud[0]=second-0x30;c.r[29].ud[0]=first-0x130;
        for(unsigned i=0;i<4;++i){put(r,first+i*4,0x3f800000u);put(r,second+i*4,0x3f800000u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fce30);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[2].ud[0]+=4;reject();c.r[2].ud[0]-=4;put(r,first+12,0x7fc00000u);reject();put(r,first+12,1u);reject();put(r,first+12,0x7f7fffffu);put(r,second+12,0x40000000u);reject();
    }
    unsigned velocityLoops=0,velocityRecords=0;h.velocityPublications=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);const unsigned count=seed%8;
        const uint32_t records=seed&1?0x70000100:0x180000;
        for(unsigned index=0;index<count;++index){const auto entry=records+index*0xa0;
            const auto object=index%3==0?entry+0x20:index%3==1?entry+0x30:0x200000+index*0x100;
            put(x,entry+4,object);const float contacts[]={-1.f,-0.f,0.f,.25f,.5f,1.f,1.5f,2.5f};put(x,entry+0x1c,std::bit_cast<uint32_t>(contacts[(seed+index)%8]));
            uint32_t current[4],previous[4],quaternion[4];for(auto *vector:{current,previous,quaternion})for(unsigned i=0;i<4;++i){random=random*1664525u+1013904223u;vector[i]=(random&0x807fffff)|((104+random%27)<<23);}
            if(seed%16==1 || seed%16==4 || seed%16==6)for(unsigned i=0;i<3;++i){current[i]=std::bit_cast<uint32_t>(float(i+5));previous[i]=std::bit_cast<uint32_t>(float(i+1));}
            if(seed%16==5)for(unsigned i=0;i<3;++i){current[i]=0x00800000;previous[i]=0;}
            current[3]=0x7f801234u+index;previous[3]=0xffc05678u+index;
            std::memcpy(x.memory.hostPointer(entry+0x30,16),current,16);std::memcpy(x.memory.hostPointer(entry+0x40,16),previous,16);std::memcpy(x.memory.hostPointer(object+0x30,16),quaternion,16);
        }
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[19].ud[0]=records;p.r[20].ud[0]=count;p.r[29].ud[0]=0x1fff000;p.r[31].ud[0]=0x170000;
        p.f[20]=float(seed%17+1)/8;if(seed&32)p.f[20]=-p.f[20];if(seed%16==1 || seed%16==4 || seed%16==6)p.f[20]=2;
        if(seed%16==3)p.f[20]=0;if(seed%16==5)p.f[20]=0.5f;if(!count)p.f[20]=std::bit_cast<float>(0x7f801234u);auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fe8a8)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fe8a8)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        for(unsigned i=0;i<count;++i){const auto entry=records+i*0xa0;check(word(y,entry+0x5c)==0x7f801234u+i);
            if(i%3==0)check(!std::memcmp(y.memory.hostPointer(entry+0x50,16),y.memory.hostPointer(entry+0x60,16),16));}
        if(count && (seed%16==1 || seed%16==4 || seed%16==6)){const uint32_t expected=seed%16==1?0x41000000:seed%16==4?0x40800000:0xc0800000;
            for(unsigned i=0;i<3;++i)check(word(y,records+0x50+i*4)==expected);}
        ++velocityLoops;velocityRecords+=count;
    }
    check(velocityRecords==1792 && h.velocityPublications==velocityRecords*2);
    {auto &r=b.runtime();constexpr uint32_t object=0x180000;R5900Context c{};c.r[5].ud[0]=object;c.r[3].ud[0]=object+0x50;c.r[4].ud[0]=object+0x30;c.f[20]=2;c.f[0]=0.5f;
        put(r,object+0x1c,0x3f000000u);for(unsigned i=0;i<4;++i){put(r,object+0x30+i*4,0x3f800000u);put(r,object+0x40+i*4,0u);}
        auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fe934);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[3].ud[0]+=4;reject();c.r[3].ud[0]-=4;put(r,object+0x1c,0x7fc00000u);reject();put(r,object+0x1c,0x3f000000u);put(r,object+0x48,1u);reject();put(r,object+0x48,0u);
        c.f[0]=std::bit_cast<float>(0x7fc00000u);reject();c.f[0]=0.5f;c.f[20]=std::numeric_limits<float>::max();put(r,object+0x30,0x40000000u);reject();
    }
    unsigned collisionQueries=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1fff000,origin=stack+0x460,direction=stack+0x220;
        // Keep the pointer field out of an active basis lane; point/output
        // aliases remain valid, but a pointer bit pattern is not a coefficient.
        const uint32_t part=seed%7==0 && seed%5!=0?stack+0x440:0x180000,current=(seed%4==0?origin:0x190030)-0x30;
        const uint32_t other=(seed%4==1?origin:seed%4==2?direction:0x1a0030)-0x30,basis=seed%5==0?origin:0x1b0150;
        uint32_t matrix[12],local[4],first[4],second[4];
        for(auto *value:{matrix,local,first,second}){const unsigned count=value==matrix?12:4;for(unsigned i=0;i<count;++i){random=random*1664525u+1013904223u;value[i]=(random&0x807fffff)|((108+random%22)<<23);}}
        for(unsigned i=0;i<3;++i)matrix[i*4+3]=0x7f801234u+i;local[3]=0xffc01234u;first[3]=0x7f805678u;second[3]=0xffc05678u;
        if(seed%16==0){for(unsigned i=0;i<12;++i)matrix[i]=i%5==0?0x3f800000:0;for(unsigned i=0;i<3;++i){local[i]=std::bit_cast<uint32_t>(float(i+1));first[i]=local[i];second[i]=std::bit_cast<uint32_t>(float(i+5));}}
        std::memcpy(x.memory.hostPointer(basis,48),matrix,48);std::memcpy(x.memory.hostPointer(part,16),local,16);
        put(x,part+0x38,other);put(x,current+4,basis-0x150);put(x,part+0x20,uint8_t(seed&2?4:0));
        std::memcpy(x.memory.hostPointer(other+0x30,16),first,16);std::memcpy(x.memory.hostPointer(current+0x30,16),second,16);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[17].ud[0]=part;p.r[18].ud[0]=current;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x170000;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fde10)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;try{y.functions.generated(0x1fde10)(y.memory.flat(),&q,&y);}catch(const std::exception &e){std::fprintf(stderr,"collision query mode=%u seed=%u: %s\n",mode,seed,e.what());throw;}check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        check(word(y,origin+12)==0xffc05678u && word(y,direction+12)==0x7f805678u);
        check(!std::memcmp(y.memory.hostPointer(direction,16),y.memory.hostPointer(origin+16,16),16));
        if(seed%16==0)for(unsigned i=0;i<3;++i)check(word(y,direction+i*4)==0xc0800000);++collisionQueries;
    }
    {auto &r=b.runtime();constexpr uint32_t part=0x180000,current=0x190000,other=0x1a0000,stack=0x1fff000;R5900Context c{};
        c.r[17].ud[0]=part;c.r[18].ud[0]=current;c.r[29].ud[0]=stack;c.r[2].ud[0]=stack+0x460;
        put(r,part+0x38,other);put(r,part+0x20,uint8_t(0));for(unsigned i=0;i<4;++i){put(r,current+0x30+i*4,0u);put(r,other+0x30+i*4,0u);}
        const auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fde6c);}catch(const std::runtime_error &){bad=true;}
            check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[2].ud[0]+=4;reject();c.r[2].ud[0]-=4;put(r,other+0x30,0x7fc00000u);reject();put(r,other+0x30,1u);reject();
        put(r,other+0x30,0x7f7fffffu);put(r,current+0x30,0xff7fffffu);reject();put(r,other+0x30,0u);put(r,current+0x30,0u);
        put(r,part+0x20,uint8_t(4));put(r,current+4,0x1b0000u);for(unsigned i=0;i<12;++i)put(r,0x1b0150+i*4,0u);for(unsigned i=0;i<3;++i)put(r,part+i*4,0u);
        put(r,0x1b0150,0x7fc00000u);reject();put(r,0x1b0150,0x7f7fffffu);put(r,part,0x40000000u);reject();
    }
    unsigned segmentDistances=0,segmentInterior=0,segmentClamped=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1fff000,first=0x180000,second=seed%13==0?first:0x190000;
        const uint32_t parameterA=seed%7==0?first+4:stack+0x20,parameterB=seed%11==0?parameterA:seed%5==0?second+0x18:stack+0x24;
        float values[2][2][4];for(auto &segment:values)for(auto &vector:segment)for(float &value:vector){random=random*1664525u+1013904223u;value=float(int(random%1025)-512)/16;}
        if(seed%8==0){const float known[2][2][4]={{{0,0,0,0},{1,0,0,0}},{{.5f,-1,2,0},{0,2,0,0}}};std::memcpy(values,known,sizeof(values));}
        if(seed%8==1){const float known[2][2][4]={{{0,0,0,0},{1,0,0,0}},{{3,2,0,0},{1,0,0,0}}};std::memcpy(values,known,sizeof(values));}
        if(seed%8==2){for(auto &segment:values)for(unsigned i=0;i<3;++i)segment[1][i]=0;}
        if(seed%8==3)for(unsigned i=0;i<3;++i)values[1][1][i]=-values[0][1][i];
        if(seed%16==4)for(auto &segment:values)for(auto &vector:segment)for(unsigned i=0;i<3;++i)vector[i]=std::bit_cast<float>(0x00800000u);
        for(unsigned s=0;s<2;++s)for(unsigned row=0;row<2;++row)values[s][row][3]=std::bit_cast<float>(0x7f801234u+s*8+row);
        std::memcpy(x.memory.hostPointer(first,32),values[0],32);std::memcpy(x.memory.hostPointer(second,32),values[1],32);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.pc=0x2120a0;p.r[4].ud[0]=first;p.r[5].ud[0]=second;p.r[6].ud[0]=parameterA;p.r[7].ud[0]=parameterB;p.r[29].ud[0]=stack;p.r[31].ud[0]=seed&1?0x1fc3dc:0x1fc5f4;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x2120a0)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x2120a0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==p.r[31].uw[0] && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));check(word(y,stack-4)==0x61616161);
        if(seed%13 && seed%8==0){check(q.f[0]==4.f);check(word(y,parameterA)==0x3f000000 && word(y,parameterB)==0x3f000000);++segmentInterior;}
        if(seed%13 && seed%8==1){check(q.f[0]==8.f);check(word(y,parameterB)==0);if(parameterA!=parameterB)check(word(y,parameterA)==0x3f800000);++segmentClamped;}++segmentDistances;
    }
    check(segmentInterior && segmentClamped);
    {auto &r=b.runtime();constexpr uint32_t first=0x180000,second=0x190000,stack=0x1fff000;R5900Context c{};c.pc=0x2120a0;c.r[31].ud[0]=0x1fc3dc;c.r[29].ud[0]=stack;c.r[2].ud[0]=stack;
        c.r[4].ud[0]=first;c.r[5].ud[0]=second;c.r[6].ud[0]=stack+0x20;c.r[7].ud[0]=stack+0x24;for(auto at:{first,second})for(unsigned i=0;i<8;++i)put(r,at+i*4,0u);
        const auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x21211c);}catch(const std::runtime_error &){bad=true;}check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[31].ud[0]=0;reject();c.r[31].ud[0]=0x1fc3dc;c.r[4].ud[0]=stack;reject();c.r[4].ud[0]=first;c.r[6].ud[0]=stack+4;reject();c.r[6].ud[0]=stack+0x20;
        put(r,first,0x7fc00000u);reject();put(r,first,1u);reject();put(r,first,std::bit_cast<uint32_t>(1.e9f));reject();
    }
    unsigned collisionCallers=0,collisionHits=0,collisionMisses=0,responseContinuations=0;h.collisionNormals=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1fff000,output=stack+0x210,parameter=stack+0x4bc,depth=stack+0x4b8;
        const uint32_t first=seed%5==0?output-0x250:0x180000,second=seed%7==0?output-0x230:seed%11==0?first:0x190000;
        float values[2][2][4];for(auto &segment:values)for(auto &vector:segment)for(float &value:vector){random=random*1664525u+1013904223u;value=float(int(random%1025)-512)/16;}
        if(seed%8==0){const float known[2][2][4]={{{0,0,0,0},{1,0,0,0}},{{.5f,-1,2,0},{0,2,0,0}}};std::memcpy(values,known,sizeof(values));}
        if(seed%8==1)for(auto &segment:values)for(unsigned i=0;i<3;++i)segment[1][i]=0;
        if(seed%8==2)for(unsigned i=0;i<3;++i)values[1][1][i]=-values[0][1][i];
        for(unsigned s=0;s<2;++s)for(unsigned row=0;row<2;++row)values[s][row][3]=std::bit_cast<float>(0x7f801234u+s*8+row);
        std::memcpy(x.memory.hostPointer(first+0x250,32),values[0],32);std::memcpy(x.memory.hostPointer(second+0x230,32),values[1],32);
        const float radiusA=seed%8==0?1.f:seed%3==0?0.f:seed%3==1?1.f:1000.f,radiusB=seed%8==0?2.f:seed%3==0?0.f:seed%3==1?1.f:1000.f;
        put(x,first+0x220,std::bit_cast<uint32_t>(radiusA));put(x,second+0x220,std::bit_cast<uint32_t>(radiusB));const auto outputW=word(x,output+12);
        constexpr uint32_t body=0x1d0000,part=0x1e0000,otherBody=0x1e1000;put(x,body,uint8_t(0));put(x,otherBody,uint8_t(0));put(x,part+0x38,otherBody);
        for(unsigned i=0;i<3;++i){put(x,body+0x30+i*4,std::bit_cast<uint32_t>(float(i+1)));put(x,otherBody+0x30+i*4,std::bit_cast<uint32_t>(-float(i+1)));}
        put(x,body+0x3c,0x7f801234u);put(x,otherBody+0x3c,0xffc01234u);put(x,0x359108,0x42c80000u);put(x,0x35910c,0x3f800000u);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}
        std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=first;p.r[5].ud[0]=second;p.r[6].ud[0]=output;p.r[7].ud[0]=parameter;p.r[8].ud[0]=depth;p.r[29].ud[0]=stack;p.r[31].ud[0]=seed&1?0x1fdf24:0x1fe000;p.r[18].ud[0]=body;p.r[17].ud[0]=part;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fc5b0)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fc5b0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==p.r[31].uw[0] && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));check(word(y,output+12)==outputW);
        if(q.r[2].uw[0]){++collisionHits;if(seed%8==0 && seed%5 && seed%7 && seed%11){check(word(y,output+8)==0x3f800000 && word(y,depth)==0x3f800000 && word(y,parameter)==0x3f000000);}}
        else ++collisionMisses;
        if(q.r[2].uw[0] && !(seed&1)){
            _mm_setcsr(control);h.reference=true;x.functions.generated(0x1fe008)(x.memory.flat(),&p,&x);const auto expected=_mm_getcsr();_mm_setcsr(control);
            h.reference=false;y.functions.generated(0x1fe008)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expected);_mm_setcsr(saved);
            p.cycleLimit=q.cycleLimit;check(q.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
            check(word(y,body+0x3c)==0x7f801234u && word(y,otherBody+0x3c)==0xffc01234u);++responseContinuations;
        }++collisionCallers;
    }
    check(collisionHits && collisionMisses && h.collisionNormals==2*collisionHits);
    {auto &r=b.runtime();constexpr uint32_t stack=0x1fff000,first=0x180000,second=0x190000;R5900Context c{};c.pc=0x1fc5f4;c.r[29].ud[0]=stack;c.r[19].ud[0]=first;c.r[20].ud[0]=second;
        c.r[18].ud[0]=stack+0x280;c.r[17].ud[0]=stack+0x52c;c.r[16].ud[0]=stack+0x528;c.r[3].ud[0]=second+0x230;c.r[2].ud[0]=second+0x240;c.r[5].ud[0]=first+0x250;c.r[4].ud[0]=first+0x260;
        put(r,stack+0x50,0x1fe000u);for(auto at:{first+0x250,second+0x230})for(unsigned i=0;i<8;++i)put(r,at+i*4,0u);c.f[1]=c.f[2]=.5f;
        const auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fc628);}catch(const std::runtime_error &){bad=true;}check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        put(r,stack+0x50,0u);reject();put(r,stack+0x50,0x1fe000u);c.r[18].ud[0]+=16;reject();c.r[18].ud[0]-=16;c.r[2].ud[0]+=4;reject();c.r[2].ud[0]-=4;
        put(r,first+0x250,0x7fc00000u);reject();put(r,first+0x250,1u);reject();put(r,first+0x250,0u);c.f[2]=3;reject();c.f[2]=.5f;c.r[19].ud[0]=stack+0x520-0x250;c.r[5].ud[0]=stack+0x520;c.r[4].ud[0]=stack+0x530;reject();
    }
    unsigned positionResponses=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1fff000,normal=stack+0x210,first=seed%5==0?normal-0x30:0x180000,second=seed%7==0?normal-0x30:seed%11==0?first:0x190000,part=0x1a0000;
        uint32_t n[4],a[4],b[4];for(auto *v:{n,a,b})for(unsigned i=0;i<3;++i){random=random*1664525u+1013904223u;v[i]=std::bit_cast<uint32_t>(float(int(random%33)-16)/16);}
        n[3]=0x7f801234;a[3]=0xffc01234;b[3]=0x7f805678;std::memcpy(x.memory.hostPointer(normal,16),n,16);std::memcpy(x.memory.hostPointer(first+0x30,16),a,16);std::memcpy(x.memory.hostPointer(second+0x30,16),b,16);
        put(x,first,uint8_t(seed%3));put(x,second,uint8_t((seed/3)%3));put(x,part+0x38,second);put(x,0x359100,0x7fc00000u);put(x,0x359104,0xff800000u);put(x,0x359108,0x42c80000u);put(x,0x35910c,std::bit_cast<uint32_t>(seed&8?-.5f:1.f));
        const float parameters[]={0.f,.005f,.01f,.5f,.99f,.995f,1.f,-.25f,1.25f,std::bit_cast<float>(0x3c23d709u),std::bit_cast<float>(0x3c23d70bu),std::bit_cast<float>(0x3f7d70a3u),std::bit_cast<float>(0x3f7d70a5u)};
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}p.f[12]=parameters[seed%13];p.f[13]=float(seed%5+1)/128;
        if(seed%16==0 && seed%5 && seed%7 && first!=second){put(x,first,uint8_t(0));put(x,second,uint8_t(0));for(unsigned i=0;i<3;++i){put(x,normal+i*4,i==2?0x3f800000u:0u);put(x,first+0x30+i*4,0u);put(x,second+0x30+i*4,i==2?0x41000000u:0u);}put(x,0x35910c,0x3f800000u);p.f[12]=.5f;p.f[13]=2;}
        const auto firstW=word(x,first+0x3c),secondW=word(x,second+0x3c);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[4].ud[0]=first;p.r[5].ud[0]=part;p.r[6].ud[0]=normal;p.r[29].ud[0]=stack;p.r[31].ud[0]=seed&1?0x1fdf68:0x1fe024;auto q=p;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fc030)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fc030)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(p.pc==p.r[31].uw[0] && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));check(word(y,first+0x3c)==firstW && word(y,second+0x3c)==secondW);
        if(seed%16==0 && seed%5 && seed%7 && first!=second)check(word(y,first+0x38)==0x3f800000 && word(y,second+0x38)==0x41100000);++positionResponses;
    }
    check(responseContinuations && h.responsePairs && h.responseFirsts && h.responseSeconds && h.responseFallbacks);
    {auto &x=a.runtime(),&y=b.runtime();constexpr uint32_t first=0x180000,second=0x190000,part=0x1a0000,stack=0x1fff000;
        std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);put(x,first,uint8_t(1));put(x,second,uint8_t(1));put(x,part+0x38,second);put(x,0x35910c,0x7f801234u);put(x,stack+0x210,0x7f801234u);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};p.r[4].ud[0]=first;p.r[5].ud[0]=part;p.r[6].ud[0]=stack+0x210;p.r[29].ud[0]=stack;p.r[31].ud[0]=0x1fe024;p.f[12]=p.f[13]=std::bit_cast<float>(0x7f801234u);auto q=p;
        h.reference=true;x.functions.generated(0x1fc030)(x.memory.flat(),&p,&x);h.reference=false;y.functions.generated(0x1fc030)(y.memory.flat(),&q,&y);
        p.cycleLimit=q.cycleLimit;check(!std::memcmp(&p,&q,sizeof(p)) && !std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
    }
    {auto &r=b.runtime();constexpr uint32_t first=0x180030,second=0x190030,stack=0x1fff000;R5900Context c{};c.pc=0x1fc030;c.r[31].ud[0]=0x1fe024;c.r[29].ud[0]=stack;c.r[4].ud[0]=first;c.r[6].ud[0]=second-0x30;c.r[3].ud[0]=second;c.f[12]=.5f;c.f[13]=.5f;
        for(auto at:{first,second,stack+0x210})for(unsigned i=0;i<4;++i)put(r,at+i*4,0u);put(r,0x35910c,0x3f800000u);
        const auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fc0c4);}catch(const std::runtime_error &){bad=true;}check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[31].ud[0]=0;reject();c.r[31].ud[0]=0x1fe024;c.r[3].ud[0]+=4;reject();c.r[3].ud[0]-=4;put(r,stack+0x210,0x7fc00000u);reject();put(r,stack+0x210,1u);reject();put(r,stack+0x210,0u);c.f[12]=3;reject();c.f[12]=.5f;put(r,0x35910c,0x7fc00000u);reject();
    }
    unsigned pointSegments=0,pointStarts=0,pointEnds=0,pointInteriors=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1fff000,segment=0x180000,point=seed%7==0?segment:seed%5==0?segment+16:0x190000,parameter=stack+0x40+(seed&4);
        float s[2][4],p[4];for(auto &v:s)for(float &value:v){random=random*1664525u+1013904223u;value=float(int(random%1025)-512)/16;}for(float &value:p){random=random*1664525u+1013904223u;value=float(int(random%1025)-512)/16;}
        if(seed%8<4){const float known[2][4]={{0,0,0,0},{4,0,0,0}};std::memcpy(s,known,sizeof(s));p[0]=seed%8==1?-2.f:seed%8==2?6.f:2.f;p[1]=3;p[2]=0;if(seed%8==3)s[1][0]=0;}
        if(seed%8==4){for(auto &v:s)for(unsigned i=0;i<3;++i)v[i]=std::bit_cast<float>(0x00800000u);for(unsigned i=0;i<3;++i)p[i]=std::bit_cast<float>(0x00800000u);}
        for(unsigned i=0;i<2;++i)s[i][3]=std::bit_cast<float>(0x7f801234u+i);p[3]=std::bit_cast<float>(0xffc05678u);
        std::memcpy(x.memory.hostPointer(segment,32),s,32);std::memcpy(x.memory.hostPointer(point,16),p,16);std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context u{};for(unsigned i=1;i<32;++i){u.r[i].ud[0]=seed*32+i;u.r[i].ud[1]=~u.r[i].ud[0];}std::memset(u.vu0_vf,0x47,sizeof(u.vu0_vf));u.vu0_status=0x1234;u.r[4].ud[0]=segment;u.r[5].ud[0]=point;u.r[6].ud[0]=parameter;u.r[29].ud[0]=stack;u.r[31].ud[0]=seed&1?0x1fc244:0x1fc4f0;auto v=u;
        const auto saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x211fb0)(x.memory.flat(),&u,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x211fb0)(y.memory.flat(),&v,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        u.cycleLimit=v.cycleLimit;check(u.pc==u.r[31].uw[0] && !std::memcmp(&u,&v,sizeof(u)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));
        for(unsigned offset:{4u,8u,12u,20u,24u,28u})check(word(y,stack-32+offset)==0x61616161);
        const float t=std::bit_cast<float>(word(y,parameter));if(t==0)++pointStarts;else if(t==1)++pointEnds;else ++pointInteriors;
        if(seed%8<4 && seed%7 && seed%5){check(v.f[0]==(seed%8==0?9.f:13.f));check(t==(seed%8==0?.5f:seed%8==2?1.f:0.f));}++pointSegments;
    }
    check(pointStarts && pointEnds && pointInteriors);
    {auto &r=b.runtime();constexpr uint32_t segment=0x180000,point=0x190000,stack=0x1fff000;R5900Context c{};c.pc=0x211fb0;c.r[31].ud[0]=0x1fc4f0;c.r[29].ud[0]=stack;c.r[4].ud[0]=segment;c.r[5].ud[0]=point;c.r[6].ud[0]=stack+0x40;c.r[2].ud[0]=stack+0x10;
        for(unsigned i=0;i<8;++i)put(r,segment+i*4,0u);for(unsigned i=0;i<4;++i)put(r,point+i*4,0u);
        const auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x211fe8);}catch(const std::runtime_error &){bad=true;}check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[31].ud[0]=0;reject();c.r[31].ud[0]=0x1fc4f0;c.r[6].ud[0]=segment+4;reject();c.r[6].ud[0]=stack+0x10;reject();c.r[6].ud[0]=stack+0x40;c.r[4].ud[0]=stack;reject();c.r[4].ud[0]=segment;
        put(r,point,0x7fc00000u);reject();put(r,point,1u);reject();put(r,point,std::bit_cast<uint32_t>(1.e9f));reject();put(r,point,0u);c.r[2].ud[0]+=4;reject();
    }
    unsigned planeCallers=0,planeHits=0,planeMisses=0,planeSkipped=0;h.planeEvaluations=h.planePublications=0;
    for(unsigned mode:{_MM_ROUND_NEAREST,_MM_ROUND_DOWN,_MM_ROUND_UP,_MM_ROUND_TOWARD_ZERO})for(unsigned seed=0;seed<128;++seed){
        auto &x=a.runtime(),&y=b.runtime();std::memset(x.memory.flat(),0x61,bozo::kFlatBytes);
        const uint32_t stack=seed&1?0x70003000:0x1fff000,owner=0x180000,point=seed%5==0?stack+0x1b0:seed%7==0?owner+0x200:0x190000,vtable=0x1c0000,group=0x1d0000,flags=0x1d1000;
        put(x,owner,vtable);put(x,vtable+0x8c,0x1fbfb0u);put(x,group+0x30,flags);std::memset(x.memory.hostPointer(flags,16),0,16);
        float normal[4],origin[4],position[4];for(auto *values:{normal,origin,position})for(unsigned i=0;i<3;++i){random=random*1664525u+1013904223u;values[i]=float(int(random%1025)-512)/16;}
        float margin=float(int(seed%33)-16)/8;
        if(seed%16<2){for(unsigned i=0;i<3;++i){normal[i]=i==2?1.f:0.f;origin[i]=0;position[i]=i==2?(seed%16==0?-2.f:2.f):0.f;}margin=.5f;}
        normal[3]=std::bit_cast<float>(0x7f801234u);origin[3]=std::bit_cast<float>(0xffc01234u);position[3]=std::bit_cast<float>(0x7f805678u);
        std::memcpy(x.memory.hostPointer(owner+0x220,16),normal,16);std::memcpy(x.memory.hostPointer(owner+0x230,16),origin,16);
        put(x,point,uint8_t(seed%9==0?1:0));put(x,point+2,uint8_t(seed%4==0?0xff:seed%8));put(x,point+8,seed%11==0?0u:owner);put(x,point+0x18,std::bit_cast<uint32_t>(margin));put(x,point+0x1c,0x3e800000u);put(x,owner+0x214,0x3f000000u);
        std::memcpy(x.memory.hostPointer(point+0x30,16),position,16);
        const bool skipped=seed%9==0 || seed%11==0;if(skipped)put(x,point+0x18,0x7f801234u);
        std::memcpy(y.memory.flat(),x.memory.flat(),bozo::kFlatBytes);
        R5900Context p{};for(unsigned i=1;i<32;++i){p.r[i].ud[0]=seed*32+i;p.r[i].ud[1]=~p.r[i].ud[0];}std::memset(p.vu0_vf,0x47,sizeof(p.vu0_vf));p.vu0_status=0x1234;p.r[17].ud[0]=point;p.r[21].ud[0]=group;p.r[29].ud[0]=stack;auto q=p;
        const auto beforeEvaluations=h.planeEvaluations,beforePublications=h.planePublications,saved=_mm_getcsr(),control=(saved&~_MM_ROUND_MASK)|mode;_mm_setcsr(control);
        h.reference=true;x.functions.generated(0x1fe1e0)(x.memory.flat(),&p,&x);const auto expectedFp=_mm_getcsr();_mm_setcsr(control);
        h.reference=false;y.functions.generated(0x1fe1e0)(y.memory.flat(),&q,&y);check(_mm_getcsr()==expectedFp);_mm_setcsr(saved);
        p.cycleLimit=q.cycleLimit;check(q.pc==0x170000 && !std::memcmp(&p,&q,sizeof(p)));check(!std::memcmp(x.memory.flat(),y.memory.flat(),bozo::kFlatBytes));check(word(y,point+0x3c)==0x7f805678u);
        if(skipped){check(h.planeEvaluations==beforeEvaluations && h.planePublications==beforePublications);++planeSkipped;}
        else{check(h.planeEvaluations==beforeEvaluations+2);if(h.planePublications==beforePublications+2){++planeHits;check(word(y,point+0x1c)==0x3f400000u);if(seed%4)check(*y.memory.hostPointer(flags+seed%8,1)==1);}
            else{check(h.planePublications==beforePublications);++planeMisses;check(word(y,point+0x1c)==0x3e800000u);}
            if(seed%16<2 && seed%7){check(word(y,point+0x38)==(seed%16==0?0x3f000000u:0x40000000u));}}
        ++planeCallers;
    }
    check(planeHits && planeMisses && planeSkipped);
    {auto &r=b.runtime();constexpr uint32_t owner=0x180000,point=0x190000,stack=0x1fff000;R5900Context c{};c.pc=0x1fbfb0;c.r[31].ud[0]=0x1fe210;c.r[29].ud[0]=stack;c.r[4].ud[0]=owner;c.r[5].ud[0]=point+0x30;c.r[6].ud[0]=stack+0x1f0;c.r[17].ud[0]=point;c.r[3].ud[0]=owner+0x230;c.r[2].ud[0]=stack;c.f[12]=.5f;
        put(r,point+8,owner);put(r,point+0x18,0x3f000000u);for(auto at:{owner+0x220,owner+0x230,point+0x30})for(unsigned i=0;i<4;++i)put(r,at+i*4,0u);
        const auto reject=[&]{const auto saved=c;std::vector<uint8_t> ram(r.memory.flat(),r.memory.flat()+bozo::kFlatBytes);bool bad=false;
            try{vp2s::nativeMathInstruction(r.memory.flat(),&c,&r,0x1fbff0);}catch(const std::runtime_error &){bad=true;}check(bad && !std::memcmp(&saved,&c,sizeof(c)) && !std::memcmp(ram.data(),r.memory.flat(),ram.size()));};
        c.r[31].ud[0]=0;reject();c.r[31].ud[0]=0x1fe210;c.r[6].ud[0]+=16;reject();c.r[6].ud[0]-=16;put(r,point+0x18,0u);reject();put(r,point+0x18,0x3f000000u);
        put(r,owner+0x220,0x7fc00000u);reject();put(r,owner+0x220,1u);reject();put(r,owner+0x220,std::bit_cast<uint32_t>(1.e9f));reject();put(r,owner+0x220,0u);
        c.r[17].ud[0]=stack-0x30;c.r[5].ud[0]=stack;put(r,stack-0x28,owner);put(r,stack-0x18,0x3f000000u);reject();
    }
    std::printf("%u bone,%u distance,%u scale,%u parameters,%u selectors (%u rigid/%u general),%u column inverses,%u paired aliases,%u paired callers,%u full/%u linear transforms,%u motion callers,%u position pairs (%u fixed),%u boxes (%u clamped/%u unchanged),%u transposed inverses,%u angular callers (%u rotation/%u aligned),%u quaternion products,%u velocity loops/%u records,%u collision queries,%u segment distances (%u interior/%u clamped samples),%u collision callers (%u hit/%u miss),%u responses/%u real continuations,%u point/segments (%u start/%u end/%u interior),%u plane callers (%u hit/%u miss/%u skipped): CPU/RAM, four modes, live XYZW; no VU state\n",cases,distances,scales,parameters,selectors,rigid,general,columns,pairs,pairedCallers,transforms,linearTransforms,motions,positionPairs,fixedPairs,boxes,clamped,unclamped,transposedInverses,angularCosines,angularRotations,angularAligned,quaternionProducts,velocityLoops,velocityRecords,collisionQueries,segmentDistances,segmentInterior,segmentClamped,collisionCallers,collisionHits,collisionMisses,positionResponses,responseContinuations,pointSegments,pointStarts,pointEnds,pointInteriors,planeCallers,planeHits,planeMisses,planeSkipped);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}
