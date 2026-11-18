#pragma once
// Save-state helpers for VP2S's native owners (BozoEngine bozo/state.h): optional values and
// shared immutable records, which the engine's archive does not cover. Every owner saves all of
// its fields, including records keyed by a guest stack: the state is taken at the dispatcher's
// safe point, where a guest call may be suspended between two of the owner's hooks, and the
// dispatcher restores the same stacks.
#include "bozo/state.h"
#include <memory>
#include <optional>
namespace vp2s::state {
using bozo::state::Archive;
template<class T>void value(Archive &a,T &v){using bozo::state::io;io(a,v);}
template<class T>void optional(Archive &a,std::optional<T> &v){
    bool has=v.has_value();bozo::state::io(a,has);
    if(a.loading()){if(has){T t{};value(a,t);v=std::move(t);}else v.reset();}
    else if(has)value(a,*v);
}
// A map whose values need their own code (each(value)); keys are saved as values.
template<class M,class F>void map(Archive &a,M &m,F each){
    auto n=bozo::state::count(a,m.size());
    if(a.loading()){m.clear();for(uint64_t i=0;i<n;++i){typename M::key_type k{};value(a,k);each(m[k]);}}
    else for(auto &[k,v]:m){auto copy=k;value(a,copy);each(v);}
}
// A shared record held by const pointer (published once, never changed): saved by value, so a
// load gives each holder its own copy; identity between holders is not kept.
template<class T>void shared(Archive &a,std::shared_ptr<const T> &p){
    bool has=p!=nullptr;bozo::state::io(a,has);
    if(a.loading()){if(has){auto t=std::make_shared<T>();value(a,*t);p=std::move(t);}else p.reset();}
    else if(has){T copy=*p;value(a,copy);}
}
}
