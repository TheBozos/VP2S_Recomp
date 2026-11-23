#pragma once
#include <cstddef>
#include <cstdint>
namespace vp2s {
// dynamic_cast<To *>(from), cached per dynamic type. The native site handlers find their owners with
// cross-casts on every call (the dispatcher from rt->services, an owner interface from its host), and
// __dynamic_cast walked the class graph each time. A cast's result is a fixed offset (or null) for a given
// dynamic type, which the object's vtable pointer names, so the last type seen and its offset are kept per
// (To, From) pair and thread. Same results as dynamic_cast for every object, including nullptr.
template<class To,class From>To *nativeCast(From *from){
    if(!from)return nullptr;
    thread_local const void *lastType=nullptr;thread_local std::ptrdiff_t delta=0;thread_local bool found=false;
    const void *type=*reinterpret_cast<const void *const *>(from);
    if(type!=lastType){To *to=dynamic_cast<To *>(from);found=to!=nullptr;
        delta=found?reinterpret_cast<const char *>(to)-reinterpret_cast<const char *>(from):0;lastType=type;}
    return found?reinterpret_cast<To *>(reinterpret_cast<char *>(from)+delta):nullptr;
}
}
