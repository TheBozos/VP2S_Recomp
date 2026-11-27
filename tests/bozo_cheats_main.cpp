// The cheats (bozo/native_cheats.cpp) against synthetic records built with the game's layouts: every value they
// write must keep the record's own anti-cheat checksum valid, and with every cheat off nothing changes.
#include "native_cheats.h"
#include "bozo/ee/dispatcher.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>
namespace {
void check(bool ok,const char *what){if(!ok)throw std::runtime_error(std::string("cheat assertion failed: ")+what);}
struct Host:bozo::ee::Host{
    void libraryCall(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("library");}
    void vu0Microprogram(bozo::ee::Dispatcher &,R5900Context *,uint32_t)override{throw std::runtime_error("VU");}
    void ioRead(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,uint64_t[2])override{throw std::runtime_error("device");}
    void ioWrite(bozo::ee::Dispatcher &,R5900Context *,uint32_t,uint32_t,const uint64_t[2])override{throw std::runtime_error("device");}
    void fault(bozo::ee::Dispatcher &,R5900Context *,const std::string &s)override{throw std::runtime_error(s);}
};
using namespace vp2s::party;
uint32_t word(Runtime &rt,uint32_t at){uint32_t v;std::memcpy(&v,rt.memory.hostPointer(at,4),4);return v;}
void put(Runtime &rt,uint32_t at,uint32_t v){std::memcpy(rt.memory.hostPointer(at,4),&v,4);}
uint32_t hpCheck(Runtime &rt,uint32_t r){return word(rt,r+0x1c)^((word(rt,r+0x10)+0x1d9)^(word(rt,r+8)+word(rt,r+0xc)));}
uint32_t flagsCheck(Runtime &rt,uint32_t r){uint16_t f;std::memcpy(&f,rt.memory.hostPointer(r+0x62,2),2);return word(rt,r+0x1c)^(word(rt,r+4)^(f+word(rt,r+0x30)));}
// A party member as the game keeps one: HP, max HP, EXP, flags, kind, both checksums.
void member(Runtime &rt,uint32_t r,uint32_t hp,uint32_t maxHp,uint32_t exp,uint16_t flags){
    put(rt,r+4,exp^kExpKey);put(rt,r+8,maxHp^kBaseMaxHpKey);put(rt,r+0xc,maxHp^kMaxHpKey);put(rt,r+0x10,hp^kHpKey);
    put(rt,r+0x1c,0x3ef25439);put(rt,r+0x30,0x21089ce9);*rt.memory.hostPointer(r+0x5a,1)=kKindKey^1;
    const uint16_t f=flags^kFlagsKey;std::memcpy(rt.memory.hostPointer(r+0x62,2),&f,2);
    put(rt,r+0x18,hpCheck(rt,r));put(rt,r+0x14,flagsCheck(rt,r));
}
uint32_t generalHash(Runtime &rt,uint32_t r){uint32_t h=word(rt,r+4);const auto *b=rt.memory.hostPointer(r+0x24,0xb2);for(unsigned i=0;i<0xb2;++i)h=(uint32_t(b[i])<<8)+((h*0x343fdu+0x269ec3u)>>1);return h;}
}
int main(){try{
    Host host;bozo::ee::Dispatcher d(host);auto &rt=d.runtime();auto &cheats=vp2s::nativeCheats();
    constexpr uint32_t battle=0x57f810,alicia=0x87c140,rufus=0x87c2a0,persistent=0x65d8c0,general=0x65d5e0,inventory=0x660660,table=0x314c00;
    put(rt,0x1d366c,battle);put(rt,battle+0xcbcc,alicia);put(rt,battle+0xcbd0,rufus);put(rt,battle+0xcbd4,0xffffffff);put(rt,battle+0xcbd8,0xffffffff);
    member(rt,alicia,63,100,0,0);member(rt,rufus,0,100,0,0x20);   // Rufus fallen
    put(rt,0x1d3694,persistent);member(rt,persistent,100,100,120,0);member(rt,persistent+0x140,100,100,80,0);
    put(rt,0x1d3690,general);put(rt,general+4,0x12345678);put(rt,general+0x6c,500^kMoneyKey);put(rt,general+0x18,generalHash(rt,general));
    // Combatants: the battle (class 16f7a0) at [*1d366c + cb88] (map-79 layout), enemy units from +44, each record at
    // +818; a live enemy and a fallen one.
    constexpr uint32_t combatants=0x5a0000,live=0x5a1000,fallen=0x5a2000,liveRecord=0x5a3000,fallenRecord=0x5a3200;
    put(rt,battle+0xcb88,combatants);put(rt,combatants,0x16f7a0u);put(rt,combatants+0x44,live);put(rt,combatants+0x48,fallen);
    put(rt,live+kUnitRecord,liveRecord);put(rt,fallen+kUnitRecord,fallenRecord);
    member(rt,liveRecord,600,816,0,0);member(rt,fallenRecord,0,816,0,0x20);
    for(const uint32_t r:{liveRecord,fallenRecord})*rt.memory.hostPointer(r+0x5a,1)=kKindKey^2;
    put(rt,kInventory,inventory);put(rt,kItemTable,table);for(uint32_t i=0;i<999;++i)put(rt,inventory+4*i,i);
    *rt.memory.hostPointer(table+0xe7*0x38+0x30,1)=99;*rt.memory.hostPointer(table+0x10*0x38+0x30,1)=1;
    const std::vector<uint8_t> before(rt.memory.flat(),rt.memory.flat()+0x2000000);
    // Every cheat off: nothing written, the money and counts read.
    vp2s::applyNativeCheats(&rt);
    check(!std::memcmp(before.data(),rt.memory.flat(),before.size()),"a cheat wrote with every cheat off");
    check(cheats.money==500 && cheats.inventoryKnown && cheats.counts[0xe7]==0,"reads with every cheat off");
    // Infinite health: Alicia back to 100 with a valid HP checksum; fallen Rufus untouched.
    cheats.infiniteHealth=true;vp2s::applyNativeCheats(&rt);cheats.infiniteHealth=false;
    check((word(rt,alicia+0x10)^kHpKey)==100 && hpCheck(rt,alicia)==word(rt,alicia+0x18),"Alicia healed under her checksum");
    check((word(rt,rufus+0x10)^kHpKey)==0,"a fallen member is not revived");
    // A battle started from a dungeon map: the scene's battle pointer and party slots 0x60 lower (+cb28 / +cb6c).
    {const std::vector<uint8_t> saved(rt.memory.flat(),rt.memory.flat()+0x2000000);
     for(uint32_t at=0xcb88;at<=0xcbd8;at+=4)put(rt,battle+at,0);
     put(rt,battle+0xcb28,combatants);put(rt,battle+0xcb6c,alicia);member(rt,alicia,40,100,0,0);
     cheats.infiniteHealth=true;vp2s::applyNativeCheats(&rt);cheats.infiniteHealth=false;
     check((word(rt,alicia+0x10)^kHpKey)==100 && hpCheck(rt,alicia)==word(rt,alicia+0x18),"healed with the dungeon layout");
     std::memcpy(rt.memory.flat(),saved.data(),saved.size());}
    // Weak enemies: the live enemy drops to 1 HP under its checksum; the fallen one stays at 0; party records untouched.
    cheats.weakEnemies=true;vp2s::applyNativeCheats(&rt);cheats.weakEnemies=false;
    check((word(rt,liveRecord+0x10)^kHpKey)==1 && hpCheck(rt,liveRecord)==word(rt,liveRecord+0x18),"a live enemy at 1 HP under its checksum");
    check((word(rt,fallenRecord+0x10)^kHpKey)==0,"a fallen enemy is left alone");
    check((word(rt,alicia+0x10)^kHpKey)==100,"party records are not enemies");
    // Money: 123456 under the general record's hash; above the cap clamps to 99 999 999.
    cheats.setMoney=123456;vp2s::applyNativeCheats(&rt);
    check((word(rt,general+0x6c)^kMoneyKey)==123456 && generalHash(rt,general)==word(rt,general+0x18) && cheats.money==123456,"money under its hash");
    cheats.setMoney=1000000000;vp2s::applyNativeCheats(&rt);check((word(rt,general+0x6c)^kMoneyKey)==99999999,"money cap");
    // Items: 50 Union Plumes (id e7), then removed; an item whose limit is 1 takes 1.
    cheats.giveCount=50;cheats.giveItem=0xe7;vp2s::applyNativeCheats(&rt);
    check(word(rt,inventory+4*0xe7)==((1u<<24)|(50u<<16)|0xe7u) && cheats.counts[0xe7]==50,"give 50");
    cheats.giveCount=0;cheats.giveItem=0xe7;vp2s::applyNativeCheats(&rt);check(word(rt,inventory+4*0xe7)==0xe7 && cheats.counts[0xe7]==0,"remove");
    cheats.giveCount=99;cheats.giveItem=0x10;vp2s::applyNativeCheats(&rt);check(((word(rt,inventory+0x40)>>16)&0xff)==1,"the item's stack limit");
    // EXP x3: a gain of 30 becomes 90 under the flags checksum; no gain, no write.
    cheats.expMultiplier=3;vp2s::applyNativeCheats(&rt);   // primes the watcher
    const uint32_t r=persistent;put(rt,r+4,150^kExpKey);put(rt,r+0x14,flagsCheck(rt,r));   // the game grants 30
    vp2s::applyNativeCheats(&rt);
    check((word(rt,r+4)^kExpKey)==210 && flagsCheck(rt,r)==word(rt,r+0x14),"EXP gain multiplied under the flags checksum");
    check((word(rt,r+0x144)^kExpKey)==80,"the other member's EXP untouched");
    vp2s::applyNativeCheats(&rt);check((word(rt,r+4)^kExpKey)==210,"no second multiplication");
    std::printf("cheats: health, weak enemies, money, items and EXP keep the game's checksums; nothing written with cheats off\n");
    return 0;
}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());return 1;}}
