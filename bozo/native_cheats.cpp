#include "native_cheats.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
namespace vp2s {
NativeCheats &nativeCheats(){
    static NativeCheats cheats;
    static const bool fromEnvironment=[]{
        // Headless tests: VP2S_CHEAT_INFINITE_HEALTH=1.
        if(const char *v=std::getenv("VP2S_CHEAT_INFINITE_HEALTH"))cheats.infiniteHealth=v[0]=='1';
        if(const char *v=std::getenv("VP2S_CHEAT_MONEY"))cheats.setMoney=std::strtoll(v,nullptr,10);
        return true;}();
    (void)fromEnvironment;return cheats;
}
namespace {
bool readWord(Runtime *rt,uint32_t at,uint32_t &v){const auto *p=rt->memory.hostPointer(at,4);if(!p || at<0x100000 || at>0x1fffffc)return false;std::memcpy(&v,p,4);return true;}
void writeWord(Runtime *rt,uint32_t at,uint32_t v){if(auto *p=rt->memory.hostPointer(at,4))std::memcpy(p,&v,4);}
// HP checksum +18 = +1c ^ ((+10 + 0x1d9) ^ (+08 + +0c)): what the game's HP setter writes after +10.
uint32_t hpChecksum(uint32_t base,uint32_t maxHp,uint32_t hp,uint32_t salt){return salt^((hp+0x1d9)^(base+maxHp));}
// A live party member's record: party kind, valid HP checksum, not fallen (flag 0x20). Its HP becomes its
// maximum, and the checksum is rewritten as the game rewrites it, so the anti-cheat checks still pass.
void restoreHealth(Runtime *rt,uint32_t record){
    uint32_t base,maxHp,hp,check,salt;const auto *kind=rt->memory.hostPointer(record+0x5a,1);const auto *flags=rt->memory.hostPointer(record+0x62,2);
    if(!kind || !flags || ((*kind^party::kKindKey)&3)!=1)return;
    if(!readWord(rt,record+0x8,base) || !readWord(rt,record+0xc,maxHp) || !readWord(rt,record+0x10,hp) || !readWord(rt,record+0x18,check) || !readWord(rt,record+0x1c,salt))return;
    if(hpChecksum(base,maxHp,hp,salt)!=check)return;   // not a consistent record: leave it to the game
    uint16_t f;std::memcpy(&f,flags,2);if((f^party::kFlagsKey)&0x20)return;   // fallen: no revival under the battle's feet
    const uint32_t full=(maxHp^party::kMaxHpKey)^party::kHpKey;
    if(full==hp)return;
    writeWord(rt,record+0x10,full);writeWord(rt,record+0x18,hpChecksum(base,maxHp,full,salt));
}
}
// The party's general record (the game-data manager's [1d3690], 0x130 bytes): money at +6c (XOR kMoneyKey, at
// most 99 999 999) under the record's hash at +18, seeded by +04 over bytes +24..+d5 (mod003 3d7a40, also its
// check at 3d7a94): h = (byte << 8) + ((h * 0x343fd + 0x269ec3) >> 1).
static uint32_t generalHash(Runtime *rt,uint32_t record){
    uint32_t h;if(!readWord(rt,record+4,h))return 0;const auto *bytes=rt->memory.hostPointer(record+0x24,0xd6-0x24);if(!bytes)return 0;
    for(unsigned i=0;i<0xd6-0x24;++i)h=(uint32_t(bytes[i])<<8)+((h*0x343fdu+0x269ec3u)>>1);
    return h;
}
static void applyMoney(Runtime *rt){
    auto &cheats=nativeCheats();uint32_t record,value,hash;
    if(!readWord(rt,0x1d3690,record) || !readWord(rt,record+0x6c,value) || !readWord(rt,record+0x18,hash) || generalHash(rt,record)!=hash){cheats.money=-1;return;}
    const auto wanted=cheats.setMoney.exchange(-1);
    if(wanted>=0){value=uint32_t(std::min<int64_t>(wanted,party::kMoneyMax))^party::kMoneyKey;writeWord(rt,record+0x6c,value);writeWord(rt,record+0x18,generalHash(rt,record));}
    cheats.money=value^party::kMoneyKey;
}
void applyNativeCheats(Runtime *rt){
    if(!rt)return;
    applyMoney(rt);
    if(!nativeCheats().infiniteHealth)return;
    // Battle: the party's battle records, [*1d366c + cbcc] (4 slots, -1 or empty outside battle).
    uint32_t battle;
    if(readWord(rt,0x1d366c,battle))for(unsigned i=0;i<4;++i){uint32_t record;if(readWord(rt,battle+0xcbcc+4*i,record) && record>=0x100000 && record<0x2000000)restoreHealth(rt,record);}
}
}
