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
        if(const char *v=std::getenv("VP2S_CHEAT_EXP"))cheats.expMultiplier=std::atoi(v);
        if(const char *v=std::getenv("VP2S_CHEAT_GIVE")){char *end=nullptr;cheats.giveItem=int(std::strtol(v,&end,0));
            cheats.giveCount=end && *end==':'?int(std::strtol(end+1,nullptr,0)):99;}
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
// Items: a requested count written into the item's inventory slot (0 empties it), at most the item's own stack
// limit; then the counts read back for the panel. The slots are checked to be the identity layout first.
static void applyItems(Runtime *rt){
    auto &cheats=nativeCheats();uint32_t inventory,table,first,second;
    if(!readWord(rt,party::kInventory,inventory) || !readWord(rt,party::kItemTable,table) || !readWord(rt,inventory,first) ||
       !readWord(rt,inventory+4,second) || (first&0xffff)!=0 || (second&0xffff)!=1){cheats.inventoryKnown=false;return;}
    const int id=cheats.giveItem.exchange(-1);
    if(id>=0 && unsigned(id)<NativeCheats::kItems){
        const auto *limit=rt->memory.hostPointer(table+uint32_t(id)*0x38+0x30,1);
        const int count=std::clamp(cheats.giveCount.load(),0,limit && *limit?int(*limit):99);
        writeWord(rt,inventory+4u*uint32_t(id),count?(1u<<24)|(uint32_t(count)<<16)|uint32_t(id):uint32_t(id));}
    for(unsigned i=0;i<NativeCheats::kItems;++i){uint32_t slot=0;readWord(rt,inventory+4*i,slot);cheats.counts[i]=(slot&0xffff)==i?uint8_t(slot>>16):0;}
    cheats.inventoryKnown=true;
}
// EXP: the persistent party records ([1d3694] + i*0x140, party kind, valid flags checksum +14 = +1c ^ (+04 ^ (+62 +
// +30))). A rise since the last frame is multiplied; +04 and +14 are rewritten as the game rewrites them.
static void applyExp(Runtime *rt){
    static std::array<int64_t,32> last{};static bool primed=false;
    auto &cheats=nativeCheats();uint32_t first;if(!readWord(rt,0x1d3694,first)){primed=false;return;}
    const int multiplier=std::clamp(cheats.expMultiplier.load(),1,100);
    for(unsigned i=0;i<last.size();++i){
        const uint32_t record=first+i*0x140;uint32_t exp,check,salt,extra;
        const auto *kind=rt->memory.hostPointer(record+0x5a,1);const auto *flags=rt->memory.hostPointer(record+0x62,2);
        if(!kind || !flags || ((*kind^party::kKindKey)&3)!=1 || !readWord(rt,record+4,exp) || !readWord(rt,record+0x14,check) ||
           !readWord(rt,record+0x1c,salt) || !readWord(rt,record+0x30,extra)){last[i]=-1;continue;}
        uint16_t f;std::memcpy(&f,flags,2);
        const auto flagsCheck=[&](uint32_t e){return salt^(e^(uint32_t(f)+extra));};
        if(flagsCheck(exp)!=check){last[i]=-1;continue;}
        int64_t value=exp^party::kExpKey;
        if(primed && last[i]>=0 && value>last[i] && multiplier>1){
            value=std::min<int64_t>(last[i]+(value-last[i])*multiplier,99999999);
            const uint32_t encoded=uint32_t(value)^party::kExpKey;writeWord(rt,record+4,encoded);writeWord(rt,record+0x14,flagsCheck(encoded));}
        last[i]=value;
    }
    primed=true;
}
void applyNativeCheats(Runtime *rt){
    if(!rt)return;
    applyMoney(rt);applyItems(rt);applyExp(rt);
    if(!nativeCheats().infiniteHealth)return;
    // Battle: the party's battle records, [*1d366c + cbcc] (4 slots, -1 or empty outside battle).
    uint32_t battle;
    if(readWord(rt,0x1d366c,battle))for(unsigned i=0;i<4;++i){uint32_t record;if(readWord(rt,battle+0xcbcc+4*i,record) && record>=0x100000 && record<0x2000000)restoreHealth(rt,record);}
}
}
