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
        if(const char *v=std::getenv("VP2S_CHEAT_WEAK_ENEMIES"))cheats.weakEnemies=v[0]=='1';
        if(const char *v=std::getenv("VP2S_CHEAT_STRONG_PARTY"))cheats.strongParty=uint32_t(std::clamp(std::atol(v),0L,99999L));
        if(const char *v=std::getenv("VP2S_CHEAT_UNDYING"))cheats.undying=v[0]=='1';
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
// Undying (test sweeps): a party battle record with flag 0x0002, 0x0800 (incapacitating statuses: battles 0x63, 0xcb)
// or 0x0020 (fallen) loses them, under the flags checksum
// +14 = +1c ^ (+04 ^ (+62 + +30)) (the raw +62 halfword), and its HP is back at its maximum under +18.
void undie(Runtime *rt,uint32_t record){
    const auto *kind=rt->memory.hostPointer(record+0x5a,1);auto *flags=rt->memory.hostPointer(record+0x62,2);
    if(!kind || !flags || ((*kind^party::kKindKey)&3)!=1)return;
    uint32_t exp,check,salt,extra,base,maxHp,hp,hpCheck;
    if(!readWord(rt,record+4,exp) || !readWord(rt,record+0x14,check) || !readWord(rt,record+0x1c,salt) || !readWord(rt,record+0x30,extra) ||
       !readWord(rt,record+0x8,base) || !readWord(rt,record+0xc,maxHp) || !readWord(rt,record+0x10,hp) || !readWord(rt,record+0x18,hpCheck))return;
    uint16_t raw;std::memcpy(&raw,flags,2);
    if((salt^(exp^(uint32_t(raw)+extra)))!=check || hpChecksum(base,maxHp,hp,salt)!=hpCheck)return;   // not consistent: leave it
    const uint16_t f=raw^party::kFlagsKey;if(!(f&0x822))return;
    const uint16_t cleared=uint16_t((f&~0x822)^party::kFlagsKey);std::memcpy(flags,&cleared,2);
    writeWord(rt,record+0x14,salt^(exp^(uint32_t(cleared)+extra)));
    const uint32_t full=(maxHp^party::kMaxHpKey)^party::kHpKey;writeWord(rt,record+0x10,full);writeWord(rt,record+0x18,hpChecksum(base,maxHp,full,salt));
}
// Strong party (test sweeps): a live party member's battle record gets base and max HP `value` and full HP, the +18
// checksum rewritten over all three (it sums +08 and +0c).
void strengthen(Runtime *rt,uint32_t record,uint32_t value){
    uint32_t base,maxHp,hp,check,salt;const auto *kind=rt->memory.hostPointer(record+0x5a,1);const auto *flags=rt->memory.hostPointer(record+0x62,2);
    if(!kind || !flags || ((*kind^party::kKindKey)&3)!=1)return;
    if(!readWord(rt,record+0x8,base) || !readWord(rt,record+0xc,maxHp) || !readWord(rt,record+0x10,hp) || !readWord(rt,record+0x18,check) || !readWord(rt,record+0x1c,salt))return;
    if(hpChecksum(base,maxHp,hp,salt)!=check)return;
    uint16_t f;std::memcpy(&f,flags,2);if((f^party::kFlagsKey)&0x20)return;
    const uint32_t b=value^party::kBaseMaxHpKey,m=value^party::kMaxHpKey,h=value^party::kHpKey;
    if(b==base && m==maxHp && h==hp)return;
    writeWord(rt,record+0x8,b);writeWord(rt,record+0xc,m);writeWord(rt,record+0x10,h);writeWord(rt,record+0x18,hpChecksum(b,m,h,salt));
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
// The scene's pointer to the running battle (class 16f7a0): its address in the scene, or 0 outside battle.
static uint32_t battleSlot(Runtime *rt){
    uint32_t scene,object,vtable;if(!readWord(rt,0x1d366c,scene) || scene<0x100000 || scene>=0x2000000)return 0;
    for(uint32_t at=scene+party::kBattleSearchFrom;at<scene+party::kBattleSearchTo;at+=4)
        if(readWord(rt,at,object) && object>=0x100000 && object<0x2000000 && readWord(rt,object,vtable) && vtable==party::kBattleClass)return at;
    return 0;
}
// Weak enemies: each enemy unit in the combatant table (the battle's +44..+60) whose record (+818) is a live
// enemy (kind 2, valid HP checksum, not fallen) drops to 1 HP, its checksum rewritten as the HP setter does.
static void weakenEnemies(Runtime *rt){
    uint32_t table;const auto slot=battleSlot(rt);
    if(!slot || !readWord(rt,slot,table))return;
    for(unsigned i=0;i<8;++i){
        uint32_t unit,record,base,maxHp,hp,check,salt;
        if(!readWord(rt,table+0x44+4*i,unit) || !unit || !readWord(rt,unit+party::kUnitRecord,record))continue;
        const auto *kind=rt->memory.hostPointer(record+0x5a,1);const auto *flags=rt->memory.hostPointer(record+0x62,2);
        if(!kind || !flags || record<0x100000 || ((*kind^party::kKindKey)&3)!=2)continue;
        if(!readWord(rt,record+0x8,base) || !readWord(rt,record+0xc,maxHp) || !readWord(rt,record+0x10,hp) || !readWord(rt,record+0x18,check) || !readWord(rt,record+0x1c,salt))continue;
        if(hpChecksum(base,maxHp,hp,salt)!=check)continue;
        uint16_t f;std::memcpy(&f,flags,2);if((f^party::kFlagsKey)&0x20)continue;
        if((hp^party::kHpKey)<=1)continue;
        const uint32_t one=1u^party::kHpKey;writeWord(rt,record+0x10,one);writeWord(rt,record+0x18,hpChecksum(base,maxHp,one,salt));
    }
}
void applyNativeCheats(Runtime *rt){
    if(!rt)return;
    applyMoney(rt);applyItems(rt);applyExp(rt);
    if(nativeCheats().weakEnemies)weakenEnemies(rt);
    // Diagnostic only (VP2S_BOZO_TRACE_PARTY_FLAGS=1): each party battle record's flags (+62 ^ a9c3) and HP when they change.
    static const bool traceFlags=std::getenv("VP2S_BOZO_TRACE_PARTY_FLAGS")!=nullptr;
    if(traceFlags)if(const auto slot=battleSlot(rt)){static uint64_t frame=0;++frame;static std::array<uint32_t,4> last{};
        for(unsigned i=0;i<4;++i){uint32_t record,hp=0;if(!readWord(rt,slot+party::kPartySlots+4*i,record) || record<0x100000 || record>=0x2000000)continue;
            const auto *flags=rt->memory.hostPointer(record+0x62,2);if(!flags)continue;uint16_t f;std::memcpy(&f,flags,2);f^=party::kFlagsKey;readWord(rt,record+0x10,hp);
            const uint32_t key=(uint32_t(f)<<16)^(hp^party::kHpKey);
            if(key!=last[i]){last[i]=key;std::fprintf(stderr,"[vp2s:party] update %llu member %u record %x flags %04x hp %u\n",(unsigned long long)frame,i,record,f,hp^party::kHpKey);}}}
    const uint32_t strong=nativeCheats().strongParty;
    const bool undying=nativeCheats().undying;
    if(!nativeCheats().infiniteHealth && !strong && !undying)return;
    // Battle: the party's battle records, 4 slots after the battle pointer (-1 or empty when unused).
    if(const auto slot=battleSlot(rt))for(unsigned i=0;i<4;++i){uint32_t record;if(!readWord(rt,slot+party::kPartySlots+4*i,record) || record<0x100000 || record>=0x2000000)continue;
        if(undying)undie(rt,record);
        if(strong)strengthen(rt,record,strong);else if(nativeCheats().infiniteHealth)restoreHealth(rt,record);}
}
}
