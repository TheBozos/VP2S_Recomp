// Debug teleport (--warp): argument parsing, the entry-point table and the
// event-script instruction the interpreter is given. Invented table rows only.
#include "native_warp.h"
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
namespace {
int failures=0;
void check(bool ok,const char *what){if(!ok){std::fprintf(stderr,"FAIL: %s\n",what);++failures;}}
bool rejectsBattle(const std::string &text){try{vp2s::parseBattle(text);}catch(const std::runtime_error &){return true;}return false;}
bool rejects(const std::string &text,const std::string &table){try{vp2s::parseWarp(text,table);}catch(const std::runtime_error &){return true;}return false;}
}
int main(){
    const auto table=(std::filesystem::temp_directory_path()/"vp2s_warp_test.tsv").string();
    {std::ofstream out(table);out<<"# map\tentry\tx\ty\tz\tfacing\tuses\tsource\n"
        "13\t0\t-2155\t0\t3\t270\t5\ta.bin+894\n13\t0\t-1950\t-4\t36\t270\t1\tb.bin+268\n82\t8192\t-2145\t0\t42\t90\t1\tc.bin+30a8\n";}
    const auto full=vp2s::parseWarp("82,-2145,0,42,270,8192",table);
    check(full.map==82 && full.x==-2145 && full.y==0 && full.z==42 && full.facing==270 && full.entry==8192 && full.camera==-1,"full form");
    const auto shortForm=vp2s::parseWarp("0x52,-2145,0,42",table);
    check(shortForm.map==82 && shortForm.facing==0 && shortForm.entry==0,"hex map, default facing and entry");
    const auto first=vp2s::parseWarp("13",table);
    check(first.map==13 && first.x==-2155 && first.z==3 && first.facing==270,"MAP takes the table's first (most used) entry");
    const auto entry=vp2s::parseWarp("82",table);
    check(entry.entry==8192 && entry.facing==90,"MAP keeps the table's entry word");
    for(const char *bad:{"","x","82,1","82,1,2","82,1,2,3,4,5,6","82,,2,3","-1,0,0,0","65536,0,0,0","99999999999,0,0,0","7"})
        check(rejects(bad,table),bad);
    check(rejects("13",table+".missing"),"missing table");
    const auto words=vp2s::warpInstruction(full);
    check(words[0]==0x3a07 && words[1]==82 && words[2]==8192 && int32_t(words[3])==-2145 && words[4]==0 && words[5]==42 && int32_t(words[6])==-1 && words[7]==270,
          "instruction: op 0x3a with 7 arguments in the script's order");
    std::filesystem::remove(table);
    {   // Battle launcher: parsing, the op-0x38 words, then the injection and resume.
        const auto b=vp2s::parseBattle("0x12");check(b.battle==0x12 && b.stage==0 && b.flags==0x01000100,"battle id, default stage and flags");
        const auto full=vp2s::parseBattle("190,38,0x01000300");check(full.battle==190 && full.stage==38 && full.flags==0x01000300,"battle id, stage, flags");
        for(const char *bad:{"","x","-1","65535","1,0x8000","1,2,0x000000ff","1,2,0x080000ff","1,2,3,4","1,,2"})check(rejectsBattle(bad),bad);
        const auto words=vp2s::battleInstruction(full);check(words.size()==3 && words[0]==0x3802 && words[1]==((38u<<16)|190) && words[2]==0x01000300,"instruction: op 0x38 with 2 arguments");
        // A field encounter's form (flags low byte 0xff): op 0x38 with 6 arguments, a one-entry inline formation.
        const auto inline_=vp2s::battleInstruction(vp2s::parseBattle("0x4f,0,0x060000ff"));
        check(inline_==std::vector<uint32_t>{0x3806u,0x4fu,0x060000ffu,0xffff004fu,0xffffffffu,6u,1u},"instruction: inline formation");
        Runtime rt;R5900Context c{};constexpr uint32_t object=0x200000,blob=0x300000,pc=0x10;
        const auto put=[&](uint32_t at,uint32_t v){std::memcpy(rt.memory.hostPointer(at,4),&v,4);};
        const auto get=[&](uint32_t at){uint32_t v;std::memcpy(&v,rt.memory.hostPointer(at,4),4);return v;};
        const uint32_t at=blob+0x20+4*pc;put(object+0x600,blob);put(object+0x18,pc);put(object+0x604,at+4);
        c.r[16].uw[0]=object;c.r[5].uw[0]=at;
        vp2s::armNativeBattle(b);check(vp2s::nativeWarpInstruction(&c,&rt,0x3ee288)==false,"injection keeps the compiled fetch");
        const uint32_t injectedPc=get(object+0x18);
        check(c.r[5].uw[0]==0x80000 && get(0x80000)==0x3802 && get(0x80004)==0x12 && get(0x80008)==0x01000100 && get(object+0x604)==0x80004,"snippet and argument pointer");
        check(blob+0x20+4*injectedPc==0x80000,"pc points at the snippet through the blob (the handler's count read)");
        check(get(0x8000c)==0x42325056 && get(0x80010)==pc,"resume tag and pc in guest RAM (save states keep them)");
        put(object+0x18,injectedPc+3);c.r[5].uw[0]=blob+0x20+4*(injectedPc+3);put(object+0x604,c.r[5].uw[0]+4);   // the handler advanced and yielded
        R5900Context other=c;other.r[16].uw[0]=object+0x1000;other.r[5].uw[0]=0x310000;const auto untouched=other;vp2s::nativeWarpInstruction(&other,&rt,0x3ee288);
        check(!std::memcmp(&other,&untouched,sizeof(other)),"another interpreter's fetch is untouched");
        vp2s::nativeWarpInstruction(&c,&rt,0x3ee288);
        check(get(object+0x18)==pc && c.r[5].uw[0]==at && get(object+0x604)==at+4,"the next fetch resumes the script's own pc");
        c.r[5].uw[0]=at+4;vp2s::nativeWarpInstruction(&c,&rt,0x3ee288);check(c.r[5].uw[0]==at+4,"resumed once only");
    }
    check(!vp2s::nativeWarpPending(),"nothing armed");
    vp2s::armNativeWarp(full);check(vp2s::nativeWarpPending(),"armed");
    if(failures)return 1;
    std::printf("warp: parsing, table lookup, 10 rejections and the op-0x3a instruction; battle: parsing, 8 rejections, injection and resume\n");
    return 0;
}
