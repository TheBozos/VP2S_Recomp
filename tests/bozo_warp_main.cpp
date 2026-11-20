// Debug teleport (--warp): argument parsing, the entry-point table and the
// event-script instruction the interpreter is given. Invented table rows only.
#include "native_warp.h"
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
namespace {
int failures=0;
void check(bool ok,const char *what){if(!ok){std::fprintf(stderr,"FAIL: %s\n",what);++failures;}}
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
    check(!vp2s::nativeWarpPending(),"nothing armed");
    vp2s::armNativeWarp(full);check(vp2s::nativeWarpPending(),"armed");
    std::filesystem::remove(table);
    if(failures)return 1;
    std::printf("warp: parsing, table lookup, 10 rejections and the op-0x3a instruction\n");
    return 0;
}
