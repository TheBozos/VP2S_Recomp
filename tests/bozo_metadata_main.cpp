// Synthetic instructions only: no game data or device services.
#include "bozo/runtime.h"
#include "fixture_functions.h"
#include <cstdio>
#include <stdexcept>

static void require(bool value) { if(!value) throw std::runtime_error("metadata dispatch mismatch"); }
static void alternate(uint8_t *,R5900Context *ctx,Runtime *) { ctx->r[2].uw[0]=99; }
int main() {
    try {
        constexpr uint32_t base=0x100000;
        Runtime rt;
        rt.functions.setGenerated(fixture_function_table,fixture_function_count);
        rt.functions.setResume(fixture_resume_table,fixture_resume_count);
        R5900Context ctx;
        ctx.r[31].uw[0]=0x200000;
        auto primary=rt.functions.generated(base);
        auto interior=rt.functions.generated(base+12);
        require(primary && interior);
        bozo_call_indirect(rt.memory.flat(),&ctx,&rt,base);
        require(ctx.r[2].uw[0]==7 && ctx.pc==0x200000);
        ctx.r[2].uw[0]=9;
        bozo_call_indirect(rt.memory.flat(),&ctx,&rt,base+12);
        require(ctx.r[2].uw[0]==10 && ctx.pc==0x200000);
        auto resume=rt.functions.resumeEntry(base+12);
        require(resume && resume->start==base && resume->end==base+24);
        ctx.r[2].uw[0]=19; ctx.pc=base+12;
        resume->resume(rt.memory.flat(),&ctx,&rt);
        require(ctx.r[2].uw[0]==20 && ctx.pc==0x200000);

        // Overlapping compiled images must follow the resident identity, never
        // fall through to stale resident/main code on an identity mismatch.
        uint8_t resident[4]={1,2,3,4};
        const BozoFunctionEntry other[]={{base,alternate}};
        const BozoResumeEntry otherResume[]={{base,base+24,alternate}};
        rt.functions.addModule(base,base+24,resident,{1,2,3,4},fixture_function_table,fixture_function_count,fixture_resume_table,fixture_resume_count);
        rt.functions.addModule(base,base+24,resident,{5,2,3,4},other,1,otherResume,1);
        require(rt.functions.generated(base)==primary);
        require(rt.functions.resumeEntry(base+12)->resume==resume->resume);
        resident[0]=5;
        require(rt.functions.generated(base)==alternate);
        require(rt.functions.resumeEntry(base+12)->resume==alternate);
        resident[0]=8;
        require(!rt.functions.generated(base) && !rt.functions.resumeEntry(base+12));
        require(!rt.functions.generated(base+24));
        std::puts("primary/interior indirect calls, interior resume and overlay identity selection pass");
        return 0;
    } catch(const std::exception &e) { std::fprintf(stderr,"%s\n",e.what()); return 1; }
}
