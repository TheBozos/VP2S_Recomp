#pragma once
#include "bozo/context.h"
#include "../port/vp2s_bootstrap.h"
namespace vp2s {
class BootstrapOwner {
public:
    virtual ~BootstrapOwner()=default;
    virtual const bootstrap::Tables &bootstrapTables() const=0;
};
bool nativeBootstrapKey(uint8_t *,R5900Context *,Runtime *,uint32_t);
}
