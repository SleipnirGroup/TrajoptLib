#include "SwervePathBuilderWrap.h"
#include <napi.h>

class TrajoptLibAddon : public Napi::Addon<TrajoptLibAddon> {
 public:
  explicit TrajoptLibAddon(Napi::Env env, Napi::Object exports) {
    exports.Set("SwervePathBuilder", SwervePathBuilderWrap::GetClass(env));
  }
};

NODE_API_ADDON(TrajoptLibAddon)
