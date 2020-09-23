#include "goldobot/platform/hal_private.hpp"

namespace goldobot {
namespace hal {
namespace platform {

struct PwmChannel {
  uint8_t timer_id{0xff};
  uint8_t channel_id{0xff};
  PinID sign_pin{0xff};
};

struct DeviceEncoder {
  uint8_t timer_id{0xff};
  uint8_t flags;
};

void hal_timer_init(const DeviceConfigTimer* config);
void hal_pwm_init(const DeviceConfigPwm* config);
void hal_encoder_init(const DeviceConfigEncoder* config);

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
