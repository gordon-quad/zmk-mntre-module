/*
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#define DT_DRV_COMPAT mnt_led_indicators

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>

#if IS_ENABLED(CONFIG_MNT_LED_INDICATORS_HOST)
#include <raw_hid/events.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>
#endif

LOG_MODULE_DECLARE(mnt, CONFIG_MNT_LOG_LEVEL);

#define INDICATORS_NODE DT_INST(0, DT_DRV_COMPAT)
#define STRIP_NODE DT_PHANDLE(INDICATORS_NODE, led_strip)
#define NUM_LEDS DT_PROP(STRIP_NODE, chain_length)

struct indicator_signal {
  uint8_t led;
  struct led_rgb color;
  uint8_t hid_indicators;
  zmk_keymap_layers_state_t layers;
};

#define LAYER_BIT(node, prop, idx) | BIT(DT_PROP_BY_IDX(node, prop, idx))

#define SIGNAL(node)                                                           \
  {                                                                            \
      .led = DT_PROP(node, led),                                               \
      .color = {.r = (DT_PROP(node, color) >> 16) & 0xff,                      \
                .g = (DT_PROP(node, color) >> 8) & 0xff,                       \
                .b = DT_PROP(node, color) & 0xff},                             \
      .hid_indicators = DT_PROP_OR(node, hid_indicators, 0),                   \
      .layers = 0 COND_CODE_1(DT_NODE_HAS_PROP(node, layers),                  \
                              (DT_FOREACH_PROP_ELEM(node, layers, LAYER_BIT)), \
                              ()),                                             \
  },

#define CHECK_LED(node)                                                        \
  BUILD_ASSERT(DT_PROP(node, led) < NUM_LEDS,                                  \
               "led-indicators: led index beyond the strip's chain-length");

DT_FOREACH_CHILD(INDICATORS_NODE, CHECK_LED)

static const struct indicator_signal signals[] = {
    DT_FOREACH_CHILD(INDICATORS_NODE, SIGNAL)};

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

// Channels claimed by any signal; the host may set only the others
static struct led_rgb owned[NUM_LEDS];
static struct led_rgb host[NUM_LEDS];
static struct k_spinlock host_lock;
static atomic_t hid_indicators;

static void render(struct k_work *work) {
  struct led_rgb pixels[NUM_LEDS];
  zmk_keymap_layers_state_t layers = zmk_keymap_layer_state();
  uint8_t hid = atomic_get(&hid_indicators);

  K_SPINLOCK(&host_lock) {
    for (int i = 0; i < NUM_LEDS; i++) {
      pixels[i].r = owned[i].r ? 0 : host[i].r;
      pixels[i].g = owned[i].g ? 0 : host[i].g;
      pixels[i].b = owned[i].b ? 0 : host[i].b;
    }
  }

  for (size_t i = 0; i < ARRAY_SIZE(signals); i++) {
    const struct indicator_signal *s = &signals[i];
    if (!(s->hid_indicators & hid) && !(s->layers & layers)) {
      continue;
    }
    struct led_rgb *p = &pixels[s->led];
    p->r = MAX(p->r, s->color.r);
    p->g = MAX(p->g, s->color.g);
    p->b = MAX(p->b, s->color.b);
  }

  int ret = led_strip_update_rgb(strip, pixels, NUM_LEDS);
  if (ret < 0) {
    LOG_ERR("Failed to update the indicator LEDs: %d", ret);
  }
}

static K_WORK_DEFINE(render_work, render);

#if IS_ENABLED(CONFIG_MNT_LED_INDICATORS_HOST)
// Raw HID report from the host: led (0xff = all), r, g, b; the rest is padding
static bool host_set(const uint8_t *r, uint8_t len) {
  if (len < 4 || (r[0] != 0xff && r[0] >= NUM_LEDS)) {
    return false;
  }

  K_SPINLOCK(&host_lock) {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (r[0] == 0xff || r[0] == i) {
        host[i] = (struct led_rgb){.r = r[1], .g = r[2], .b = r[3]};
      }
    }
  }
  return true;
}
#endif

static int indicators_event_listener(const zmk_event_t *eh) {
  const struct zmk_hid_indicators_changed *ind =
      as_zmk_hid_indicators_changed(eh);
  if (ind != NULL) {
    atomic_set(&hid_indicators, ind->indicators);
  }

#if IS_ENABLED(CONFIG_MNT_LED_INDICATORS_HOST)
  // A suspended or gone host can't clear its LEDs, so drop them
  const struct zmk_usb_conn_state_changed *usb =
      as_zmk_usb_conn_state_changed(eh);
  if (usb != NULL && usb->conn_state != ZMK_USB_CONN_HID) {
    K_SPINLOCK(&host_lock) { memset(host, 0, sizeof(host)); }
  }

  const struct raw_hid_received_event *raw = as_raw_hid_received_event(eh);
  if (raw != NULL && !host_set(raw->data, raw->length)) {
    return ZMK_EV_EVENT_BUBBLE;
  }
#endif

  k_work_submit(&render_work);
  return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_indicators, indicators_event_listener);
ZMK_SUBSCRIPTION(led_indicators, zmk_hid_indicators_changed);
ZMK_SUBSCRIPTION(led_indicators, zmk_layer_state_changed);
#if IS_ENABLED(CONFIG_MNT_LED_INDICATORS_HOST)
ZMK_SUBSCRIPTION(led_indicators, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(led_indicators, raw_hid_received_event);
#endif

static int led_indicators_init(void) {
  if (!device_is_ready(strip)) {
    LOG_ERR("Indicator LED strip not ready");
    return -ENODEV;
  }

  for (size_t i = 0; i < ARRAY_SIZE(signals); i++) {
    struct led_rgb *o = &owned[signals[i].led];
    o->r |= signals[i].color.r;
    o->g |= signals[i].color.g;
    o->b |= signals[i].color.b;
  }

  k_work_submit(&render_work);
  return 0;
}

SYS_INIT(led_indicators_init, APPLICATION,
         CONFIG_MNT_LED_INDICATORS_INIT_PRIORITY);
