#include "twc_controller.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace twc_controller {

static const char *const TAG = "twc";

// ---- Protocol constants (see docs/TWC-RS485-Protocol.md) ----
static const uint8_t SLIP_END = 0xC0;
static const uint8_t SLIP_ESC = 0xDB;
static const uint8_t SLIP_ESC_END = 0xDC;
static const uint8_t SLIP_ESC_ESC = 0xDD;
static const uint8_t FRAME_TRAILER = 0xFE;  // real TWCs end every frame with C0 FE

static const uint16_t CMD_MASTER_LINKREADY1 = 0xFCE1;
static const uint16_t CMD_MASTER_LINKREADY2 = 0xFBE2;
static const uint16_t CMD_MASTER_HEARTBEAT = 0xFBE0;
static const uint16_t CMD_SLAVE_LINKREADY = 0xFDE2;
static const uint16_t CMD_SLAVE_HEARTBEAT = 0xFDE0;
static const uint16_t CMD_START_CHARGING = 0xFCB1;
static const uint16_t CMD_STOP_CHARGING = 0xFCB2;
static const uint16_t CMD_ACK_START = 0xFDB1;
static const uint16_t CMD_ACK_STOP = 0xFDB2;
static const uint16_t CMD_GET_FW_OLD = 0xFB1B;
static const uint16_t RSP_FW_OLD = 0xFD1B;
static const uint16_t CMD_GET_MODEL = 0xFB1A;  // read only. FC1A (write) is forbidden.
static const uint16_t RSP_MODEL = 0xFD1A;
static const uint16_t CMD_GET_PLUG = 0xFBB4;
static const uint16_t RSP_PLUG = 0xFDB4;
static const uint16_t CMD_GET_METER = 0xFBEB;
static const uint16_t RSP_METER = 0xFDEB;
static const uint16_t CMD_GET_FW = 0xFBEC;
static const uint16_t RSP_FW = 0xFDEC;
static const uint16_t CMD_GET_SERIAL = 0xFBED;
static const uint16_t RSP_SERIAL = 0xFDED;
static const uint16_t CMD_GET_VIN1 = 0xFBEE;
static const uint16_t RSP_VIN1 = 0xFDEE;
static const uint16_t CMD_GET_VIN2 = 0xFBEF;
static const uint16_t RSP_VIN2 = 0xFDEF;
static const uint16_t CMD_GET_VIN3 = 0xFBF1;
static const uint16_t RSP_VIN3 = 0xFDF1;
static const uint16_t CMD_IDLE = 0xFC1D;
// Commands that write OTP memory or crash the TWC. Never sent, refused in send_frame_().
static const uint16_t CMD_FORBIDDEN[] = {0xFC19, 0xFC1A, 0xFBE8, 0xFCEA, 0xFCE5, 0xFCA1, 0xFBA4};

// ---- Timing ----
static const uint32_t BUS_IDLE_MS = 25;          // no RX byte for this long = bus quiet (1 frame ~20 ms)
static const uint32_t MIN_TX_GAP_MS = 120;       // between our own frames
static const uint32_t RX_FRAME_TIMEOUT_MS = 100; // drop half-received frame
static const uint32_t LINKREADY_INTERVAL_MS = 1000;
static const uint32_t HEARTBEAT_INTERVAL_MS = 1000;  // per TWC; split into one slot per configured charger
static const uint32_t OFFLINE_AFTER_MS = 10000;
static const uint32_t FORGET_AFTER_MS = 30000;   // real master gives up after ~26 s

// flush() returns UARTFlushResult in newer ESPHome and void in older. Pick at compile time.
// Result values: 0 = SUCCESS, 3 = ASSUMED_SUCCESS.
template<typename U> static auto flush_ok_(U *u, int) -> decltype(static_cast<int>(u->flush()), bool()) {
  int r = static_cast<int>(u->flush());
  return r == 0 || r == 3;
}
template<typename U> static bool flush_ok_(U *u, long) {
  u->flush();
  return true;
}

static inline uint16_t be16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }
static inline uint32_t be32(const uint8_t *p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

// ================= child entities =================

void TWCCurrentNumber::control(float value) { this->parent_->on_current_limit(value); }

void TWCSettingNumber::control(float value) { this->parent_->on_setting(this->setting_, value); }

void TWCChargingSwitch::write_state(bool state) { this->parent_->on_charging_enabled(state); }

void TWCModeSelect::control(size_t index) { this->parent_->on_mode(index); }

void TWCCarTypeSelect::control(size_t index) { this->parent_->on_car_type(index); }

void TWCOffsetNumber::control(float value) { this->parent_->on_energy_offset(value); }

void TWCCharger::on_energy_offset(float kwh) {
  if (std::isnan(kwh))
    return;
  this->offset_kwh_ = kwh;
  this->offset_pref_.save(&kwh);
  if (this->offset_number_ != nullptr)
    this->offset_number_->publish_state(kwh);
  ESP_LOGI(TAG, "[%u] Energy offset -> %.2f kWh", this->index_ + 1, kwh);
  this->publish_total_energy_();
}

void TWCCharger::add_energy_(double kwh) {
  if (!(kwh > 0))
    return;
  this->lifetime_kwh_ += kwh;
  // Flash write once per whole kWh (a power cut loses at most 1 kWh); session end saves the rest.
  if (std::floor(this->lifetime_kwh_) != std::floor(this->lifetime_saved_kwh_)) {
    this->lifetime_saved_kwh_ = this->lifetime_kwh_;
    this->lifetime_pref_.save(&this->lifetime_kwh_);
  }
  if (this->lifetime_kwh_ - this->lifetime_published_kwh_ >= 0.01)
    this->publish_total_energy_();
}

void TWCCharger::publish_total_energy_() {
  this->lifetime_published_kwh_ = this->lifetime_kwh_;
  if (this->total_energy_sensor_ != nullptr)
    this->total_energy_sensor_->publish_state(this->lifetime_kwh_ + this->offset_kwh_);
}

static const CarKind CAR_TYPE_OPTIONS[] = {CAR_UNKNOWN, CAR_1_PHASE, CAR_3_SUB, CAR_2_SUB};  // Auto, 1-phase, 3-phase, 2 sub

void TWCCharger::on_car_type(size_t index) {
  uint8_t i = index < 4 ? index : 0;
  if (CAR_TYPE_OPTIONS[i] != this->manual_kind_)
    ESP_LOGI(TAG, "[%u] Car type -> %s", this->index_ + 1, i == 0 ? "auto" : (i == 1 ? "1-phase" : (i == 2 ? "3-phase" : "2 sub-chargers")));
  this->manual_kind_ = CAR_TYPE_OPTIONS[i];
  this->car_type_pref_.save(&i);
  if (this->car_type_select_ != nullptr)
    this->car_type_select_->publish_state((size_t) i);
  this->publish_phase_data_();
}

void TWCController::set_circuit_power_sensor(sensor::Sensor *s) {
  this->circuit_power_ = s;
  s->add_on_state_callback([this](float w) {
    if (std::isnan(w))
      return;
    uint32_t now = millis();
    this->circuit_samples_.emplace_back(now, w);
    while (!this->circuit_samples_.empty() &&
           (now - this->circuit_samples_.front().first > 600000 || this->circuit_samples_.size() > 300))
      this->circuit_samples_.pop_front();
  });
}

// Mean and spread (max - min) of the circuit power in [from, to]. carry: the reading just before `from`
// counts too (HA only sends changes). Needs at least 2 readings.
bool TWCController::circuit_window_(uint32_t from, uint32_t to, bool carry, float &mean, float &spread) const {
  float sum = 0, mn = 1e9f, mx = -1e9f;
  int n = 0;
  const std::pair<uint32_t, float> *before = nullptr;
  for (auto &s : this->circuit_samples_) {
    if ((int32_t) (s.first - from) < 0) {
      if (carry)
        before = &s;
      continue;
    }
    if ((int32_t) (s.first - to) > 0)
      break;
    if (before != nullptr && n == 0) {
      sum += before->second, mn = std::min(mn, before->second), mx = std::max(mx, before->second), n++;
    }
    sum += s.second, mn = std::min(mn, s.second), mx = std::max(mx, s.second), n++;
  }
  if (n == 0 && before != nullptr) {  // no change in the window: the last reading still holds
    sum = mn = mx = before->second;
    n = 2;
  }
  if (n < 2)
    return false;
  mean = sum / n;
  spread = mx - mn;
  return true;
}

void TWCController::on_mode(size_t index) {
  uint8_t m = index == 1 ? 1 : 0;
  if (m != this->mode_)
    ESP_LOGI(TAG, "Control mode -> %s", m ? "power (Max Power)" : "amps (Set Current)");
  this->mode_ = m;
  this->mode_pref_.save(&m);
  if (this->mode_select_ != nullptr)
    this->mode_select_->publish_state((size_t) m);
  this->update_limits_();
}

// ================= controller: setup / config =================

void TWCController::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);
  }

  for (uint8_t s = 0; s < SET_COUNT; s++) {
    auto *n = this->setting_number_[s];
    if (n == nullptr)
      continue;
    this->setting_pref_[s] = global_preferences->make_preference<float>(n->get_object_id_hash());
    float restored;
    if (this->setting_pref_[s].load(&restored) && !std::isnan(restored))
      this->setting_[s] = clamp<float>(restored, n->traits.get_min_value(), n->traits.get_max_value());
    n->publish_state(this->setting_[s]);
  }
  if (this->mode_select_ != nullptr) {
    this->mode_pref_ = global_preferences->make_preference<uint8_t>(this->mode_select_->get_object_id_hash());
    uint8_t m;
    if (this->mode_pref_.load(&m) && m <= 1)
      this->mode_ = m;
    this->mode_select_->publish_state((size_t) this->mode_);
  }

  for (auto *c : this->chargers_)
    c->setup_();
  this->update_limits_();

  std::string s1, s2;  // VIN decoder self-check (made-up VINs: 2025 Model Y, 2017 Model X)
  if (decode_vin("XP7YGCEK0SB000001", s1) != CAR_3_SUB || s1 != "Tesla Model Y 2025" ||
      decode_vin("5YJXCDE26HF012345", s2) != CAR_2_SUB || s2 != "Tesla Model X 2017")
    ESP_LOGE(TAG, "VIN decode self-check FAILED: '%s' / '%s'", s1.c_str(), s2.c_str());
}

void TWCController::dump_config() {
  ESP_LOGCONFIG(TAG, "TWC Controller:");
  ESP_LOGCONFIG(TAG, "  Our TWC ID: %04X, chargers: %u", this->twcid_, (unsigned) this->chargers_.size());
  ESP_LOGCONFIG(TAG, "  Current: min %u A, max %u A per charger", this->min_current_, this->max_current_);
  if (this->setting_number_[SET_TOTAL_AMPS] != nullptr)
    ESP_LOGCONFIG(TAG, "  Total current: %.0f A (last charger gets the rest, < %u A -> paused)",
                  this->setting_[SET_TOTAL_AMPS], this->min_current_);
  if (this->setting_number_[SET_MAX_POWER] != nullptr)
    ESP_LOGCONFIG(TAG, "  Max power: %.0f W (0 = off), priority charger 1: %.0f %%", this->setting_[SET_MAX_POWER],
                  this->setting_[SET_PRIORITY]);
  ESP_LOGCONFIG(TAG, "  Grid: %s, nominal voltage: %.0f V, phase threshold %.1f A", GRID_NAMES[this->grid_type_],
                this->nominal_voltage_, this->phase_threshold_);
  LOG_PIN("  Flow control pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Frames OK/bad: %u/%u", (unsigned) this->rx_frames_ok_, (unsigned) this->rx_frames_bad_);
  for (auto *c : this->chargers_)
    c->dump_config_();
}

void TWCController::on_setting(uint8_t s, float value) {
  static const char *const NAMES[] = {"Total current", "Max power", "Priority charger 1"};
  if (value != this->setting_[s])
    ESP_LOGI(TAG, "%s -> %.0f", NAMES[s], value);
  this->setting_[s] = value;
  this->setting_pref_[s].save(&value);
  if (this->setting_number_[s] != nullptr)
    this->setting_number_[s]->publish_state(value);
  this->update_limits_();
}

// Limits, in two steps:
// 1. Wanted amps: own setting (Set Current), or with max power on: the power split below (overrides it).
// 2. Shared fuse split in YAML order: max_current (main fuse), lowered by Total Current if set, is shared by
//    all chargers. Each takes min(wanted, rest); the last of several chargers (no Set Current) takes the rest.
//    Below min_current -> 0 A (paused).
// Whole amps; below min_current -> 0 (some cars latch "unplug and replug" below 6 A, so pause instead).
float TWCController::whole_amps_(float a) const {
  a = std::floor(a + 0.001f);
  return a < this->min_current_ ? 0 : a;
}

void TWCController::update_limits_() {
  std::vector<float> want;
  for (auto *c : this->chargers_)
    want.push_back(c->current_number_ != nullptr ? c->limit_amps_ : this->max_current_);
  if (this->power_mode_())
    this->power_split_(want);

  float remaining = this->max_current_;
  if (this->setting_number_[SET_TOTAL_AMPS] != nullptr)
    remaining = std::min(remaining, this->setting_[SET_TOTAL_AMPS]);
  for (size_t i = 0; i < this->chargers_.size(); i++) {
    auto *c = this->chargers_[i];
    float cap = std::min(remaining, (float) this->max_current_);
    if (c->slave_max_amps_ > 0)
      cap = std::min(cap, c->slave_max_amps_);
    float base = std::min(want[i], cap);
    float a = base;
    if (c->probe_delta_ != 0 && a >= this->min_current_)  // quick type check: 4 A step, never over the fuse
      a = std::max((float) this->min_current_, std::min(a + c->probe_delta_, cap));
    a = this->whole_amps_(a);
    // A probe step down does not free amps for the others: they must stay still while it measures.
    remaining -= std::max(a, this->whole_amps_(base));
    c->set_assigned_(a);
  }
}

// Smart power control: max power (W) is split by priority (charger 1 gets priority %, the others share the
// rest). Each car's amps = its power share / watts per amp for that car (3-phase, 2 sub-chargers, 1-phase).
// A car below min_current is raised to min, paid from the others. Not enough for all at min -> the lowest
// priority car is paused. No car / full car: gets min_current (so it can start) and no share.
void TWCController::power_split_(std::vector<float> &want) {
  size_t n = this->chargers_.size();
  uint32_t now = millis();
  float prio = clamp<float>(this->setting_[SET_PRIORITY], 0, 100);
  std::vector<float> w(n), k(n);
  std::vector<bool> active(n);
  for (size_t i = 0; i < n; i++) {
    auto *c = this->chargers_[i];
    w[i] = n == 1 ? 100.0f : (i == 0 ? prio : (100.0f - prio) / (n - 1));
    k[i] = c->watts_per_amp_();
    bool plugged = c->have_slave_ && c->charging_enabled_ && c->slave_state_ != 0x00 && c->slave_state_ != 0xFF;
    bool full = c->idle_since_ms_ != 0 && now - c->idle_since_ms_ >= 120000;
    active[i] = plugged && !full;
    if (!active[i])
      want[i] = this->min_current_;
  }
  // Repeat while a car has to be paused for lack of power.
  for (size_t pass = 0; pass <= n; pass++) {
    float budget = this->setting_[SET_MAX_POWER];
    std::vector<int8_t> st(n);  // 0 = gets a share, 1 = fixed at min/max, -1 = not in the split
    for (size_t i = 0; i < n; i++)
      st[i] = active[i] ? 0 : -1;
    // Water-fill: fix a car that hits min or max, split the rest again.
    for (size_t round = 0; round <= n; round++) {
      float sum_w = 0;
      size_t free = 0;
      for (size_t i = 0; i < n; i++)
        if (st[i] == 0) {
          sum_w += w[i];
          free++;
        }
      if (free == 0)
        break;
      bool fixed = false;
      for (size_t i = 0; i < n && !fixed; i++) {
        if (st[i] != 0)
          continue;
        float share = sum_w > 0 ? w[i] / sum_w : 1.0f / free;
        float a = std::max(0.0f, budget) * share / k[i];
        if (a > this->max_current_ || a < this->min_current_) {
          want[i] = a > this->max_current_ ? this->max_current_ : this->min_current_;
          budget -= k[i] * want[i];
          st[i] = 1;
          fixed = true;
        } else {
          want[i] = a;
        }
      }
      if (!fixed)
        break;
    }
    // 1 % slack: 6 A + 6 A on IT (3974 W with 1.73) must fit with sqrt(3).
    if (budget >= -0.01f * this->setting_[SET_MAX_POWER])
      return;
    // Over budget with cars at minimum: pause the lowest priority one (last in YAML on a tie), split again.
    int worst = -1;
    for (size_t i = 0; i < n; i++)
      if (active[i] && (worst < 0 || w[i] <= w[worst]))
        worst = i;
    if (worst < 0)
      return;
    active[worst] = false;
    want[worst] = 0;
  }
}

// ================= charger: setup / entity callbacks =================

void TWCCharger::setup_() {
  auto *p = this->parent_;
  // Current limit: restore last value, else use set_current from YAML.
  if (this->current_number_ != nullptr) {
    this->current_pref_ = global_preferences->make_preference<float>(this->current_number_->get_object_id_hash());
    float restored;
    if (this->current_pref_.load(&restored) && !std::isnan(restored))
      this->limit_amps_ = (uint8_t) lroundf(restored);
  }
  this->limit_amps_ = clamp<uint8_t>(this->limit_amps_, p->min_current_, p->max_current_);
  if (this->current_number_ != nullptr)
    this->current_number_->publish_state(this->limit_amps_);
  // "Max Ampere" = installation limit from YAML (the TWC's own rating, e.g. 32 A, is only logged).
  if (this->max_allowable_current_sensor_ != nullptr)
    this->max_allowable_current_sensor_->publish_state(p->max_current_);

  // Charging switch: honours restore_mode (default RESTORE_DEFAULT_ON).
  if (this->charging_switch_ != nullptr) {
    auto initial = this->charging_switch_->get_initial_state_with_restore_mode();
    this->charging_enabled_ = initial.has_value() ? *initial : true;
    this->charging_switch_->publish_state(this->charging_enabled_);
  }
  if (this->car_type_select_ != nullptr) {
    this->car_type_pref_ = global_preferences->make_preference<uint8_t>(this->car_type_select_->get_object_id_hash());
    uint8_t i = 0;
    if (!this->car_type_pref_.load(&i) || i > 3)
      i = 0;
    this->manual_kind_ = CAR_TYPE_OPTIONS[i];
    this->car_type_select_->publish_state((size_t) i);
  }
  // Lifetime counter + offset (keys fixed per charger so renaming the entities keeps the count).
  this->lifetime_pref_ = global_preferences->make_preference<double>(0x7C3A1000u + this->index_);
  this->offset_pref_ = global_preferences->make_preference<float>(0x7C3A2000u + this->index_);
  double lt;
  if (this->lifetime_pref_.load(&lt) && std::isfinite(lt) && lt >= 0)
    this->lifetime_kwh_ = this->lifetime_saved_kwh_ = lt;
  float off;
  if (this->offset_pref_.load(&off) && std::isfinite(off))
    this->offset_kwh_ = off;
  if (this->offset_number_ != nullptr)
    this->offset_number_->publish_state(this->offset_kwh_);
  this->publish_total_energy_();
  if (this->type_check_bs_ != nullptr)
    this->type_check_bs_->publish_state(false);

  this->set_online_(false);
  if (this->state_text_ != nullptr)
    this->state_text_->publish_state("No TWC");
}

void TWCCharger::dump_config_() {
  ESP_LOGCONFIG(TAG, "  Charger %u: TWC %s%04X, own limit %u A, assigned %.0f A, charging %s", this->index_ + 1,
                this->pinned_id_ ? "pinned " : "", this->have_slave_ ? this->slave_id_ : this->pinned_id_,
                this->limit_amps_, this->assigned_amps_, YESNO(this->charging_enabled_));
}

void TWCCharger::on_current_limit(float amps) {
  auto *p = this->parent_;
  uint8_t a = clamp<uint8_t>((uint8_t) lroundf(amps), p->min_current_, p->max_current_);
  if (a != this->limit_amps_)
    ESP_LOGI(TAG, "[%u] Current limit -> %u A", this->index_ + 1, a);
  this->limit_amps_ = a;
  float saved = a;
  this->current_pref_.save(&saved);
  if (this->current_number_ != nullptr)
    this->current_number_->publish_state(a);
  this->last_limit_sent_ms_ = 0;
  p->update_limits_();
}

void TWCCharger::on_charging_enabled(bool enabled) {
  if (enabled != this->charging_enabled_) {
    ESP_LOGI(TAG, "[%u] Charging %s", this->index_ + 1, enabled ? "ENABLED" : "DISABLED");
    this->charging_enabled_ = enabled;
    this->update_run_();
    this->last_limit_sent_ms_ = 0;
  }
  if (this->charging_switch_ != nullptr)
    this->charging_switch_->publish_state(enabled);
}

void TWCCharger::set_assigned_(float amps) {
  if (amps == this->assigned_amps_)
    return;
  if (this->assigned_amps_ >= 0)
    ESP_LOGI(TAG, "[%u] Assigned %.0f A%s", this->index_ + 1, amps, amps == 0 ? " (paused)" : "");
  this->assigned_amps_ = amps;
  if (this->assigned_current_sensor_ != nullptr)
    this->assigned_current_sensor_->publish_state(amps);
  this->last_limit_sent_ms_ = 0;
  this->update_run_();
}

// Run = enabled and > 0 A assigned. Changes send FCB1 (start) / FCB2 (stop) x3, because a 0 A heartbeat
// alone is ignored by some TWC firmware (car stays at 6 A).
void TWCCharger::update_run_() {
  bool run = this->charging_enabled_ && this->assigned_amps_ > 0;
  if (run == this->running_)
    return;
  this->running_ = run;
  if (!this->have_slave_)
    return;
  ESP_LOGI(TAG, "[%u] %s", this->index_ + 1, run ? "START (FCB1)" : "STOP (FCB2)");
  this->parent_->drop_commands_(this);  // drop an opposite command still waiting
  this->parent_->queue_command_(this, run ? CMD_START_CHARGING : CMD_STOP_CHARGING, 3);
  if (!run)
    this->last_stop_sent_ms_ = millis();
}

// ================= main loop =================

void TWCController::loop() {
  this->read_bus_();
  this->schedule_(millis());
}

// ---------------- RX ----------------

void TWCController::read_bus_() {
  uint32_t now = millis();
  if (this->rx_in_frame_ && this->rx_len_ > 0 && now - this->last_rx_byte_ms_ > RX_FRAME_TIMEOUT_MS) {
    ESP_LOGV(TAG, "RX timeout, dropping %u bytes", (unsigned) this->rx_len_);
    this->rx_in_frame_ = false;
    this->rx_len_ = 0;
    this->rx_escape_ = false;
  }

  uint8_t c;
  while (this->available() > 0 && this->read_byte(&c)) {
    this->last_rx_byte_ms_ = millis();
    if (this->rx_bytes_++ < 64) {
      ESP_LOGV(TAG, "RX byte %02X", c);
    }

    if (c == SLIP_END) {
      if (this->rx_in_frame_ && this->rx_len_ >= 4) {
        // Possible end of frame. Valid -> process. Invalid -> this C0 is a new start.
        size_t len = this->rx_len_;
        uint8_t sum = 0;
        for (size_t i = 1; i + 1 < len; i++)
          sum += this->rx_buf_[i];
        if (len >= 13 && sum == this->rx_buf_[len - 1]) {
          this->rx_frames_ok_++;
          this->process_frame_(this->rx_buf_, len);
          this->rx_in_frame_ = false;
        } else {
          this->rx_frames_bad_++;
          ESP_LOGV(TAG, "Bad frame (%u bytes, cksum %02X != %02X): %s", (unsigned) len, sum,
                   this->rx_buf_[len - 1], format_hex_pretty(this->rx_buf_, len).c_str());
          this->rx_in_frame_ = true;
        }
      } else {
        this->rx_in_frame_ = true;  // start (or re-sync on a short run of garbage)
      }
      this->rx_len_ = 0;
      this->rx_escape_ = false;
      continue;
    }

    if (!this->rx_in_frame_)
      continue;  // trailer byte (FE) or noise between frames

    if (this->rx_escape_) {
      this->rx_escape_ = false;
      if (c == SLIP_ESC_END)
        c = SLIP_END;
      else if (c == SLIP_ESC_ESC)
        c = SLIP_ESC;
      else {
        this->rx_in_frame_ = false;  // illegal escape, drop
        this->rx_len_ = 0;
        this->rx_frames_bad_++;
        continue;
      }
    } else if (c == SLIP_ESC) {
      this->rx_escape_ = true;
      continue;
    }

    if (this->rx_len_ >= MAX_FRAME) {
      this->rx_in_frame_ = false;
      this->rx_len_ = 0;
      this->rx_frames_bad_++;
      continue;
    }
    this->rx_buf_[this->rx_len_++] = c;
  }
}

TWCCharger *TWCController::find_(uint16_t id) {
  for (auto *c : this->chargers_)
    if (c->have_slave_ && c->slave_id_ == id)
      return c;
  return nullptr;
}

// A new TWC: the charger pinned to its ID, else the first free unpinned one (YAML order).
TWCCharger *TWCController::claim_(uint16_t id) {
  for (auto *c : this->chargers_)
    if (!c->have_slave_ && c->pinned_id_ == id)
      return c;
  for (auto *c : this->chargers_)
    if (c->pinned_id_ == id)
      return nullptr;  // pinned charger already busy (should not happen)
  for (auto *c : this->chargers_)
    if (!c->have_slave_ && c->pinned_id_ == 0)
      return c;
  return nullptr;
}

bool TWCController::any_slave_() const {
  for (auto *c : this->chargers_)
    if (c->have_slave_)
      return true;
  return false;
}

void TWCController::process_frame_(const uint8_t *f, size_t len) {
  uint16_t cmd = be16(f);
  uint16_t src = be16(f + 2);
  ESP_LOGV(TAG, "RX %s", format_hex_pretty(f, len).c_str());

  if (src == this->twcid_ && (cmd >> 8) != 0xFD) {
    return;  // our own frame (echo from transceiver)
  }

  switch (cmd) {
    case CMD_SLAVE_LINKREADY:
      this->handle_slave_linkready_(f, len);
      return;
    case CMD_SLAVE_HEARTBEAT: {
      if (be16(f + 4) != this->twcid_)
        return;
      TWCCharger *c = this->find_(src);
      if (c == nullptr) {
        // We rebooted while the TWC was still linked: adopt it.
        c = this->claim_(src);
        if (c == nullptr)
          return;
        c->adopt_(src, (len <= 14) ? 1 : 2, 0);
        ESP_LOGI(TAG, "[%u] Adopted TWC %04X from heartbeat, protocol %u", c->index_ + 1, src, c->protocol_);
        this->startup_linkready1_ = this->startup_linkready2_ = 0;
        this->update_limits_();
      }
      c->handle_heartbeat_(f, len);
      return;
    }
    case RSP_MODEL:
    case RSP_FW_OLD: {
      // These can come without a TWC ID (older fw): then they answer the charger we just polled.
      TWCCharger *c = this->find_(src);
      if (c == nullptr)
        c = this->last_polled_;
      if (c != nullptr && c->have_slave_)
        c->process_(cmd, f, len);
      return;
    }
    case RSP_METER:
    case RSP_PLUG:
    case RSP_VIN1:
    case RSP_VIN2:
    case RSP_VIN3:
    case RSP_SERIAL:
    case RSP_FW: {
      TWCCharger *c = this->find_(src);
      if (c != nullptr)
        c->process_(cmd, f, len);
      return;
    }
    case CMD_ACK_START:
    case CMD_ACK_STOP:
      ESP_LOGD(TAG, "TWC %04X acknowledged %s", src, cmd == CMD_ACK_START ? "START" : "STOP");
      return;
    case CMD_MASTER_LINKREADY1:
    case CMD_MASTER_LINKREADY2:
    case CMD_MASTER_HEARTBEAT:
      ESP_LOGW(TAG, "Another master (%04X) on the bus! Set the TWC rotary switch to F.", src);
      return;
    case CMD_IDLE:
      return;
    default:
      ESP_LOGD(TAG, "Unhandled frame %04X: %s", cmd, format_hex_pretty(f, len).c_str());
      return;
  }
}

void TWCController::handle_slave_linkready_(const uint8_t *f, size_t len) {
  uint16_t id = be16(f + 2);
  float max_amps = be16(f + 5) / 100.0f;

  if (id == this->twcid_) {
    ESP_LOGW(TAG, "TWC uses our ID %04X - sending linkready so it picks a new one", id);
    this->startup_linkready1_ = 5;
    this->startup_linkready2_ = 5;
    return;
  }
  TWCCharger *c = this->find_(id);
  if (c == nullptr) {
    c = this->claim_(id);
    if (c == nullptr) {
      ESP_LOGW(TAG, "TWC %04X ignored: no free charger in YAML (add it under chargers:)", id);
      return;
    }
    c->adopt_(id, (len <= 14) ? 1 : 2, max_amps);
    ESP_LOGI(TAG, "[%u] Found TWC %04X, sign %02X, max %.2f A, protocol %u", c->index_ + 1, id, f[4], max_amps,
             c->protocol_);
    // A real master stops its startup burst once it has a slave.
    this->startup_linkready1_ = this->startup_linkready2_ = 0;
    this->update_limits_();
  }
  c->last_slave_hb_ms_ = millis();
  c->last_limit_sent_ms_ = 0;
}

// ================= charger: RX handlers =================

void TWCCharger::adopt_(uint16_t id, uint8_t protocol, float max_amps) {
  this->have_slave_ = true;
  this->slave_id_ = id;
  this->protocol_ = protocol;
  this->slave_max_amps_ = max_amps;
  this->have_serial_ = this->have_firmware_ = this->have_model_ = false;
  this->vin_parts_ = 0;
  this->poll_index_ = 0;
  this->last_slave_hb_ms_ = millis();
  this->last_limit_sent_ms_ = 0;
}

void TWCCharger::forget_() {
  ESP_LOGW(TAG, "[%u] Forgetting TWC %04X, waiting for linkready", this->index_ + 1, this->slave_id_);
  this->have_slave_ = false;
  this->slave_state_ = 0xFF;
  this->parent_->drop_commands_(this);
}

void TWCCharger::supervise_(uint32_t now) {
  if (!this->have_slave_)
    return;
  uint32_t silent = now - this->last_slave_hb_ms_;
  if (this->online_ && silent > OFFLINE_AFTER_MS) {
    ESP_LOGW(TAG, "[%u] TWC %04X silent for %u s", this->index_ + 1, this->slave_id_, (unsigned) (silent / 1000));
    this->set_online_(false);
    if (this->state_text_ != nullptr)
      this->state_text_->publish_state("Offline");
  }
  if (silent > FORGET_AFTER_MS)
    this->forget_();
}

void TWCCharger::process_(uint16_t cmd, const uint8_t *f, size_t len) {
  uint16_t src = be16(f + 2);
  switch (cmd) {
    case RSP_METER:
      if (len >= 15)
        this->handle_meter_(f, len);
      return;
    case RSP_PLUG: {
      if (len < 6)
        return;
      // 2 = start failed / error (seen after FCB1 when the car rejects it)
      static const char *const PLUG[] = {"Unplugged", "Charging", "Error", "Plugged in, not charging"};
      const char *t = f[4] < 4 ? PLUG[f[4]] : "Unknown";
      if (this->plug_text_ != nullptr && this->plug_text_->state != t)
        this->plug_text_->publish_state(t);
      return;
    }
    case RSP_MODEL: {
      // Two layouts seen: with TWC ID (model at byte 4) or without (byte 2, older fw).
      std::string m;
      size_t off = src == this->slave_id_ ? 4 : 2;
      if (!TWCController::ascii_field_(f + off, len - 1 - off, m))
        return;
      this->have_model_ = true;
      ESP_LOGI(TAG, "[%u] Model: %s", this->index_ + 1, m.c_str());
      if (this->model_text_ != nullptr)
        this->model_text_->publish_state(m);
      return;
    }
    case RSP_VIN1:
    case RSP_VIN2:
    case RSP_VIN3:
      if (len >= 12)
        this->handle_vin_(f, cmd == RSP_VIN1 ? 0 : (cmd == RSP_VIN2 ? 1 : 2));
      return;
    case RSP_SERIAL: {
      if (len < 16)
        return;
      std::string s;
      if (TWCController::ascii_field_(f + 4, 11, s)) {
        if (!this->have_serial_)
          ESP_LOGI(TAG, "[%u] TWC %04X serial %s", this->index_ + 1, this->slave_id_, s.c_str());
        this->have_serial_ = true;
        if (this->serial_text_ != nullptr && this->serial_text_->state != s)
          this->serial_text_->publish_state(s);
      }
      return;
    }
    case RSP_FW: {
      char buf[20];
      snprintf(buf, sizeof(buf), "%u.%u.%u.%u", f[4], f[5], f[6], f[7]);
      this->have_firmware_ = true;
      if (this->firmware_text_ != nullptr && this->firmware_text_->state != buf)
        this->firmware_text_->publish_state(buf);
      return;
    }
    case RSP_FW_OLD: {
      // Old firmware reply carries no TWC ID: version at bytes 2..4.
      if (this->have_firmware_)
        return;
      char buf[16];
      snprintf(buf, sizeof(buf), "%u.%u.%u", f[2], f[3], f[4]);
      this->have_firmware_ = true;
      if (this->firmware_text_ != nullptr)
        this->firmware_text_->publish_state(buf);
      return;
    }
    default:
      return;
  }
}

void TWCCharger::handle_heartbeat_(const uint8_t *f, size_t len) {
  auto *p = this->parent_;
  this->last_slave_hb_ms_ = millis();
  this->set_online_(true);

  uint8_t state = f[6];
  float offered = be16(f + 7) / 100.0f;
  float actual = be16(f + 9) / 100.0f;

  if (state != this->slave_state_) {
    ESP_LOGD(TAG, "[%u] TWC state %02X (%s), offered %.2f A, actual %.2f A", this->index_ + 1, state,
             TWCController::state_to_text_(state), offered, actual);
    this->slave_state_ = state;
    if (this->state_sensor_ != nullptr)
      this->state_sensor_->publish_state(state);
    if (this->state_text_ != nullptr)
      this->state_text_->publish_state(TWCController::state_to_text_(state));

    // shortcut: session = TWC state leaves 00 until it returns to 00; a full car that goes 03->00 splits it.
    if (state != 0x00 && !this->session_active_) {
      this->session_active_ = true;
      this->session_start_kwh_ = this->total_kwh_;  // may still be unknown right after boot; set on first meter
      this->session_wh_ = 0;
      this->session_wh_published_ = 0;
      this->auto_kind_ = this->cand_kind_ = CAR_UNKNOWN;  // new car: detect again
      this->cand_count_ = 0;
      this->q_phase_ = (p->grid_type_ == GRID_IT3 && p->circuit_power_ != nullptr) ? Q_WAIT_START : Q_OFF;
      this->q_tries_ = this->q_count_ = 0;
      this->q_kind_ = CAR_UNKNOWN;
      this->q_next_ms_ = 0;
      if (this->session_energy_sensor_ != nullptr)
        this->session_energy_sensor_->publish_state(0);
      ESP_LOGI(TAG, "[%u] Session start (counter %u)", this->index_ + 1, (unsigned) this->total_kwh_);
    } else if (state == 0x00 && this->session_active_) {
      this->session_active_ = false;
      if (this->lifetime_kwh_ != this->lifetime_saved_kwh_) {
        this->lifetime_saved_kwh_ = this->lifetime_kwh_;
        this->lifetime_pref_.save(&this->lifetime_kwh_);
      }
      ESP_LOGI(TAG, "[%u] Session end (counter %u)", this->index_ + 1, (unsigned) this->total_kwh_);
    }
  }
  if (std::fabs(offered - this->offered_amps_) >= 0.01f) {
    this->offered_amps_ = offered;
    if (this->offered_current_sensor_ != nullptr)
      this->offered_current_sensor_->publish_state(offered);
  }
  if (std::fabs(actual - this->actual_amps_) >= 0.1f || (actual == 0 && this->actual_amps_ != 0)) {
    this->actual_amps_ = actual;
    if (this->actual_current_sensor_ != nullptr)
      this->actual_current_sensor_->publish_state(actual);
    if (this->charging_bs_ != nullptr)
      this->charging_bs_->publish_state(actual >= p->phase_threshold_);
    this->publish_phase_data_();
  }

  uint32_t now = millis();
  // Full car: running but drawing nothing (the power split gives its share away after 2 min).
  if (actual >= p->phase_threshold_ || !this->running_)
    this->idle_since_ms_ = 0;
  else if (this->idle_since_ms_ == 0)
    this->idle_since_ms_ = now;

  // L1 check on IT 3-phase, for S/X and for cars without a useful VIN: with 2 sub-chargers (N-L1, N-L2)
  // the TWC sees only part of the current on L1. L1 < 75 % of offered for 60 s -> 2 sub-chargers;
  // L1 > 90 % of offered -> S/X: 3 sub-chargers, other car: not 2-sub (meter / default decides).
  // A car that limits its own current also gives low L1, so with a circuit meter the unknown car's type
  // is left to the meter checks (they measure it); the L1 check then only runs for S/X.
  bool unknown = this->car_kind_ == CAR_UNKNOWN;
  bool l1_unknown = unknown && p->circuit_power_ == nullptr;
  if (p->grid_type_ == GRID_IT3 && (this->is_sx_ || l1_unknown) && actual >= p->phase_threshold_ && offered > 0) {
    CarKind &target = unknown ? this->auto_kind_ : this->car_kind_;
    CarKind k = target;
    if (actual < 0.75f * offered) {
      if (this->sx_low_since_ms_ == 0)
        this->sx_low_since_ms_ = now;
      else if (now - this->sx_low_since_ms_ >= 60000)
        k = CAR_2_SUB;
    } else {
      this->sx_low_since_ms_ = 0;
      if (actual > 0.9f * offered && (!unknown || k == CAR_2_SUB))
        k = unknown ? CAR_UNKNOWN : CAR_3_SUB;
    }
    if (k != target) {
      ESP_LOGI(TAG, "[%u] L1 check: %s (L1 %.1f A, offered %.1f A)", this->index_ + 1,
               k == CAR_2_SUB ? "2 sub-chargers" : (k == CAR_3_SUB ? "3 sub-chargers" : "not 2 sub-chargers"), actual,
               offered);
      target = k;
      this->publish_phase_data_();
    }
  }
  if (p->grid_type_ == GRID_IT3 && p->circuit_power_ != nullptr) {
    this->meter_detect_(now, actual);
    this->quick_check_(now, actual);
  }

  // Integrate our own power once per heartbeat. Session energy on IT 3-phase: the TWC kWh counter only
  // counts V1 * I1 there (measured: counter ~2.4 kW vs real sqrt3 * 242 V * 10.3 A = 4.3 kW).
  if (this->session_active_ && this->last_integrate_ms_ != 0 && !std::isnan(this->last_power_w_)) {
    // At most 5 s per step: after a gap (TWC offline) the old power must not count for the whole gap.
    uint32_t dt = std::min<uint32_t>(now - this->last_integrate_ms_, 5000);
    float wh = this->last_power_w_ * (float) dt / 3.6e6f;
    this->session_wh_ += wh;
    if (p->grid_type_ == GRID_IT3)
      this->add_energy_(wh / 1000.0);
  }
  this->last_integrate_ms_ = now;
  if (this->session_active_ && p->grid_type_ == GRID_IT3 && this->session_energy_sensor_ != nullptr &&
      this->session_wh_ - this->session_wh_published_ >= 10.0f) {
    this->session_wh_published_ = this->session_wh_;
    this->session_energy_sensor_->publish_state(this->session_wh_ / 1000.0f);
  }
}

void TWCCharger::handle_meter_(const uint8_t *f, size_t len) {
  auto *p = this->parent_;
  uint32_t kwh = be32(f + 4);
  for (int i = 0; i < 3; i++) {
    this->phase_v_[i] = be16(f + 8 + 2 * i);
    // 20-byte frame has amps*2 per phase; a 16-byte variant (no currents) exists on some units.
    this->phase_a_[i] = len >= 18 ? f[14 + i] / 2.0f : 0;
  }
  this->meter_valid_ = true;
  if (this->phase_v_[0] > 150)  // slow average of L1 for the power split (meter every ~2 s -> ~3 min)
    this->v_avg_ = this->v_avg_ > 150 ? this->v_avg_ + 0.01f * (this->phase_v_[0] - this->v_avg_) : this->phase_v_[0];
  if (p->grid_type_ != GRID_TN3) {
    // Only TN 3-phase has real L2/L3. On 1-phase and IT 3-phase (TWC N = house L3) they are meaningless.
    this->phase_v_[1] = this->phase_v_[2] = this->phase_a_[1] = this->phase_a_[2] = 0;
  }

  if (kwh != this->total_kwh_) {
    // Lifetime off IT 3-phase: TWC counter steps; the first value after boot and jumps (> 5 kWh) don't count.
    if (p->grid_type_ != GRID_IT3 && this->total_kwh_ != 0xFFFFFFFF && kwh > this->total_kwh_ &&
        kwh - this->total_kwh_ <= 5)
      this->add_energy_(kwh - this->total_kwh_);
    // Absolute counter is wrong on some TWCs; only its increments are trusted (session energy, power fallback).
    if (this->session_active_) {
      if (this->session_start_kwh_ == 0xFFFFFFFF)
        this->session_start_kwh_ = kwh;
      // IT 3-phase: the counter only sees V1 * I1; session energy comes from integration (heartbeat).
      if (this->session_energy_sensor_ != nullptr && p->grid_type_ != GRID_IT3)
        this->session_energy_sensor_->publish_state(kwh - this->session_start_kwh_);
    }
    this->total_kwh_ = kwh;
    if (this->total_kwh_sensor_ != nullptr)
      this->total_kwh_sensor_->publish_state(kwh);
  }
  for (int i = 0; i < 3; i++) {
    if (this->voltage_sensor_[i] != nullptr && this->voltage_sensor_[i]->state != this->phase_v_[i])
      this->voltage_sensor_[i]->publish_state(this->phase_v_[i]);
    // Meter current is 0.5 A steps. Off TN only L1 is measured, so show the heartbeat's 0.01 A value instead.
    float a = (i == 0 && p->grid_type_ != GRID_TN3 && this->actual_amps_ >= 0) ? this->actual_amps_ : this->phase_a_[i];
    if (this->phase_current_sensor_[i] != nullptr && this->phase_current_sensor_[i]->state != a)
      this->phase_current_sensor_[i]->publish_state(a);
  }
  // Raw frame at DEBUG when it changes: lets us see what the TWC really reports per phase (e.g. on IT).
  size_t n = std::min(len, sizeof(this->last_meter_raw_));
  if (memcmp(this->last_meter_raw_, f, n) != 0) {
    memcpy(this->last_meter_raw_, f, n);
    ESP_LOGD(TAG, "[%u] Meter raw %s: %u kWh, %.0f/%.0f/%.0f V, %.1f/%.1f/%.1f A", this->index_ + 1,
             format_hex_pretty(f, n).c_str(), (unsigned) kwh, this->phase_v_[0], this->phase_v_[1],
             this->phase_v_[2], this->phase_a_[0], this->phase_a_[1], this->phase_a_[2]);
  }
  this->publish_phase_data_();
}

void TWCCharger::handle_vin_(const uint8_t *f, uint8_t part) {
  size_t n = (part == 2) ? 3 : 7;
  memcpy(this->vin_ + part * 7, f + 4, n);
  this->vin_parts_ |= (1 << part);
  if (this->vin_parts_ != 0x07)
    return;
  this->vin_parts_ = 0;

  std::string vin;
  bool empty = true;
  for (uint8_t b : this->vin_)
    if (b != 0)
      empty = false;
  if (empty) {
    vin = "0";  // no car, non-Tesla car, or DIP 2 (legacy) mode
  } else if (!TWCController::ascii_field_(this->vin_, 17, vin) || vin.size() != 17) {
    ESP_LOGW(TAG, "[%u] Discarding corrupt VIN", this->index_ + 1);
    return;
  }
  // Seen in the field: chars 4-7 of part 1 sometimes hold part 2's bytes, fixed on the
  // next read ~14 s later. So a new VIN must be read the same twice in a row.
  if (vin != this->vin_published_ && vin != this->vin_candidate_) {
    this->vin_candidate_ = vin;
    ESP_LOGD(TAG, "[%u] VIN %s read once, waiting for a second read", this->index_ + 1, vin.c_str());
    return;
  }
  if (vin != this->vin_published_) {
    this->vin_published_ = vin;
    if (this->vin_text_ != nullptr)
      this->vin_text_->publish_state(vin);
    std::string info;
    this->car_kind_ = TWCController::decode_vin(vin, info);  // S/X: year is only the first guess, see heartbeat
    this->is_sx_ = this->car_kind_ != CAR_UNKNOWN && (vin[3] == 'S' || vin[3] == 'X');
    this->sx_low_since_ms_ = 0;
    ESP_LOGI(TAG, "[%u] Car: %s (IT 3-phase sub-chargers: %u)", this->index_ + 1, info.c_str(), this->car_kind_);
    if (this->car_info_text_ != nullptr)
      this->car_info_text_->publish_state(info);
    this->publish_phase_data_();
  }
}

CarKind TWCController::decode_vin(const std::string &vin, std::string &info) {
  if (vin.size() != 17) {
    info = "No car or no VIN";
    return CAR_UNKNOWN;
  }
  static const char *const TESLA_WMI[] = {"5YJ", "7SA", "7G2", "LRW", "XP7", "SFZ"};
  bool tesla = false;
  for (const char *w : TESLA_WMI)
    tesla |= vin.compare(0, 3, w) == 0;
  if (!tesla) {
    info = "Non-Tesla (" + vin.substr(0, 3) + ")";
    return CAR_UNKNOWN;
  }
  // Position 10 = model year; I, O, Q, U, Z and 0 are never used.
  static const char YEARS[] = "ABCDEFGHJKLMNPRSTVWXY123456789";  // A=2010
  const char *y = strchr(YEARS, vin[9]);
  int year = y ? 2010 + (y - YEARS) : 0;
  const char *model;
  CarKind kind = CAR_UNKNOWN;
  switch (vin[3]) {
    case '3': model = "Model 3"; kind = CAR_3_SUB; break;
    case 'Y': model = "Model Y"; kind = CAR_3_SUB; break;
    // S/X before 2021 use only 2 of 3 sub-chargers on IT (N-L1, N-L2).
    case 'S': model = "Model S"; kind = year >= 2021 ? CAR_3_SUB : CAR_2_SUB; break;
    case 'X': model = "Model X"; kind = year >= 2021 ? CAR_3_SUB : CAR_2_SUB; break;
    case 'R': model = "Roadster"; break;
    case 'T': model = "Semi"; break;
    case 'C': model = "Cybertruck"; break;
    default: model = "(unknown model)"; break;
  }
  info = std::string("Tesla ") + model + (year ? " " + std::to_string(year) : "");
  return kind;
}

// ---------------- car type ----------------

CarKind TWCCharger::kind_() const {
  if (this->manual_kind_ != CAR_UNKNOWN)
    return this->manual_kind_;
  if (this->car_kind_ != CAR_UNKNOWN)
    return this->car_kind_;
  return this->auto_kind_;
}

// Car type from the circuit power meter (IT 3-phase, car without a useful VIN). When this charger's current
// steps by >= 3 A, compare the circuit power before and after: W per amp ~ V -> 1-phase, ~ sqrt3 * V -> 3-phase.
// Thrown away when: the meter or the current is not steady, another charger stepped at the same time, or the
// result is not within +-20 % of one of the two. The same result twice in a row changes the type.
void TWCCharger::meter_detect_(uint32_t now, float actual) {
  auto *p = this->parent_;
  if (now - this->ring_ms_ >= 5000) {
    this->ring_ms_ = now;
    this->ring_[this->ring_pos_] = std::max(0.0f, actual);
    this->ring_pos_ = (this->ring_pos_ + 1) % RING;
    if (this->ring_n_ < RING)
      this->ring_n_++;
  }
  auto ago = [this](int k) { return this->ring_ago_(k); };  // k * 5 s ago
  if (this->q_phase_ >= Q_MEAS_START) {  // the quick check owns the current steps now
    this->step_pending_ = false;
    return;
  }

  if (!this->step_pending_) {
    if (this->ring_n_ >= 4 && std::fabs(ago(0) - ago(3)) >= 3.0f && now > 140000) {
      this->step_pending_ = true;
      this->step_trig_ms_ = now;
      this->step_t0_ms_ = now - 15000;  // the current started to move about here
      this->i_before_ = ago(3);
      this->last_step_ms_ = now;
    }
    return;
  }
  uint32_t since = now - this->step_trig_ms_;
  if (since > 400000) {
    this->step_pending_ = false;
    ESP_LOGD(TAG, "[%u] Meter check skipped: current not steady or too few meter readings", this->index_ + 1);
    return;
  }
  if (since < 90000)
    return;  // let the car settle
  float mn = 1e9f, mx = -1e9f, sum = 0;
  for (int k = 0; k < 12; k++) {  // last 60 s
    float v = ago(k);
    mn = std::min(mn, v), mx = std::max(mx, v), sum += v;
  }
  if (mx - mn > 1.0f)
    return;  // still moving, wait
  // before: 2 min up to the step (the last older reading counts); after: from step + 30 s, real readings only.
  // A slow meter (e.g. every 60 s) just makes us wait for 2 readings after the step.
  float before, after, sb, sa;
  if (!p->circuit_window_(this->step_t0_ms_ - 120000, this->step_t0_ms_, true, before, sb) ||
      !p->circuit_window_(this->step_trig_ms_ + 30000, now, false, after, sa))
    return;  // wait for more meter readings
  this->step_pending_ = false;
  float i_after = sum / 12, d_i = i_after - this->i_before_;
  if (std::fabs(d_i) < 3.0f || this->car_kind_ != CAR_UNKNOWN || this->manual_kind_ != CAR_UNKNOWN)
    return;
  for (auto *c : p->chargers_)
    if (c != this && c->last_step_ms_ != 0 && now - c->last_step_ms_ < 400000) {
      ESP_LOGD(TAG, "[%u] Meter check skipped: charger %u changed too", this->index_ + 1, c->index_ + 1);
      return;
    }
  float v = this->v_avg_ > 150 ? this->v_avg_ : p->nominal_voltage_;
  float r = (after - before) / (v * d_i);
  CarKind k = (r >= 0.8f && r <= 1.2f) ? CAR_1_PHASE : (r >= 1.39f && r <= 2.08f) ? CAR_3_SUB : CAR_UNKNOWN;
  ESP_LOGI(TAG, "[%u] Meter check: %.0f -> %.0f W (spread %.0f/%.0f), %.1f -> %.1f A, %.2f x V per A -> %s",
           this->index_ + 1, before, after, sb, sa, this->i_before_, i_after, r,
           sb > 400 || sa > 400 ? "meter not steady, skipped"
                                : (k == CAR_1_PHASE ? "1-phase" : (k == CAR_3_SUB ? "3-phase" : "no match, skipped")));
  if (sb > 400 || sa > 400 || k == CAR_UNKNOWN)
    return;
  if (k == this->cand_kind_) {
    this->cand_count_++;
  } else {
    this->cand_kind_ = k;
    this->cand_count_ = 1;
  }
  if (this->cand_count_ >= 2 && this->auto_kind_ != k && this->auto_kind_ != CAR_2_SUB) {
    ESP_LOGI(TAG, "[%u] Car type from meter: %s", this->index_ + 1, k == CAR_1_PHASE ? "1-phase" : "3-phase");
    this->auto_kind_ = k;
    this->publish_phase_data_();
  }
}

float TWCCharger::ring_spread_(int n, float *mean) const {
  if (this->ring_n_ < n)
    return 1e9f;
  float mn = 1e9f, mx = -1e9f, sum = 0;
  for (int k = 0; k < n; k++) {
    float v = this->ring_ago_(k);
    mn = std::min(mn, v), mx = std::max(mx, v), sum += v;
  }
  if (mean != nullptr)
    *mean = sum / n;
  return mx - mn;
}

bool TWCCharger::quick_wanted_() const {
  return this->session_active_ && this->running_ && this->car_kind_ == CAR_UNKNOWN &&
         this->manual_kind_ == CAR_UNKNOWN && this->auto_kind_ == CAR_UNKNOWN;
}

void TWCCharger::quick_end_(const char *why) {
  if (this->probe_delta_ != 0) {
    this->probe_delta_ = 0;
    this->parent_->update_limits_();
  }
  if (this->q_phase_ >= Q_MEAS_START)
    ESP_LOGI(TAG, "[%u] Type check end: %s", this->index_ + 1, why);
  this->q_phase_ = Q_OFF;
}

// Quick car type check, IT 3-phase, car without VIN and car type Auto (circuit meter every ~5 s in HA).
// Measurement 1: charge start (0 -> I). Measurement 2/3: the ESP steps 4 A (down if it can, else up within
// the fuse) and back. Each: circuit mean 30 s before the step vs 10-40 s after, with steady current.
// Two equal results set the type. Disturbed (> 400 W jump, other charger moving): retry in 5 min, 3 tries.
void TWCCharger::quick_check_(uint32_t now, float actual) {
  auto *p = this->parent_;
  float thr = p->phase_threshold_;
  float prev = this->q_prev_actual_;
  this->q_prev_actual_ = actual;
  if (this->q_phase_ != Q_OFF && !this->session_active_)
    this->quick_end_("unplugged");
  else if (this->q_phase_ >= Q_MEAS_START && !this->quick_wanted_())
    this->quick_end_(this->auto_kind_ != CAR_UNKNOWN ? "type known" : "VIN / car type / charging stopped");

  switch (this->q_phase_) {
    case Q_OFF:
      break;
    case Q_WAIT_START:
      if (!this->quick_wanted_() || actual < thr)
        break;
      if (prev >= 0 && prev < thr) {
        this->q_phase_ = Q_MEAS_START;
        this->q_step_ms_ = now;
        this->q_ib_ = std::max(0.0f, prev);
        ESP_LOGI(TAG, "[%u] Type check: charge start", this->index_ + 1);
      } else {
        this->q_phase_ = Q_PROBE_WAIT;  // already charging (e.g. after a reboot): go straight to the step
      }
      break;
    case Q_PROBE_WAIT: {
      if ((int32_t) (now - this->q_next_ms_) < 0)
        break;
      float ib;
      if (this->ring_spread_(6, &ib) > 1.0f || ib < thr)
        break;  // wait for 30 s of steady current
      bool busy = false;
      for (auto *c : p->chargers_)
        if (c != this && (c->q_phase_ == Q_PROBE || c->q_phase_ == Q_BACK))
          busy = true;  // one step at a time on the circuit
      if (busy)
        break;
      float before = this->assigned_amps_;
      this->probe_delta_ = before >= p->min_current_ + 4 ? -4 : 4;
      p->update_limits_();
      if (std::fabs(this->assigned_amps_ - before) < 3.0f) {
        this->quick_end_("no room for a 4 A step");
        break;
      }
      ESP_LOGI(TAG, "[%u] Type check: step %.0f -> %.0f A", this->index_ + 1, before, this->assigned_amps_);
      this->q_phase_ = Q_PROBE;
      this->q_step_ms_ = now;
      this->q_ib_ = ib;
      break;
    }
    case Q_MEAS_START:
    case Q_PROBE:
    case Q_BACK: {
      uint32_t since = now - this->q_step_ms_;
      if (since < 40000)
        break;
      float ia;
      bool steady = this->ring_spread_(6, &ia) <= 1.0f;
      if (!steady && since < 180000)
        break;  // car still moving: wait up to 3 min
      QPhase ph = this->q_phase_;
      const char *what = ph == Q_MEAS_START ? "start" : (ph == Q_PROBE ? "step" : "back");
      CarKind k = CAR_UNKNOWN;
      if (steady)
        k = this->quick_eval_(now, ia, what);
      else
        ESP_LOGI(TAG, "[%u] Type check %s: current not steady, skipped", this->index_ + 1, what);
      if (k != CAR_UNKNOWN) {
        this->q_count_ = k == this->q_kind_ ? this->q_count_ + 1 : 1;
        this->q_kind_ = k;
        if (this->q_count_ >= 2) {
          ESP_LOGI(TAG, "[%u] Car type from type check: %s", this->index_ + 1, k == CAR_1_PHASE ? "1-phase" : "3-phase");
          this->auto_kind_ = k;
          this->quick_end_("done");
          this->publish_phase_data_();
          break;
        }
      }
      if (ph == Q_PROBE) {  // step back: the next measurement
        this->probe_delta_ = 0;
        p->update_limits_();
        this->q_phase_ = Q_BACK;
        this->q_step_ms_ = now;
        if (steady)
          this->q_ib_ = ia;
      } else if (ph == Q_BACK && ++this->q_tries_ >= 3) {
        this->quick_end_("no result after 3 tries");
      } else {
        this->q_phase_ = Q_PROBE_WAIT;
        this->q_next_ms_ = ph == Q_BACK ? now + 300000 : now;
      }
      break;
    }
  }
  bool on = this->q_phase_ == Q_MEAS_START || this->q_phase_ == Q_PROBE || this->q_phase_ == Q_BACK ||
            (this->q_phase_ == Q_PROBE_WAIT && this->q_tries_ == 0);
  if (on != this->q_on_) {
    this->q_on_ = on;
    if (this->type_check_bs_ != nullptr)
      this->type_check_bs_->publish_state(on);
  }
}

// One measurement: circuit power 30 s before the step vs the last 30 s (from step + 10 s), per amp changed.
CarKind TWCCharger::quick_eval_(uint32_t now, float ia, const char *what) {
  auto *p = this->parent_;
  float before = 0, after = 0, sb = 0, sa = 0;
  bool ok = p->circuit_window_(this->q_step_ms_ - 30000, this->q_step_ms_, true, before, sb) &&
            p->circuit_window_(std::max(this->q_step_ms_ + 10000, now - 30000), now, false, after, sa);
  float d_i = ia - this->q_ib_;
  bool other = false;
  for (auto *c : p->chargers_)
    if (c != this && c->ring_spread_(10, nullptr) > 1.5f && c->ring_n_ >= 10)
      other = true;
  if (!ok || std::fabs(d_i) < 2.5f) {
    ESP_LOGI(TAG, "[%u] Type check %s: %s, skipped", this->index_ + 1, what,
             ok ? "current step too small" : "too few meter readings");
    return CAR_UNKNOWN;
  }
  float v = this->v_avg_ > 150 ? this->v_avg_ : p->nominal_voltage_;
  float r = (after - before) / (v * d_i);
  CarKind k = (r >= 0.8f && r <= 1.2f) ? CAR_1_PHASE : (r >= 1.39f && r <= 2.08f) ? CAR_3_SUB : CAR_UNKNOWN;
  bool noisy = sb > 400 || sa > 400 || other;
  ESP_LOGI(TAG, "[%u] Type check %s: %.0f -> %.0f W (spread %.0f/%.0f), %.1f -> %.1f A, %.2f x V per A -> %s",
           this->index_ + 1, what, before, after, sb, sa, this->q_ib_, ia, r,
           other ? "other charger moving, skipped"
                 : noisy ? "meter not steady, skipped"
                         : (k == CAR_1_PHASE ? "1-phase" : (k == CAR_3_SUB ? "3-phase" : "no match, skipped")));
  return noisy ? CAR_UNKNOWN : k;
}

// ---------------- phase / power ----------------

// Watts per offered amp, for the power split. Measured L1 voltage, slowly averaged (~3 min) so the
// amps don't flap on small voltage changes; nominal voltage until the meter has reported.
float TWCCharger::watts_per_amp_() const {
  auto *p = this->parent_;
  float v = this->v_avg_ > 150 ? this->v_avg_ : p->nominal_voltage_;
  if (p->grid_type_ == GRID_SINGLE)
    return v;
  if (p->grid_type_ == GRID_TN3) {
    uint8_t ph = 0;
    if (this->meter_valid_ && this->actual_amps_ >= p->phase_threshold_)
      for (float a : this->phase_a_)
        ph += a >= p->phase_threshold_;
    return v * (ph ? ph : 3);  // not charging yet: assume 3-phase (safe side)
  }
  // IT 3-phase: 1-phase V, 2 sub-chargers 1.15 * V; 3 sub-chargers and unknown cars sqrt(3) * V (safe side).
  CarKind k = this->kind_();
  return k == CAR_1_PHASE ? v : (k == CAR_2_SUB ? 1.15f * v : std::sqrt(3.0f) * v);
}

void TWCCharger::publish_phase_data_() {
  auto *p = this->parent_;
  float power = 0;
  uint8_t active = 0;
  const char *mode;
  float charging_amps = std::max(0.0f, this->actual_amps_);  // "Charging Current": what the car draws

  if (p->grid_type_ != GRID_TN3) {
    // 1-phase and IT 3-phase: only L1 V is real. Use the precise heartbeat current.
    float v = this->phase_v_[0] > 150 ? this->phase_v_[0] : p->nominal_voltage_;
    float i = this->actual_amps_;
    if (i < p->phase_threshold_) {
      mode = "Idle";
    } else if (p->grid_type_ == GRID_SINGLE) {
      power = v * i;  // regardless of car
      active = 1;
      mode = "1-phase";
    } else {
      // IT 3-phase. Car type: select (manual) > VIN > detected (meter / L1 check) > assumed 3-phase.
      CarKind k = this->kind_();
      bool manual = this->manual_kind_ != CAR_UNKNOWN, from_vin = !manual && this->car_kind_ != CAR_UNKNOWN;
      if (k == CAR_1_PHASE) {
        power = v * i;  // TWC L1-N = house L1-L3
        active = 1;
        mode = manual ? "1-phase IT (manual)" : "1-phase IT (meter)";
      } else if (k == CAR_2_SUB) {
        // N-L1 + N-L2: L1 only sees part of it, the car draws the offered current (measured:
        // 6 A offered -> 2.8 A on L1, ~1662 W in car). P = 1.15 * V * offered.
        i = std::max(0.0f, this->offered_amps_);
        charging_amps = i;
        power = 1.15f * v * i;
        active = 2;
        mode = manual ? "3-phase IT (2 sub-chargers, manual)"
                      : (from_vin ? "3-phase IT (2 sub-chargers)" : "3-phase IT (2 sub-chargers, detected)");
      } else {
        // 3 sub-chargers, or unknown car treated as 3-phase (measured: ~ 241 V * 10.3 A * 1.73).
        power = std::sqrt(3.0f) * v * i;  // matched the car app (Model Y)
        active = 3;
        mode = manual ? "3-phase IT (manual)"
                      : (from_vin ? "3-phase IT (3 sub-chargers)"
                                  : (k == CAR_3_SUB ? "3-phase IT (meter)" : "3-phase IT (assumed)"));
      }
    }
  } else if (!this->meter_valid_) {
    // Old firmware: no FDEB meter. Assume single phase.
    active = this->actual_amps_ >= p->phase_threshold_ ? 1 : 0;
    power = this->actual_amps_ * p->nominal_voltage_;
    mode = active ? "1-phase (no meter data)" : "Idle";
  } else {
    // TN: every phase measured, P = sum of V*I.
    for (int i = 0; i < 3; i++) {
      if (this->phase_a_[i] >= p->phase_threshold_) {
        active++;
        float v = this->phase_v_[i] > 50 ? this->phase_v_[i] : p->nominal_voltage_;
        power += v * this->phase_a_[i];
      }
    }
    static const char *const tn[] = {"Idle", "1-phase", "2-phase", "3-phase"};
    mode = tn[active];
  }

  this->last_power_w_ = power;
  if (this->current_sensor_ != nullptr && !(std::fabs(this->current_sensor_->state - charging_amps) < 0.1f))
    this->current_sensor_->publish_state(charging_amps);
  if (this->active_phases_sensor_ != nullptr && this->active_phases_sensor_->state != active)
    this->active_phases_sensor_->publish_state(active);
  if (this->power_sensor_ != nullptr) {
    float old = this->power_sensor_->state;
    if (std::isnan(old) != std::isnan(power) || std::fabs(old - power) >= 10.0f)
      this->power_sensor_->publish_state(power);
  }
  if (this->charge_mode_text_ != nullptr && this->charge_mode_text_->state != mode)
    this->charge_mode_text_->publish_state(mode);
}

void TWCCharger::set_online_(bool online) {
  if (this->online_bs_ != nullptr && (online != this->online_ || !this->online_bs_->has_state()))
    this->online_bs_->publish_state(online);
  this->online_ = online;
}

void TWCCharger::log_stats_() {
  if (!this->have_slave_)
    return;
  ESP_LOGD(TAG, "[%u] TWC %04X %s, state %02X, assigned %.0f A, offered %.1f A, actual %.1f A", this->index_ + 1,
           this->slave_id_, this->online_ ? "online" : "offline", this->slave_state_, this->assigned_amps_,
           this->offered_amps_, this->actual_amps_);
}

// ---------------- TX ----------------

bool TWCController::bus_free_(uint32_t now) const {
  return !(this->rx_in_frame_ && this->rx_len_ > 0) && (now - this->last_rx_byte_ms_ >= BUS_IDLE_MS) &&
         (now - this->last_tx_ms_ >= MIN_TX_GAP_MS);
}

std::vector<uint8_t> TWCController::make_frame_(uint16_t cmd, uint16_t dest, uint8_t fill_to) {
  std::vector<uint8_t> b;
  b.reserve(20);
  b.push_back(cmd >> 8);
  b.push_back(cmd & 0xFF);
  b.push_back(this->twcid_ >> 8);
  b.push_back(this->twcid_ & 0xFF);
  if (dest != 0) {
    b.push_back(dest >> 8);
    b.push_back(dest & 0xFF);
  }
  while (b.size() < fill_to)
    b.push_back(0);
  return b;
}

void TWCController::send_frame_(std::vector<uint8_t> body) {
  // out[] below holds 2 SLIP_END + trailer + every byte (and the checksum) escaped to 2 bytes.
  if (body.size() < 2 || 2 * (body.size() + 1) + 3 > 48) {
    ESP_LOGE(TAG, "Refusing to send frame of %u bytes", (unsigned) body.size());
    return;
  }
  uint16_t cmd = be16(body.data());
  for (uint16_t bad : CMD_FORBIDDEN) {
    if (cmd == bad) {
      ESP_LOGE(TAG, "Refusing to send dangerous command %04X", cmd);
      return;
    }
  }
  // Checksum: sum of all body bytes except the first, computed over the unescaped body.
  uint8_t sum = 0;
  for (size_t i = 1; i < body.size(); i++)
    sum += body[i];
  body.push_back(sum);

  uint8_t out[48];
  size_t n = 0;
  out[n++] = SLIP_END;
  for (uint8_t c : body) {
    if (c == SLIP_END) {
      out[n++] = SLIP_ESC;
      out[n++] = SLIP_ESC_END;
    } else if (c == SLIP_ESC) {
      out[n++] = SLIP_ESC;
      out[n++] = SLIP_ESC_ESC;
    } else {
      out[n++] = c;
    }
  }
  out[n++] = SLIP_END;
  out[n++] = FRAME_TRAILER;

  ESP_LOGV(TAG, "TX %s", format_hex_pretty(body.data(), body.size()).c_str());
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);
  this->write_array(out, n);
  // Wait until the last stop bit is out before releasing the bus.
  // ESPHome >= 2025 returns a result; older versions return void.
  if (!flush_ok_(this, 0))
    this->flush_fail_++;
  this->tx_frames_++;
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  this->last_tx_ms_ = millis();
}

void TWCController::queue_command_(TWCCharger *c, uint16_t cmd, uint8_t repeat) {
  this->cmd_queue_.push_back(PendingCommand{c, cmd, repeat});
}

void TWCController::drop_commands_(TWCCharger *c) {
  this->cmd_queue_.erase(std::remove_if(this->cmd_queue_.begin(), this->cmd_queue_.end(),
                                        [c](const PendingCommand &pc) { return pc.charger == c; }),
                         this->cmd_queue_.end());
}

std::vector<uint8_t> TWCCharger::heartbeat_frame_() {
  uint8_t fill = this->protocol_ == 1 ? 13 : 15;
  auto b = this->parent_->make_frame_(CMD_MASTER_HEARTBEAT, this->slave_id_, 6);
  float a = this->running_ ? this->assigned_amps_ : 0;
  if (this->slave_max_amps_ > 0 && a > this->slave_max_amps_)
    a = this->slave_max_amps_;
  uint16_t ca = (uint16_t) lroundf(std::max(0.0f, a) * 100.0f);
  // Send "set limit" (05 proto 1 / 09 proto 2) only when needed, else 00 so the TWC reports its real
  // state (01 = charging) instead of echoing 09. Needed = limit changed (last_limit_sent_ms_ == 0),
  // TWC applies another limit, TWC asks for one (04/08), or 30 s safety re-assert (6 A stuck fix).
  uint32_t now = millis();
  bool mismatch = std::fabs(this->offered_amps_ * 100.0f - ca) > 50.0f;
  bool asks = this->slave_state_ == 0x04 || this->slave_state_ == 0x08;
  if (this->last_limit_sent_ms_ == 0 || mismatch || asks || now - this->last_limit_sent_ms_ >= 30000) {
    this->last_limit_sent_ms_ = now;
    b.push_back(this->protocol_ == 1 ? 0x05 : 0x09);
    b.push_back(ca >> 8);
    b.push_back(ca & 0xFF);
  } else {
    b.push_back(0x00);  // no change
    b.push_back(0x00);
    b.push_back(0x00);
  }
  b.push_back(0x00);  // master's own plug not inserted
  while (b.size() < fill)
    b.push_back(0);
  return b;
}

uint16_t TWCCharger::next_poll_cmd_() {
  static const uint16_t SEQ[] = {CMD_GET_METER, CMD_GET_VIN1, CMD_GET_METER, CMD_GET_VIN2,   CMD_GET_METER,
                                 CMD_GET_VIN3,  CMD_GET_METER, CMD_GET_FW,   CMD_GET_METER,  CMD_GET_SERIAL,
                                 CMD_GET_METER, CMD_GET_PLUG,  CMD_GET_METER, CMD_GET_MODEL};
  uint16_t cmd = SEQ[this->poll_index_];
  this->poll_index_ = (this->poll_index_ + 1) % (sizeof(SEQ) / sizeof(SEQ[0]));
  if (cmd == CMD_GET_MODEL && (this->have_model_ || this->protocol_ == 1))
    cmd = CMD_GET_METER;
  if (cmd == CMD_GET_PLUG && this->protocol_ == 1)
    cmd = CMD_GET_METER;
  if (cmd == CMD_GET_SERIAL && this->have_serial_)
    cmd = CMD_GET_METER;
  if (cmd == CMD_GET_FW && this->have_firmware_)
    cmd = CMD_GET_METER;
  if (cmd == CMD_GET_FW && this->protocol_ == 1)
    cmd = CMD_GET_FW_OLD;
  return cmd;
}

void TWCController::schedule_(uint32_t now) {
  if (now - this->last_stats_ms_ >= 60000) {
    this->last_stats_ms_ = now;
    ESP_LOGD(TAG, "Link: rx bytes %u, frames ok %u, bad %u, tx frames %u, flush fail %u", (unsigned) this->rx_bytes_,
             (unsigned) this->rx_frames_ok_, (unsigned) this->rx_frames_bad_, (unsigned) this->tx_frames_,
             (unsigned) this->flush_fail_);
    for (auto *c : this->chargers_)
      c->log_stats_();
  }

  for (auto *c : this->chargers_)
    c->supervise_(now);

  // Every 5 s: redo the power split (cars plug in, get full, sub-chargers detected) and the total power.
  if (now - this->last_power_tick_ms_ >= 5000) {
    this->last_power_tick_ms_ = now;
    if (this->power_mode_())
      this->update_limits_();
    if (this->total_power_sensor_ != nullptr) {
      float sum = 0;
      for (auto *c : this->chargers_)
        if (c->have_slave_ && !std::isnan(c->last_power_w_))
          sum += c->last_power_w_;
      if (!this->total_power_sensor_->has_state() || std::fabs(this->total_power_sensor_->state - sum) >= 10.0f)
        this->total_power_sensor_->publish_state(sum);
    }
  }

  if (!this->bus_free_(now))
    return;

  bool any = this->any_slave_();
  // 1. Startup burst: 5x linkready1, 5x linkready2 (stopped early if a TWC shows up).
  //    Later TWCs announce themselves with their own linkready every ~10 s; we answer those.
  if (!any && (this->startup_linkready1_ > 0 || this->startup_linkready2_ > 0)) {
    if (now - this->last_tx_ms_ < LINKREADY_INTERVAL_MS)
      return;
    bool first = this->startup_linkready1_ > 0;
    auto b = this->make_frame_(first ? CMD_MASTER_LINKREADY1 : CMD_MASTER_LINKREADY2, 0, 4);
    b.push_back(this->sign_);
    while (b.size() < 15)
      b.push_back(0);
    this->send_frame_(b);
    if (first)
      this->startup_linkready1_--;
    else
      this->startup_linkready2_--;
    return;
  }

  if (!any)
    return;

  // 2. Heartbeat slots: one per configured charger, so each TWC gets one every second
  //    (1 charger: every 1000 ms, 2: slots of 500 ms, 4: 250 ms). Empty slot = nothing sent.
  uint32_t slot = HEARTBEAT_INTERVAL_MS / this->chargers_.size();
  if (this->last_heartbeat_ms_ == 0 || now - this->last_heartbeat_ms_ >= slot) {
    TWCCharger *c = this->chargers_[this->hb_index_];
    this->hb_index_ = (this->hb_index_ + 1) % this->chargers_.size();
    this->last_heartbeat_ms_ = now;
    this->poll_charger_ = nullptr;
    if (c->have_slave_) {
      this->send_frame_(c->heartbeat_frame_());
      this->poll_charger_ = c;
      return;
    }
  }

  // 3. Re-assert STOP while stopped/paused and the car is still drawing current.
  for (auto *c : this->chargers_) {
    if (c->have_slave_ && !c->running_ && c->actual_amps_ >= this->phase_threshold_ &&
        now - c->last_stop_sent_ms_ >= this->stop_reassert_ms_) {
      bool queued = false;
      for (auto &pc : this->cmd_queue_)
        queued |= pc.charger == c;
      if (!queued) {
        ESP_LOGD(TAG, "[%u] Still drawing %.1f A while stopped - re-sending STOP", c->index_ + 1, c->actual_amps_);
        this->queue_command_(c, CMD_STOP_CHARGING, 1);
        c->last_stop_sent_ms_ = now;
      }
    }
  }

  // 4. Start/stop commands, one per free slot, after the heartbeat reply window.
  uint32_t reply_window = std::min<uint32_t>(250, slot / 2);
  if (!this->cmd_queue_.empty() && now - this->last_heartbeat_ms_ >= reply_window) {
    PendingCommand &pc = this->cmd_queue_.front();
    TWCCharger *c = pc.charger;
    if (c->have_slave_) {
      ESP_LOGD(TAG, "[%u] TX command %04X to %04X", c->index_ + 1, pc.cmd, c->slave_id_);
      this->send_frame_(this->make_frame_(pc.cmd, c->slave_id_, c->protocol_ == 1 ? 13 : 15));
    }
    if (--pc.repeat == 0 || !c->have_slave_)
      this->cmd_queue_.pop_front();
    return;
  }

  // 5. Data poll for the charger just heartbeated, half-way through its slot.
  TWCCharger *c = this->poll_charger_;
  if (c != nullptr && c->have_slave_ && now - this->last_heartbeat_ms_ >= slot / 2) {
    uint16_t cmd = c->next_poll_cmd_();
    this->send_frame_(this->make_frame_(cmd, cmd == CMD_GET_FW_OLD ? 0 : c->slave_id_, c->protocol_ == 1 ? 13 : 15));
    this->poll_charger_ = nullptr;  // one poll per slot
    this->last_polled_ = c;         // FD1A / FD1B replies without a TWC ID belong to it
  }
}

// ---------------- helpers ----------------

const char *TWCController::state_to_text_(uint8_t s) {
  switch (s) {
    case 0x00:
      return "Ready";
    case 0x01:
      return "Charging";
    case 0x02:
      return "Error";
    case 0x03:
      return "Plugged in, not charging";
    case 0x04:
      return "Plugged in, ready";
    case 0x05:
      return "Busy";
    case 0x06:
      return "Raising current";
    case 0x07:
      return "Lowering current";
    case 0x08:
      return "Starting to charge";
    case 0x09:
      return "Limit acknowledged";
    case 0x0A:
      return "Adjustment done";
    default:
      return "Unknown";
  }
}

bool TWCController::ascii_field_(const uint8_t *data, size_t len, std::string &out) {
  out.clear();
  for (size_t i = 0; i < len; i++) {
    if (data[i] == 0 || data[i] == 0xFF)  // FF: model string terminator seen on FD1A
      break;
    if (data[i] < 0x20 || data[i] > 0x7E)
      return false;
    out.push_back((char) data[i]);
  }
  return !out.empty();
}

}  // namespace twc_controller
}  // namespace esphome
