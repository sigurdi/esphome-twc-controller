// TWC Gen2 RS485 "fake master" for ESPHome.
// Rewrite of jnicolson/esphome-twc-controller (GPLv3) for local use.
// Single-threaded: all RX parsing and TX scheduling happens in loop().
// Drives 1-4 Wall Connectors on one bus and one main fuse (max_current). YAML: multi_charger.chargers.
// Protocol notes: docs/TWC-RS485-Protocol.md
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace twc_controller {

// Installation setting (YAML). Norwegian IT 3-phase: TWC N = house L3, TWC L3 unused; TWC meter then reports only L1.
enum GridType : uint8_t { GRID_SINGLE = 0, GRID_TN3 = 1, GRID_IT3 = 2 };
static const char *const GRID_NAMES[] = {"1-phase (TN/IT)", "TN 3-phase", "IT 3-phase"};
// How the connected car loads IT 3-phase: 1-phase (TWC L1-N), 2 or 3 sub-chargers.
enum CarKind : uint8_t { CAR_UNKNOWN = 0, CAR_1_PHASE = 1, CAR_2_SUB = 2, CAR_3_SUB = 3 };

class TWCController;
class TWCCharger;

class TWCCurrentNumber : public number::Number, public Parented<TWCCharger> {
 protected:
  void control(float value) override;
};

// Controller-wide settings (HA numbers), restored after reboot.
enum Setting : uint8_t { SET_TOTAL_AMPS = 0, SET_MAX_POWER = 1, SET_PRIORITY = 2, SET_COUNT = 3 };

class TWCSettingNumber : public number::Number, public Parented<TWCController> {
 public:
  void set_setting(uint8_t s) { this->setting_ = s; }

 protected:
  void control(float value) override;
  uint8_t setting_{0};
};

// Control mode: 0 = Current (Set Current / Total Current rule), 1 = Power (Max Power rules).
class TWCModeSelect : public select::Select, public Parented<TWCController> {
 protected:
  void control(size_t index) override;
};

// Energy offset (kWh): added to the charger's own lifetime counter (e.g. to start at a known value).
class TWCOffsetNumber : public number::Number, public Parented<TWCCharger> {
 protected:
  void control(float value) override;
};

class TWCChargingSwitch : public switch_::Switch, public Parented<TWCCharger> {
 protected:
  void write_state(bool state) override;
};

// Car type per charger: 0 Auto, 1 1-phase, 2 3-phase, 3 2 sub-chargers (IT 3-phase only).
class TWCCarTypeSelect : public select::Select, public Parented<TWCCharger> {
 protected:
  void control(size_t index) override;
};

// One Wall Connector (slave) on the bus: its link state, car, meter, session and entities.
class TWCCharger : public Parented<TWCController> {
  friend class TWCController;

 public:
  void set_pinned_id(uint16_t id) { this->pinned_id_ = id; }
  void set_initial_current(uint8_t v) { this->limit_amps_ = v; }
  void set_current_number(TWCCurrentNumber *n) { this->current_number_ = n; }
  void set_charging_switch(TWCChargingSwitch *s) { this->charging_switch_ = s; }
  void set_car_type_select(TWCCarTypeSelect *s) { this->car_type_select_ = s; }
  void on_car_type(size_t index);
  void set_energy_offset_number(TWCOffsetNumber *n) { this->offset_number_ = n; }
  void on_energy_offset(float kwh);

  // Sensors (names kept compatible with the upstream component)
  void set_current_sensor(sensor::Sensor *s) { this->current_sensor_ = s; }
  void set_max_allowable_current_sensor(sensor::Sensor *s) { this->max_allowable_current_sensor_ = s; }
  void set_assigned_current_sensor(sensor::Sensor *s) { this->assigned_current_sensor_ = s; }
  void set_total_kwh_delivered_sensor(sensor::Sensor *s) { this->total_kwh_sensor_ = s; }
  void set_phase_1_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_[0] = s; }
  void set_phase_2_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_[1] = s; }
  void set_phase_3_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_[2] = s; }
  void set_phase_1_current_sensor(sensor::Sensor *s) { this->phase_current_sensor_[0] = s; }
  void set_phase_2_current_sensor(sensor::Sensor *s) { this->phase_current_sensor_[1] = s; }
  void set_phase_3_current_sensor(sensor::Sensor *s) { this->phase_current_sensor_[2] = s; }
  void set_actual_current_sensor(sensor::Sensor *s) { this->actual_current_sensor_ = s; }
  void set_offered_current_sensor(sensor::Sensor *s) { this->offered_current_sensor_ = s; }
  void set_state_sensor(sensor::Sensor *s) { this->state_sensor_ = s; }
  void set_power_sensor(sensor::Sensor *s) { this->power_sensor_ = s; }
  void set_active_phases_sensor(sensor::Sensor *s) { this->active_phases_sensor_ = s; }
  void set_session_energy_sensor(sensor::Sensor *s) { this->session_energy_sensor_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { this->total_energy_sensor_ = s; }
  void set_car_info_text_sensor(text_sensor::TextSensor *s) { this->car_info_text_ = s; }
  void set_plug_state_text_sensor(text_sensor::TextSensor *s) { this->plug_text_ = s; }
  void set_model_text_sensor(text_sensor::TextSensor *s) { this->model_text_ = s; }
  void set_serial_text_sensor(text_sensor::TextSensor *s) { this->serial_text_ = s; }
  void set_firmware_version_text_sensor(text_sensor::TextSensor *s) { this->firmware_text_ = s; }
  void set_connected_vin_text_sensor(text_sensor::TextSensor *s) { this->vin_text_ = s; }
  void set_state_text_text_sensor(text_sensor::TextSensor *s) { this->state_text_ = s; }
  void set_charge_mode_text_sensor(text_sensor::TextSensor *s) { this->charge_mode_text_ = s; }
  void set_online_binary_sensor(binary_sensor::BinarySensor *s) { this->online_bs_ = s; }
  void set_vehicle_charging_binary_sensor(binary_sensor::BinarySensor *s) { this->charging_bs_ = s; }
  void set_type_check_binary_sensor(binary_sensor::BinarySensor *s) { this->type_check_bs_ = s; }

  // --- called by child entities / lambdas ---
  void on_current_limit(float amps);
  void on_charging_enabled(bool enabled);
  void start_charging() { this->on_charging_enabled(true); }
  void stop_charging() { this->on_charging_enabled(false); }

 protected:
  void setup_();
  void dump_config_();
  void adopt_(uint16_t id, uint8_t protocol, float max_amps);
  void forget_();
  void supervise_(uint32_t now);
  void process_(uint16_t cmd, const uint8_t *f, size_t len);
  void handle_heartbeat_(const uint8_t *f, size_t len);
  void handle_meter_(const uint8_t *f, size_t len);
  void handle_vin_(const uint8_t *f, uint8_t part);
  void publish_phase_data_();
  void set_online_(bool online);
  void set_assigned_(float amps);
  void update_run_();
  std::vector<uint8_t> heartbeat_frame_();
  uint16_t next_poll_cmd_();
  void log_stats_();
  float watts_per_amp_() const;
  CarKind kind_() const;  // manual (car type select) > VIN > detected (meter / L1) > unknown (treated as 3-phase)
  void meter_detect_(uint32_t now, float actual);
  // Quick car type check (IT 3-phase, no VIN, car type Auto): charge start + one 4 A step and back.
  void quick_check_(uint32_t now, float actual);
  bool quick_wanted_() const;
  CarKind quick_eval_(uint32_t now, float ia, const char *what);
  void quick_end_(const char *why);
  float ring_ago_(int k) const { return this->ring_[(this->ring_pos_ + RING - 1 - k) % RING]; }
  float ring_spread_(int n, float *mean) const;  // last n samples (5 s each); 1e9 when fewer
  void add_energy_(double kwh);           // lifetime counter: add, save each whole kWh, publish
  void publish_total_energy_();

  uint8_t index_{0};      // 0-based, logs show index_ + 1
  uint16_t pinned_id_{0};  // YAML slave_id, 0 = take the next TWC that shows up

  // entities
  TWCCurrentNumber *current_number_{nullptr};
  TWCChargingSwitch *charging_switch_{nullptr};
  ESPPreferenceObject current_pref_;

  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *max_allowable_current_sensor_{nullptr};
  sensor::Sensor *assigned_current_sensor_{nullptr};
  sensor::Sensor *total_kwh_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *phase_current_sensor_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *actual_current_sensor_{nullptr};
  sensor::Sensor *offered_current_sensor_{nullptr};
  sensor::Sensor *state_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *active_phases_sensor_{nullptr};
  sensor::Sensor *session_energy_sensor_{nullptr};
  // Own lifetime counter (total_energy): IT 3-phase from our power, else TWC counter steps. Saved in flash.
  sensor::Sensor *total_energy_sensor_{nullptr};
  TWCOffsetNumber *offset_number_{nullptr};
  ESPPreferenceObject lifetime_pref_, offset_pref_;
  double lifetime_kwh_{0};
  double lifetime_saved_kwh_{0};
  double lifetime_published_kwh_{-1};
  float offset_kwh_{0};
  text_sensor::TextSensor *car_info_text_{nullptr};
  text_sensor::TextSensor *plug_text_{nullptr};
  text_sensor::TextSensor *model_text_{nullptr};
  text_sensor::TextSensor *serial_text_{nullptr};
  text_sensor::TextSensor *firmware_text_{nullptr};
  text_sensor::TextSensor *vin_text_{nullptr};
  text_sensor::TextSensor *state_text_{nullptr};
  text_sensor::TextSensor *charge_mode_text_{nullptr};
  binary_sensor::BinarySensor *online_bs_{nullptr};
  binary_sensor::BinarySensor *charging_bs_{nullptr};
  binary_sensor::BinarySensor *type_check_bs_{nullptr};

  // Slave link
  bool have_slave_{false};
  bool online_{false};
  uint16_t slave_id_{0};
  uint8_t protocol_{2};
  float slave_max_amps_{0};
  uint32_t last_slave_hb_ms_{0};
  uint8_t slave_state_{0xFF};
  float actual_amps_{-1};  // -1 = not yet reported, so the first 0 A gets published
  float offered_amps_{-1};
  uint8_t poll_index_{0};

  // Meter
  float phase_v_[3]{0, 0, 0};
  float phase_a_[3]{0, 0, 0};
  bool meter_valid_{false};
  float v_avg_{0};  // slow L1 voltage average for the power split
  uint32_t total_kwh_{0xFFFFFFFF};
  uint8_t last_meter_raw_[20]{};

  // Info
  uint8_t vin_[17]{};
  uint8_t vin_parts_{0};
  std::string vin_published_;
  std::string vin_candidate_;  // a new VIN is published only after two equal reads
  bool have_serial_{false};
  bool have_firmware_{false};
  bool have_model_{false};

  // Control
  uint8_t limit_amps_{6};      // own setting (number / set_current)
  float assigned_amps_{-1};    // after the total split; 0 = paused
  bool charging_enabled_{true};
  bool running_{true};         // enabled and assigned > 0; changes send FCB1/FCB2
  uint32_t last_stop_sent_ms_{0};
  uint32_t last_limit_sent_ms_{0};  // 0 = send the limit (09) on the next heartbeat
  CarKind car_kind_{CAR_UNKNOWN};   // from VIN (S/X: corrected by the L1 check)
  CarKind auto_kind_{CAR_UNKNOWN};  // car without useful VIN: from the circuit meter or the L1 check
  CarKind manual_kind_{CAR_UNKNOWN};  // car type select, UNKNOWN = Auto
  TWCCarTypeSelect *car_type_select_{nullptr};
  ESPPreferenceObject car_type_pref_;
  bool is_sx_{false};             // Model S/X: sub-charger count is detected at runtime on IT 3-phase
  uint32_t sx_low_since_ms_{0};   // L1 current below 75 % of offered since ...

  // Circuit meter detection: current history every 5 s (3 min), one pending current step.
  static const uint8_t RING = 36;
  float ring_[RING]{};
  uint8_t ring_pos_{0}, ring_n_{0};
  uint32_t ring_ms_{0};
  bool step_pending_{false};
  uint32_t step_trig_ms_{0}, step_t0_ms_{0}, last_step_ms_{0};
  float i_before_{0};
  CarKind cand_kind_{CAR_UNKNOWN};  // must be seen twice in a row
  uint8_t cand_count_{0};
  // Quick type check state
  enum QPhase : uint8_t { Q_OFF, Q_WAIT_START, Q_MEAS_START, Q_PROBE_WAIT, Q_PROBE, Q_BACK };
  QPhase q_phase_{Q_OFF};
  int8_t probe_delta_{0};  // added to the assigned amps while the 4 A step runs
  uint8_t q_tries_{0}, q_count_{0};
  CarKind q_kind_{CAR_UNKNOWN};
  uint32_t q_step_ms_{0}, q_next_ms_{0};
  float q_ib_{0}, q_prev_actual_{-1};
  bool q_on_{false};
  float last_power_w_{NAN};
  uint32_t idle_since_ms_{0};     // running but car draws nothing since ... (full car)
  float session_wh_{0};           // own integration; used when the TWC counter can't see all current (2 sub-chargers)
  float session_wh_published_{-1};
  uint32_t last_integrate_ms_{0};

  // Session (plug-in to unplug, i.e. TWC state leaves 00 and returns to 00)
  bool session_active_{false};
  uint32_t session_start_kwh_{0xFFFFFFFF};
};

class TWCController : public Component, public uart::UARTDevice {
  friend class TWCCharger;

 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // --- config ---
  void set_twcid(uint16_t id) { this->twcid_ = id; }
  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_min_current(uint8_t v) { this->min_current_ = v; }
  void set_max_current(uint8_t v) { this->max_current_ = v; }
  void set_nominal_voltage(float v) { this->nominal_voltage_ = v; }
  void set_phase_threshold(float v) { this->phase_threshold_ = v; }
  void set_stop_reassert_interval(uint32_t ms) { this->stop_reassert_ms_ = ms; }
  void set_grid_type(uint8_t g) { this->grid_type_ = (GridType) g; }
  void set_setting_number(uint8_t s, TWCSettingNumber *n, float initial) {
    this->setting_number_[s] = n;
    this->setting_[s] = initial;
  }
  void set_total_power_sensor(sensor::Sensor *s) { this->total_power_sensor_ = s; }
  void set_mode_select(TWCModeSelect *s) { this->mode_select_ = s; }
  void set_circuit_power_sensor(sensor::Sensor *s);
  void on_mode(size_t index);
  void add_charger(TWCCharger *c) {
    c->index_ = this->chargers_.size();
    this->chargers_.push_back(c);
  }

  void on_setting(uint8_t s, float value);

  // VIN -> sub-charger count and "Tesla Model X 2017". Public for the self-check.
  static CarKind decode_vin(const std::string &vin, std::string &info);

 protected:
  // RX
  void read_bus_();
  void process_frame_(const uint8_t *f, size_t len);
  void handle_slave_linkready_(const uint8_t *f, size_t len);
  TWCCharger *find_(uint16_t id);
  TWCCharger *claim_(uint16_t id);

  // TX
  bool bus_free_(uint32_t now) const;
  void send_frame_(std::vector<uint8_t> body);
  std::vector<uint8_t> make_frame_(uint16_t cmd, uint16_t dest, uint8_t fill_to);
  void queue_command_(TWCCharger *c, uint16_t cmd, uint8_t repeat);
  void drop_commands_(TWCCharger *c);
  void schedule_(uint32_t now);

  // helpers
  void update_limits_();
  float whole_amps_(float a) const;
  void power_split_(std::vector<float> &want);
  bool any_slave_() const;
  static const char *state_to_text_(uint8_t state);
  static bool ascii_field_(const uint8_t *data, size_t len, std::string &out);

  // config
  uint16_t twcid_{0x7777};
  uint8_t sign_{0x77};
  GPIOPin *flow_control_pin_{nullptr};
  uint8_t min_current_{6};
  uint8_t max_current_{16};
  float nominal_voltage_{230.0f};
  float phase_threshold_{1.0f};
  uint32_t stop_reassert_ms_{5000};
  GridType grid_type_{GRID_SINGLE};

  std::vector<TWCCharger *> chargers_;
  // Settings: total amps (nullptr = each charger uses its own limit), max power W (0 = off),
  // priority = % of the power for charger 1 (others share the rest equally).
  TWCSettingNumber *setting_number_[SET_COUNT]{nullptr, nullptr, nullptr};
  float setting_[SET_COUNT]{0, 0, 50};
  ESPPreferenceObject setting_pref_[SET_COUNT];
  sensor::Sensor *total_power_sensor_{nullptr};
  TWCModeSelect *mode_select_{nullptr};  // nullptr: power mode whenever Max Power > 0
  // Circuit power from HA (optional): readings of the last 10 min, for car type detection.
  sensor::Sensor *circuit_power_{nullptr};
  std::deque<std::pair<uint32_t, float>> circuit_samples_;
  bool circuit_window_(uint32_t from, uint32_t to, bool carry, float &mean, float &spread) const;
  ESPPreferenceObject mode_pref_;
  uint8_t mode_{0};
  bool power_mode_() const {
    return this->setting_number_[SET_MAX_POWER] != nullptr && this->setting_[SET_MAX_POWER] > 0 &&
           (this->mode_select_ == nullptr || this->mode_ == 1);
  }
  uint32_t last_power_tick_ms_{0};

  // RX parser state
  static const size_t MAX_FRAME = 32;
  uint8_t rx_buf_[MAX_FRAME];
  size_t rx_len_{0};
  bool rx_in_frame_{false};
  bool rx_escape_{false};
  uint32_t last_rx_byte_ms_{0};
  uint32_t rx_frames_ok_{0};
  uint32_t rx_frames_bad_{0};
  uint32_t last_stats_ms_{0};
  uint32_t rx_bytes_{0};
  uint32_t tx_frames_{0};
  uint32_t flush_fail_{0};

  // TX scheduler state: one heartbeat slot per configured charger, each TWC gets one every second.
  uint32_t last_tx_ms_{0};
  uint32_t last_heartbeat_ms_{0};
  uint8_t hb_index_{0};
  TWCCharger *poll_charger_{nullptr};  // poll pending for this charger (the one just heartbeated)
  TWCCharger *last_polled_{nullptr};
  uint8_t startup_linkready1_{5};
  uint8_t startup_linkready2_{5};
  struct PendingCommand {
    TWCCharger *charger;
    uint16_t cmd;
    uint8_t repeat;
  };
  std::deque<PendingCommand> cmd_queue_;
};

}  // namespace twc_controller
}  // namespace esphome
