# Home Assistant: kWh per car

The charger's `total_energy` sensor counts all energy. To also count kWh **per car**, use a Utility Meter with one tariff
per car, plus an automation that picks the tariff from the VIN. The ESP needs none of this; it is all set up in HA.
It works the same with one charger shared by several cars as with several chargers.

A Tesla reports its VIN to the Wall Connector, so it can be recognised. Other cars report no VIN (`Connected VIN` = `0`).
They can go to a "Guest" counter, or you can choose the car from a dropdown.

## 1. Energy offset (optional, once)

Set **Total Energy Offset** to what the charger had delivered before, so that `total_energy` starts at the right value.
Do not lower it later: HA treats that as a meter reset.

## 2. Helpers

Go to **Settings → Devices & services → Helpers → Create helper**.

**a) Dropdown (optional)**
- Name: `Car without VIN`.
- Options: `Auto`, `Guest`, plus a name for each car you have without a VIN (for example `Leaf`).
- This creates `input_select.car_without_vin`. If you skip it, everything behaves as `Auto`.

**b) Utility Meter**
- Name: `Charger 1 per car`.
- Input sensor: the charger's **Total Energy**.
- Meter reset cycle: none.
- Tariffs: one per car, for example `Guest`, `Leaf`, `Model 3`, `Model Y`.

This creates `select.charger_1_per_car` and one kWh sensor per tariff. Those sensors can be added to the HA Energy
dashboard.

With several chargers, make one Utility Meter per charger, and one template sensor per car that adds them up.

## 3. Automation

Create a new automation, open **⋮ → Edit in YAML** and paste the YAML below. Before you save, change:
- the entity IDs, to match your entities in HA;
- the VINs, to your cars.

What it does:
- A known VIN selects that car.
- A car without a VIN selects what the dropdown says, then a 1-phase car on IT (optional rule), and otherwise `Guest`.
- If the dropdown is missing, it counts as `Auto`.
- If the car is not a tariff in the Utility Meter, nothing happens.

```yaml
alias: Charger 1 select car
mode: queued
triggers:
  - trigger: state
    entity_id:
      - sensor.twc_connected_vin
      - sensor.twc_charge_mode
      - input_select.car_without_vin
  - trigger: homeassistant
    event: start
conditions:
  - condition: template
    value_template: >
      {{ states('sensor.twc_connected_vin') not in ['unknown', 'unavailable'] }}
actions:
  - variables:
      vins:
        "5YJ3XXXXXXXXXXXXX": Model 3      # your VINs here
        "7SAYXXXXXXXXXXXXX": Model Y
      manual: ['Guest', 'Leaf']           # the dropdown options that are cars
      vin: "{{ states('sensor.twc_connected_vin') }}"
      mode: "{{ states('sensor.twc_charge_mode') }}"
      choice: "{{ states('input_select.car_without_vin') }}"
      car: >
        {% if vin in vins %}{{ vins[vin] }}
        {% elif choice in manual %}{{ choice }}
        {% elif mode.startswith('1-phase IT') %}Leaf
        {% else %}Guest{% endif %}
  - condition: template
    value_template: >
      {{ car in (state_attr('select.charger_1_per_car', 'options') or [])
         and car != states('select.charger_1_per_car') }}
  - action: select.select_option
    target:
      entity_id: select.charger_1_per_car
    data:
      option: "{{ car }}"
```

Notes:
- The firmware publishes a VIN only after reading the same VIN twice, so a VIN that was read wrong does not switch the
  tariff.
- On IT 3-phase, a car without a VIN is measured during the first ~1.5 minutes (see *Car type* in the README). Until then
  it counts as `Guest`. Remove the `1-phase IT` line if you do not need it. It only works with `Car Type` set to `Auto`.
- The car is seen by its VIN, so the same car can be counted on several chargers if each one has its own automation.
