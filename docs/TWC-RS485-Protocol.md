# Tesla Wall Connector Gen 2 — RS485 load-sharing protocol

Compiled 2026-11-09 from source code and community docs. Nothing here comes from Tesla.
It is all reverse engineered, so treat it as "observed behaviour", not spec.

Sources (tags used below):

| Tag | Source |
|---|---|
| DV | dracoventions/TWCManager (`TWCManager.py`, original, Chris Dragon) — local copy `ref/TWCManager` |
| CP | craigpeacock/TWC (`TWC.c`, Linux C, fw 4.5.3) — local copy `ref/TWC` |
| JN | jnicolson/esphome-twc-controller (ESP32/esphome) — local copy `ref/esphome-twc-controller` |
| NG | ngardiner/TWCManager (maintained fork, `lib/TWCManager/…`, `docs/`) |
| UK | ukoda/TWCProtocolMonitor `Protocol.md` — best message catalogue |
| MD | MelchMwoan/esphome-twc-director `PROTOCOL.md` — low confidence, conflicts with others |
| TMC | teslamotorsclub.com thread "New Wall Connector load sharing protocol" (72830) — origin of it all |

---

## 1. Physical layer

| Item | Value | Src |
|---|---|---|
| Bus | RS485, 2-wire half duplex | all |
| Serial | **9600 baud, 8N1** (verified with scope) | DV, CP, JN |
| Terminals | 4-pin header right of rotary switch: IN(+,−) left, OUT(+,−) right. Use IN *or* OUT. + to +, − to −. | Tesla manual, NG |
| Cable | Shielded twisted pair, ≤15 m between units (Tesla manual: 18 AWG; NG: 22 AWG fine) | Tesla, NG |
| Termination | 120 Ω across D+/D− at chain ends; bias 680 Ω D+→5 V, D−→GND. Often not needed on short runs; missing termination = garbage bytes between frames | DV, UK |
| Ground | TWC exposes no RS485 ground reference | UK |
| Rotary switch | **F = slave** (what we want). 0 = test, E = invalid. Master positions set current (EU 1–8 = 6/8/10/13/16/20/25/32 A). In slave mode switch does not limit current — the master does. | DV, NG, lucienkerl |
| Max units | 1 master + 3 slaves | Tesla, UK, DV |
| Transceiver | Drive DE/RE pin high only while sending; flush UART before releasing (JN `flow_control_pin`) | JN |

⚠ Wrong wiring to that header can damage the TWC (NG).

---

## 2. Framing (SLIP-like)

```
C0 | body (escaped) | checksum (escaped) | C0 | [trailer byte]
```

- **Delimiter**: `0xC0` start and end.
- **Escaping** (same as SLIP): inside body `C0 → DB DC`, `DB → DB DD`. Checksum byte gets escaped too.
- **Checksum**: `sum(body[1 .. end]) & 0xFF` — sum of every body byte **except the first** (the FB/FC/FD type byte). Computed on the *unescaped* body. (DV, CP, JN, NG, UK agree.)
- **Trailer byte** after closing C0: TWCs send `FE` (early TMC logs: every frame ends `C0 FE`). Without bias resistors it often arrives corrupted as `FC`/`FF`/`02 00` or is lost (TMC). JN sends `C0 … C0 FF`, UK says send `FE` because some TWCs ignore frames without it, NG sends nothing. **Send `FE`; when receiving, ignore anything between closing C0 and the next C0.**

### Lengths (unescaped, no C0, checksum included)

| Length | Meaning |
|---|---|
| 14 | Protocol v1 (old fw). Heartbeat payload 7 bytes. Min current 5 A |
| 16 | Protocol v2 (fw ≥ ~4.x). Heartbeat payload 9 bytes. Min current 6 A |
| 20 | Long replies (fw 4.5.3+): FDEB, FDED, FDEE, FDEF, FDF1, FBA4 |

Detect protocol version from length of slave's FDE2 linkready (DV/NG). v2 slaves usually also accept v1-length commands.

### Receiver tips (from DV/JN experience)
- If a `C0` shows up after < ~3 bytes, you probably locked onto an end delimiter — treat it as new start (JN).
- DV restarts frame on `C0` seen before 15 bytes; finishes frame on `C0` after ≥16.
- Partial frame timeout ~2 s (DV).
- Only transmit when RX is idle, or you collide (DV sends only when no incoming bytes).

---

## 3. Generic frame layout

All multi-byte numbers are **big-endian**. Amps are **centiamps** (`0x0C80` = 3200 = 32.00 A) unless noted.

```
byte 0-1  command  (type byte FB/FC/FD + function byte)
byte 2-3  sender TWCID
byte 4-5  receiver TWCID   (requests/heartbeats)   — or start of payload (broadcasts/replies)
...       payload, zero padded
last      checksum
```

Type byte (MD, consistent with all others):
- `FB` = request, expects `FD` reply with same function byte
- `FC` = command / broadcast, no data reply
- `FD` = data: reply or slave/master broadcast

**TWCID**: 2 bytes. Normally the last 4 digits of the TSN serial, as BCD-looking hex (TSN `…9819` → `98 19`; TMC). On conflict the **slave** picks a new random ID and new sign; master never changes. Fakes use `0x7777` (DV/NG), `0xF00D` (MD), `0xAB32` (JN example).

**Sign**: 1 random byte after ID in linkready frames. Constant until reset. Purpose unknown. `0x77` works.

---

## 4. Message catalogue

### 4.1 Link / presence

| Cmd | Dir | Layout (v2, 16 B) | Notes |
|---|---|---|---|
| `FC E1` | M→all | `FCE1 masterID sign 00×8 [00 00] ck` | Master linkready 1. Sent ~5–7× at boot |
| `FB E2` | M→all | `FBE2 masterID sign 00×8 [00 00] ck` | Master linkready 2. 5× at boot (UK saw 3×). Slaves ignore first, answer 2nd with 5× FDE2 ~0.2 s later |
| `FD E2` | S→all | `FDE2 slaveID sign maxAmps(2) 00×6 [00 00] ck` | **Slave linkready.** maxAmps = `1F40` (80 A, US) or `0C80` (32 A, EU). Sent every ~10 s (UK: 6 s) while no master heartbeat |
| `FC 1D` | M→all | `FC1D 00×11 ck` | Idle "still here", 3× every ~4 h (TMC log; DV code comment says 2 h) |

- Neither FCE1 nor FBE2 is needed. A master that just answers FDE2 with heartbeats works (DV tested).
- Seeing FCE1/FBE2 from the unit you want to control = it is set to master. Turn rotary to F.
- Advertising a different maxAmps than the other units → master error "6 red blinks" (mismatched current).

### 4.2 Heartbeat (the core)

Master sends to each slave ~1×/s (round robin). Slave answers each one within ~150–200 ms. Slave never sends a heartbeat unprompted.

**FB E0 — master → slave**
```
FB E0 | masterID | slaveID | cmd | amps(2) | masterPlugged | 00 00 00 [00 00] | ck
                              6     7-8         9
```

| cmd | Meaning |
|---|---|
| `00` | No change |
| `02` | Force error. byte 7 = bitmap: `01` 3 blinks (bad rotary), `02` 5 blinks (>3 slaves), `04` 6 blinks (mixed max current). Slave then stops talking until reset. Bits `11110xxx` need power cycle. **Don't send.** |
| `05` | Set limit (v1). In v2 only works *before* charge starts; ignored while charging |
| `06` | v2: raise by 2 A. Slave echoes 06, ~44 s later reports 0A |
| `07` | v2: lower by 2 A temporarily. Slave echoes 07, raises back after ~10 s, reports 0A. Real masters use 06/07 to probe who wants amps |
| `08` | Ack slave stopped charging; bytes 7-8 may carry amps |
| `09` | **v2: set limit to amps in 7-8.** The one to use |

byte 9 = `01` if the master's own cable is in a car.

**FD E0 — slave → master**
```
FD E0 | slaveID | masterID | state | maxAmps(2) | actualAmps(2) | 00 00 [00 00] | ck
                              6       7-8           9-10
```

| state | Meaning |
|---|---|
| `00` | Ready (car may or may not be in). Also after FCB2 stop |
| `01` | Plugged in, charging |
| `02` | Error (e.g. no master heartbeat too long) |
| `03` | Plugged in, do not charge (car full / stopped / scheduled). Can stick — replug needed |
| `04` | Plugged in, ready / scheduled. With maxAmps = 0 → "give me a limit" |
| `05` | Busy (~1 s blips) |
| `06`/`07` | Echo of master 06/07 |
| `08` | Starting to charge (ramps 0 → ~1.3 A); also seen after master came back |
| `09` | Echo of master 09 — near constant in v2 |
| `0A` | 06/07 adjustment period done |
| `0F` | Seen once, unknown |

maxAmps = limit the slave is applying (echoes master, even insane values like 655.35 A — **slave does not sanity-check**). actualAmps = what car draws (noise of ±0.1–0.3 A is normal).

Handshake seen by DV (v1):
```
S: 04 00 00 …     → M: 05 12 C0 …  (slave asks, master grants 48.00 A)
S: 04 12 C0 …     → M: 00 00 00 …  (slave confirms, master says ok)
```
JN's simple approach: send `09 + amps` when setpoint changes or slave goes to state 04, else send `00`.

### 4.3 Data requests (master → slave, 16 B, reply from slave)

Request layout: `FBxx masterID slaveID 00×9 ck` (NG pads 8 zeros for some; both seem accepted).

| Req | Reply | Reply layout | Notes |
|---|---|---|---|
| `FB 19` | `FD 19` | ID + 11-char serial (old) | TMC: also mfg date |
| `FB 1A` | `FD 1A` | model string e.g. `EVW2T32HLC` | |
| `FB 1B` | `FD 1B` | `major minor rev` at [2..4] (no ID) | old fw ver, e.g. 4.5.3 |
| `FB B4` | `FD B4` | ID, plug state at [4]: 0 unplugged, 1 charging, 2 ?, 3 plugged not charging | |
| **`FB EB`** | **`FD EB`** (20 B) | `ID kWh(4) V1(2) V2(2) V3(2) I1 I2 I3 00 00 ck` | **Meter.** kWh lifetime uint32; volts uint16; current 1 byte each = **A × 2** (0.5 A steps; CP, JN; confirmed by TMC log where `1A` = 13 A). Energy is **kWh** (TMC logs show small whole numbers like 5); MD's "Wh" claim looks wrong. US example `00000014 00F6 0000 0000`; EU 3-phase `00000038 00E6 00F1 00E8`. Real master broadcasts FDEB every ~2 s (UK). NG polls ≤1/min |
| `FB EC` | `FD EC` | ID + `a b c d` | Extended fw, e.g. 4.5.3.2 |
| `FB ED` | `FD ED` (20 B) | ID + 11-char serial, not NUL-terminated | New serial |
| `FB EE` | `FD EE` (20 B) | ID + 7 VIN chars | VIN 1–7 |
| `FB EF` | `FD EF` (20 B) | ID + 7 VIN chars | VIN 8–14 |
| `FB F1` | `FD F1` (20 B) | ID + 7 chars (only 3 used) | VIN 15–17. All zero = no car / non-Tesla |

### 4.4 Commands (no data reply)

| Cmd | Layout | Meaning |
|---|---|---|
| `FC B1` | `FCB1 masterID slaveID 00×9 ck` | Start charging (close contactor). Ack `FD B1` |
| `FC B2` | same | Stop charging (open contactor). Ack `FD B2`. Slave → state 00 |

Alternative stop: heartbeat `09` with 0 A (car goes to state 03 and sleeps). Valid currents are 0 or ≥ 6 A (v2) / ≥ 5 A (v1).

### 4.5 ☠ Never send

| Cmd | Why |
|---|---|
| `FC 19` | Burns new TSN/TWCID into **OTP** (one-time-programmable) memory. Permanent. Often freezes TWC |
| `FC 1A` | Burns new model number into OTP. After 3 writes with a bad value the TWC boots into a broken "master" state and never charges again. Bricked 2 TWCs in TMC thread |
| `FB E8` | Crashes TWC (DV) |
| `FC EA` | Acts like a 3–4 s reset press, leaves TWC in odd state (heartbeat `02 01`) |
| `FC E5` | Pulls an MCU GPIO low; interrupts car CAN ~3 s. Repeating it clicks car relay on/off. Useless, wears relays |
| `FB E0` with cmd `02` | Latches error state until reset button |
| Scanning unknown commands | Never brute-force FB00–FFFF. That is how the TWCs above died |

JN and DV block FC19/FC1A in their send function. Do the same.

---

## 5. Timing and state machine

| Event | Timing | Src |
|---|---|---|
| Master boot | 5–7× FCE1 then 5× FBE2 (real: ~1 s apart; fakes use 0.1 s fine) | DV, UK |
| Slave linkready while unlinked | every ~10 s (UK: 6 s) | DV, UK |
| Master heartbeat per slave | ~1 s; JN uses ~0.5 s heartbeat + ~1 s data request alternating | all |
| Slave reply latency | 150–200 ms | UK |
| Master quiet after any frame | ~400 ms | UK |
| Slave w/o heartbeat | ~10 s → red LED 4 blinks, back to linkready | DV |
| Master drops silent slave | ~26 s | DV, NG |
| Car w/o heartbeat while charging | 20–30 s → "charge cable fault", port red | DV |

**When master dies**: sources disagree. DV/UK: slave falls back to linkready + error, car stops. lucienkerl: last setpoint holds. **Design assumption: you must keep heartbeating, and the car will stop if you don't.**

Minimal fake-master loop:
```
1. (optional) send 5× FCE1, 5× FBE2
2. listen; on FDE2 → add slave (ID, maxAmps, proto ver by length), send FBE0 at once
3. every ~1 s per slave: FBE0 (09 + amps if changed / slave state 04, else 00)
4. between heartbeats: rotate FBEB, FBED, FBEE/EF/F1, FBEC requests
5. on FDE0: store state + actual amps
6. slave silent > 26 s → forget it
```

---

## 6. Practical gotchas

- **Min current**: 6 A (v2). Below that car can latch state 03 until replug (DV). DV keeps a car on ≥ 1 min before cutting, to save the relay and avoid car errors.
- **6 A lock (car fw Oct 2017)**: car may stick at 6 A. DV workaround: briefly offer 21 A (US) / 16 A (EU), then drop to wanted value.
- **Never exceed wiring**: slave obeys any value. Clamp to breaker × 0.8 and to the advertised maxAmps.
- **Three-phase EU**: FDEB gives per-phase V and A; use for load balancing.
- **IDs on the wire** are raw bytes; JN stores them unswapped on little-endian ESP32 — keep them as byte pairs, don't convert.
- **JN bug**: `GetPowerStatus()` sends `GET_FIRMWARE_VER` (FB1B), not FBEB. Its poll loop uses FBEB directly so it still works.
- Reset button: ~4 s clears error, ~6 s reboot.

---

## 7. Findings from the TMC thread (read 2026-11-09, 34 pages, 665 posts)

Main researcher: WinterDragoness (= CDragon, author of DV). He dumped and traced the firmware over JTAG.

### Hardware and firmware
- MCU is a TI **TMS320F28034**. JTAG was left unlocked. The I²C EEPROM (UA23) holds almost nothing.
- Model number and TSN live in the MCU's **OTP memory**, in 16-value blocks. That is why FC19/FC1A can brick a unit (section 4.5).
- IN and OUT terminals are the same bus wired straight through. Every unit hears every frame.
- **TWCID = last 4 digits of the TSN (serial)** in normal units, e.g. TSN `A19D0029819` → ID `9819`. TSN format: letter + year + date code + number (`A18G…` = made late Mar/early Apr 2018).
- Model strings seen: `EVWTS80HL` (US 80 A), `EVW2T32HL` (EU 32 A).

### Firmware versions (FB1B reply `FD 1B maj min rev`)
| Version | Notes |
|---|---|
| 4.3.11, 4.3.13 | Protocol 1 (14-byte frames). Heartbeat `05` with 0 A **stops charging cleanly**: relay clicks off, car goes to state 03 and sleeps, resumes when car wakes (~15 min) |
| 4.4.13 (`04 04 0D`) | Protocol 2 (16-byte). Cannot go below 6 A; 0 A is ignored |
| 4.5.3 (ext 4.5.3.2) | Units from about Mar 2018. Adds 11 commands: FCB1/FCB2, FBB4, FBEC, FBED, FBEE/EF/F1, test-mode enter/exit, a memory read |

WD flashed P1 firmware onto a P2 unit over JTAG to get clean 0 A stop. Not practical for normal users.
Gen 3 Wall Connector: its RS485 port is silent and ignores everything. **This protocol is Gen 2 only.**

### Stopping a charge on protocol 2 — the hard problem
| Method | Result |
|---|---|
| Heartbeat `09` with 0 A | Ignored; car keeps charging at 6 A |
| "640 A offset": amps ≥ 640.00 A → car draws (value − 640) A, so `FA00` = 0 A | Lets you set 0–15 A, but 0 A **still does not stop** the charge. Curiosity only |
| `FCB2` stop (4.5.3+) | Opens relay but sends no CAN stop to the car → red ring, "unplug and try again". Seen on 2014 Model S and 2019/2020 Model 3. FCB1 then won't restart without replugging |
| FCB2/FCB1 with **DIP switch 2 down** (legacy, CAN off) | Works, but after a few cycles the car errors and may cap itself at 30 A |
| Heartbeat `02 01` (error) | Instant stop, needs reset button |
| Stop sending heartbeats | Car stops after 30–40 s; sometimes needs replug |
| Tesla vehicle API (stop/start charge) | What DV/NG ended up using. Needs car credentials |

Car fw 2020.24.6 fixed a Model 3 bug where the car would not wake after FCB1 restored power (Craig 128). Later behaviour of FCB1/FCB2 with newer car firmware: not confirmed in the thread.

### Other details
- Real master boot burst, US unit: 5× `FCE1` then 5× `FBE2`, all ending `C0 FE` (IanAber log).
- Master idle `FC 1D 00…00 1D`, 3× every **4 hours** (TheNoOne). DV's "2 hours" is likely wrong.
- When a fake slave fails the handshake, a real master repeats its heartbeat 26× before giving up.
- Car ↔ TWC talk single-wire CAN on the pilot line. CAN ID `0x30C` (1 byte: 00 stop / 01 start) seen from the car side (Fuzzylogic). DIP switch 2 down disables CAN = plain J1772 mode.
- J1772 pilot allows 6–51 A in 0.6 A steps. Stick to whole amps; some fractional changes triggered the car's 6 A drop bug.
- A real master with rotary at position 1 (6 A) plus a slave → both refuse to charge, heartbeat `02 01`.
- Most comms failures in the thread were wiring: untwisted pair, loose terminal, missing bias/termination, a bad USB adapter, or baud typo (9200).

---

## 8. More projects / links

- ukoda/TWCProtocolMonitor — Protocol.md: https://github.com/ukoda/TWCProtocolMonitor
- ngardiner/TWCManager (active fork, docs/): https://github.com/ngardiner/TWCManager
- MelchMwoan/esphome-twc-director: https://github.com/MelchMwoan/esphome-twc-director
- lucienkerl/esphome-twc-gateway: https://github.com/lucienkerl/esphome-twc-gateway
- erwin314/twc_gen2: https://github.com/erwin314/twc_gen2
- zagor/wallconnector (Python): https://github.com/zagor/wallconnector
- shreddedbacon/twc-controller (Go): https://github.com/shreddedbacon/twc-controller
- jnicolson/TWCController (ESP32 board hardware): https://github.com/jnicolson/TWCController
- TMC thread (blocks bots, read in browser): https://teslamotorsclub.com/tmc/threads/new-wall-connector-load-sharing-protocol.72830/
- evcc: no native Gen2 driver, bridges via TWCManager HTTP: https://github.com/evcc-io/evcc/discussions/6413
