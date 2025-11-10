# EtherCAT CiA-402 CSP Servo Template (C) — with Omron R88D-1SN reference

**What this is:** a master-agnostic, beginner-friendly CSP template in C that shows *where* each piece goes:
PDO→struct mapping, CiA-402 enable sequence, set-point production (edge policy), dwell, ramp, and a simple FE monitor.
It keeps an **Omron R88D-1SN (CSP)** block as a concrete reference (IDs + common SDOs), yet all master calls are left as TODOs.

## Who is this for?
- You know “EtherCAT/CiA-402” by name but want a **clear, readable** C starting point.
- You need to **teach or audit** what a minimal CSP loop should contain.
- You want a **neutral** template you can port to *any* EtherCAT master.

## Safety first
This is a **teaching** file: no homing, only software clamps. Commission safely:
E-Stop, limit switches, load-free tests, and drive/vendor manuals at hand. Do not run on production machinery.

## File layout
- `servo_template.c` — the whole template (heavy inline comments).
- (you add) `CMakeLists.txt` or your build files.
- (you add) `LICENSE` of your choice.

## Quick start (conceptual)
1. Enumerate slaves, note for your drive: `vendorId`, `productCode`, PDO sizes and bit offsets.
2. Map the process image in `map_io()` and set `g_in/g_out`.
3. Optionally write SDOs:
   - `6060:0 = 8` (CSP)
   - `60C2:1 = 1000` (µs, match loop period)
   - `6065:0 = 20000` (FE window, example)
4. Call `ServoTemplate_Init(...)` once.
5. Call `ServoTemplate_Run(...)` every 1 ms from your cyclic task.
6. Watch the logs; tune `INC_STEP`, `LIMIT_POS`, `DWELL_MS`, `RAMP_MS`, and the **edge policy**.

## Porting checklist
- [ ] Implement `map_io()` with your master’s image API.
- [ ] Implement `sdo_write_u8/u32()` (CoE).
- [ ] (Optional) Implement `state_request()/state_get()` if you transition PREOP→OP here.
- [ ] Validate **PDO layouts** against your ESI/ENI; adjust `Drive_Inputs/Outputs`.
- [ ] Confirm **endian/unaligned** access; keep the provided `EC_GET/SET` if your master lacks them.
- [ ] Decide **edge policy** for set-points: `ON_TICK` vs `ON_CHANGE` (Omron CSP latches on edge).
- [ ] Review **limits** and FE window; add homing and hard limits before real motion.

## Configuration knobs (in code)
- `LOOP_PERIOD_MS`, `INC_STEP`, `LIMIT_POS`, `DWELL_MS`, `RAMP_MS`
- `FE_WINDOW_COUNTS`, `FE_WARN_PCT`
- `FAULT_COOLDOWN_MS`, `COMM_COOLDOWN_MS`
- `SETPOINT_EDGE_POLICY` (0: every tick, 1: only when target changes)
- `OMRON_R88D_EXAMPLE` and the SDO flags under it

## Signals explained (short)
- **StatusWord (0x6041)**: bit3 Fault, bit6 Switch-on disabled, mask `0x006F` encodes the main CiA-402 state.
- **ControlWord (0x6040)**: `0x0006` Shutdown → `0x0007` SwitchOn → `0x000F` EnableOp; `0x0080` FaultReset pulse.
- **TargetPosition (0x607A)**: desired position (counts). Omron CSP needs **bit4 edge** to latch a new target.

## Omron R88D-1SN note
Block kept for real-world reference (VendorID `0x00000083`, ProductCode `0x00000002`).
Common SDOs shown in the code; enable or disable per your firmware/manual.  
If you use another drive, switch off `OMRON_R88D_EXAMPLE` and keep the structure.

## Troubleshooting
- **Drive ignores targets** → check edge policy (bit4), OperationEnabled state, and interpolation time (60C2:1).
- **PDO mismatch** → verify ESI/ENI vs `Drive_Inputs/Outputs` sizes and **bit offsets**.
- **FE warnings** → enlarge FE window (6065:0), review ramp/dwell/INC_STEP, and mechanical load.

## License
Add a license file of your choice to your repository (MIT, Apache-2.0, etc.).
