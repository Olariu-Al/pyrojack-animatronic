# Cat-Eye Pumpkin: printable parts (v2B-print)

All 20 STLs are exported from `cat_eye_pumpkin.scad` with `quality = "final"`.
Each file is already oriented for printing, flat face down. Units are mm.
To regenerate a part, set `render_mode = "part"` and `part = "<name>"` in the .scad.

**Bed size:** `head_back` (230 × 227 × 163) and `base_shell` (240 × 240) need a 256 × 256 mm bed.

## Parts

| File | Qty | Material | Supports | Settings / notes |
|---|---|---|---|---|
| head_back.stl | 1 | PLA, orange | **Tree supports**, build plate + model | Seam face down. Supports go inside the dome and under the 4 mechanism posts and the stem collar. 4 walls (the 1.6 mm wall prints solid), 15 % infill. |
| head_front.stl | 1 | PLA, orange | **Tree supports** | Seam face down, face up. Supports inside the mask and under the two LCD sockets. 4 walls. |
| trapdoor_lid.stl | 1 | PLA, brown or orange | Build-plate supports under the 2 hinge knuckles only | Underside down. |
| stem.stl | 1 | PLA, green | None | Use a 5 mm brim (small footprint). |
| base_shell.stl | 1 | PLA or PETG, black | None (bridges only) | Top skin down. 3 walls, 20 % infill. |
| neck_mount.stl | 1 | PETG | None | 4 walls, 25 % infill. Takes the tilt-stop impacts. |
| backbone_frame.stl | 1 | PETG | None | Rear face down. 5 walls, 40 % gyroid. |
| channel_cover.stl | 1 | PETG | None | 100 % infill. |
| black_plate.stl | 1 | PETG, black | None | Includes the 40 mm LED-wire anchor tab. 5 walls, 50 % infill. |
| sector_gear.stl | 1 | PETG | None | Gear face down, ratchet cup up. 0.12–0.16 mm layers, 100 % infill. |
| drive_disk.stl | 1 | PETG | None | Horn face down. 100 % infill; the flexure pawls must be solid. |
| antireverse_pawl.stl | 1 | PETG | None | 100 % infill. |
| rack_carrier.stl | 1 | PETG | None | 100 % infill. Ream the Ø8 bore with an 8 mm drill until the Al tube slides in with a push. |
| shoulder_cap.stl | 1 | PETG | None | 100 % infill. |
| flame_body.stl | 1 | Translucent or natural PETG | None | 0.16 mm layers. Walls are ≥ 1.8 mm everywhere for even light diffusion. |
| flame_collar.stl | 1 | Translucent PETG | None | 100 % infill. |
| keeper.stl | 1 | PETG | None | 100 % infill. |
| deadbolt.stl | 1 | PETG | None | 100 % infill. |
| lcd_retainer.stl | **2** | PLA | None | |
| tpu_pads.stl | 1 set | TPU 95A | None | 3 pads: stop pad Ø8 × 1.5, cap pad Ø8 × 1, lid pad Ø12 × 1. |

## Changes from the Phase 2A model

1. **Flame vs black plate collision (fixed).** During the Step 4 tip-slip, the plunger travels 11.93 mm *below* its armed position. In 2A this drove the flame base 9.9 mm into the black plate, which would stall the MG995. The flame now starts 13.5 mm above the plate (1.57 mm left at maximum over-travel). The flame is 47.5 mm tall instead of 59 mm, so the lid height is unchanged. It still sticks out **44.2 mm** when fired, the same as before.
2. **LED access.** The 2A flame was a sealed print with no way to fit the LEDs. It is now two parts:
   - **flame_collar:** grub-screwed to the tube.
   - **flame_body:** has an open Ø16.2 bottom and slides over the LED band. It is clamped between the collar and the shoulder cap.
   - The cavity fits a band up to **Ø13 × 12 mm**: 4× WS2812B-2020, as in the BOM, wrapped around the tube.
3. **LED wiring.** Three 30 AWG silicone wires leave through a Ø3.5 side hole and form a ~40 mm loop to the anchor tab on the black plate. The tab sits outside the flame's 24.5 mm envelope, at the middle of the exit hole's travel, so the loop stays nearly taut at both ends of the stroke. Point the side hole at the tab.
4. **Changes for flat printing:**
   - The gear's 1 mm front boss is gone, so the gear prints flat. The hub nose moved, keeping the same 0.4 mm play.
   - The drive disk's horn recess is gone. The MG995's centre screw centres it.
   - The rack teeth now run out to the carrier's front face, so they print on the bed instead of overhanging.
   - The stem has a 45° base cone, and the lid has a matching countersink.
   - The LCD retainer's screw ears are now solid lugs, and its cable notch runs into the window. This removes two tangent-geometry slivers that made the ring non-manifold.

**Re-checked after these changes:** flame vs mechanism at top / armed / hold / max over-travel; flame vs shell, crown, deadbolt and MG90S; flame vs lid open (110°) and closed; new rack vs channel and gear; gear vs frame, disk and washer; anchor tab vs shell and internals; stem vs lid. All clear. The only contacts are the designed seating faces.

## Assembly notes for the changed parts

- **Flame:**
  1. Slide the LED band onto the tube, put the flame body over it, then the collar.
  2. Push the shoulder cap into the tube top.
  3. Press the collar up so the flame is clamped against the cap, then tighten the collar grub.
- **Stop pad:** glue the TPU pad on the lug top under the M3 stop screw, then set the screw so the pick-up tooth sits **+9.25 mm** above the gear axis at top of stroke. The clean-engagement window is +7.5 to +11.0. `render_mode = "mesh_check"` shows the target mesh.
- **Lid and cap pads:** press them into the Ø12.4 and Ø8.4 pockets; they sit flush.

## Purchased hardware (unchanged)

- **Tubes and springs:**
  - D8×1 Al tube, 165.6 mm (shaft)
  - D6×1 Al tube, 99.6 mm (spring guide)
  - Ejector spring 0.8 × 11.8 OD × 100
  - Neck spring 3.0 × 33 OD × 50
  - Steel plate 230 × 230 × 4
- **Fasteners:**
  - M3 heat-set inserts (Ø4.1 holes)
  - M3×12 stop screw + nut
  - M3×20 horn screw + Ø10 washer
  - M3×30 hinge pin
  - M3 grubs (collar, carrier, neck cups)
  - M2 screws (servo tabs, switch, IMU)
- **Magnets and dowels:**
  - 8×3 magnets ×16 (shell seam)
  - 6×3 magnets ×2 (lid hold-open)
  - D3×10 dowels ×4
- **Electronics:**
  - A3144 hall sensor + 3×2 magnets ×2
  - KW11-3Z lever switch
  - 4× WS2812B-2020 LED band
