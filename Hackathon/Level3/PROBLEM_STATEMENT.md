# Level 3 — Project MIRAGE

> **Team Aerial Robotics IITK| Y25 Recruitment Hackathon**

---

## 1. Background

Modern autonomous drone swarms rely on passive ground markers such as ArUco codes and QR codes for navigation, zone identification, and precision landing. However, these markers have a critical security weakness: they broadcast their information to every camera at every altitude. Any friendly drone — or an enemy drone — that flies overhead can read them.

We need a smarter marker. One that tells different information depending on how close the drone is.

The human visual system and any camera acts as a spatial frequency filter. At high altitude, a camera captures only **low spatial frequencies** (smooth gradients, general shapes, broad colours). At low altitude, it captures **high spatial frequencies** (sharp edges, fine binary patterns, precise detail). By engineering an image that separates information across these two frequency bands, we can create a single, flat, passive marker that reveals different content at different altitudes.

---

## 2. The Challenge

Design and implement a complete algorithmic pipeline that produces a single image — called a **Hybrid Marker** — with the following behaviour:

| Drone Altitude | What the Drone Sees | ArUco Detection |
|----------------|----------------------|-----------------|
| ≥ 30 metres (High) | Decoy image — a large, bold H helipad symbol | Must **FAIL** |
| 8 – 20 metres (Transition) | Partial overlap of both images | Should **FAIL** |
| ≤ 5 metres (Low) | Secret ArUco marker with the announced ID_X | Must **SUCCEED** |

The marker must be a **single flat `.png` file**. No layers, no metadata, no active components.

---

## 3. How Altitude is Simulated (No Drone Required)

Since no physical drone, ROS, or Gazebo is available, altitude is simulated mathematically using the **Pinhole Camera Model**. The formula below calculates exactly how many pixels the printed marker occupies on the drone's camera sensor at a given real-world altitude:

```
Pixels on Sensor = (f × S_real × Resolution) / (Z × Sensor_width)
```

### Fixed Hardware Constants (Raspberry Pi Camera V2)

| Parameter | Value |
|-----------|-------|
| Focal length (f) | `3.04 mm` |
| Sensor width | `3.68 mm` |
| Resolution | `1920 pixels` (width) |
| Physical marker size (S_real) | `1.0 metre` |

### Altitude vs Sensor Pixels

| Altitude (Z) | Pixels on Sensor | Visual Result |
|---------------|------------------|---------------|
| 2 m | ~793 px | Full ArUco detail visible |
| 5 m | ~317 px | ArUco still readable |
| 15 m | ~105 px | Transition — both blending |
| 30 m | ~52 px | Only decoy visible |
| 100 m | ~15 px | Fully blurred, only color/shape |

> **Hint:** To simulate altitude Z in software, teams can shrink their 512×512 hybrid image down to the calculated pixel size using `cv2.resize()`, then optionally apply a mild Gaussian blur to simulate atmospheric haze. This single operation sufficiently replicates what a real drone camera would see at that height.

---

## 4. Technical Constraints

| Rule | Detail |
|------|--------|
| Output image | Must be exactly `512 × 512` pixels, grayscale (single channel) |
| ArUco dictionary | Must be `DICT_4X4_50` |
| Language | Python 3 with OpenCV (`opencv-contrib-python`) and NumPy |
| ArUco ID | Announced at hackathon start — must be accepted as a command-line argument. Hardcoding any ID = **immediate disqualification** |

---

## 5. Deliverables

Teams must submit a single GitHub repository containing all of the following:

| File | Description |
|------|-------------|
| `generator.py` | Takes `far_target_decoy.png` and the announced ArUco marker from the `aruco_markers/` folder as inputs. Outputs `hybrid_marker.png`. Teams must tune `σ_low` and `σ_high` using the Pinhole Camera Model math. **Usage:** `python generator.py decoy.png aruco_markers/aruco_id_X.png X output.png` |
| `hybrid_marker.png` | The final 512×512 grayscale hybrid marker. This is the **primary judged artifact**. |
| `viewer.py` | **Build from scratch — NOT provided in starter kit.** Takes image path, altitude in metres, and ArUco ID as command-line arguments. Simulates what the drone sees, saves the output image, and prints whether ArUco is detected. **Usage:** `python viewer.py hybrid_marker.png 30 X` |
| `visual_check.py` | **Build from scratch — NOT provided in starter kit.** Measures the ghosting score of the hybrid marker. Must print a numeric ghosting score. Submitted score must be below 5. **Usage:** `python visual_check.py hybrid_marker.png far_target_decoy.png` |
| `drone_view_2.0m.png` | Saved output of `viewer.py` at 2 metres — **mandatory visual proof**. |
| `drone_view_30.0m.png` | Saved output of `viewer.py` at 30 metres — **mandatory visual proof**. |
| `report.pdf` | Maximum 2 pages. Must include: the σ values chosen, the mathematical reasoning behind them, and the GSD calculation proving the transition altitude. |

### `viewer.py` — Usage & Expected Output

> `viewer.py` and `visual_check.py` are **NOT provided** in the starter kit. Teams must build both from scratch. Submitting a blank file or a file that does not run will result in **zero marks** for those criteria.

`viewer.py` must accept three command-line arguments — image path, altitude in metres, and ArUco ID:

```bash
python viewer.py hybrid_marker.png 30 X
python viewer.py hybrid_marker.png 2 X
```

**Expected terminal output at 30 metres:**
```
Drone at 30.0m → Marker occupies 52×52 pixels on sensor
Atmospheric blur: Applied
Drone sees: DECOY IMAGE dominant
ArUco ID_X NOT DETECTED → Continue cruising
Image saved → drone_view_30.0m.png
```

**Expected terminal output at 2 metres:**
```
Drone at 2.0m → Marker occupies 793×793 pixels on sensor
Atmospheric blur: None
Drone sees: ARUCO MARKER dominant
ArUco ID_X DETECTED → Precision landing zone identified
Image saved → drone_view_2.0m.png
```

**Saved image format:** The saved `drone_view` PNG must show a pixelated drone POV view at that height. It must include a top bar showing altitude and sensor resolution, the drone view in the centre, and a coloured bottom banner — green if ArUco detected, red if not.

> **Important:** Judges will call `viewer.py` live during evaluation with random altitude values. Your script must handle any positive altitude correctly.

---

## 6. Ghosting — What It Is & Why It Costs Points

Ghosting is the most common failure mode. It occurs when the ArUco marker's black grid pattern is faintly visible in the high-altitude view, meaning the decoy has been contaminated by the near-target's low-frequency content.

**To avoid ghosting:**
- **Normalise the high-pass signal** — subtract its mean before adding it to the decoy so it carries no DC offset
- **Increase the sigma gap** — `σ_low` must be at least **3×** larger than `σ_high`

Teams must measure their own ghosting score using their `visual_check.py` and include the score in their submission. Aim for a ghosting score **below 5**. Above 15 will visibly fail the judges' eye test and lose all visual quality marks.

---

## 7. Judging & Evaluation Criteria

| Criterion | Weightage | How It Is Tested |
|-----------|-----------|------------------|
| ArUco ID_X detected at ≤ 5m | **40%** | `detector.py` shrinks image to 5m pixel size and runs ArUco detection |
| ArUco ID_X hidden at ≥ 30m | **30%** | `detector.py` shrinks image to 30m pixel size and runs ArUco detection |
| Visual quality — no ghosting on decoy | **15%** | Judging script run at 30m; judges visually inspect the window for ArUco grid bleed-through |
| Math justification in `report.pdf` | **15%** | Judges verify σ values are backed by GSD/Nyquist calculations, not trial and error |

---

## 8. Starter Kit

```
Mirage-Starter-Kit/
│
├── far_target_decoy.png        ← one decoy image
│
├── aruco_markers/              ← all 50 markers pre-generated
│   ├── aruco_id_00.png
│   ├── aruco_id_01.png
│   ├── aruco_id_02.png
│   ├── ...
│   └── aruco_id_49.png
│
├── generator.py               ← badly tuned template
├── simulator.py               ← altitude tester
└── README.md
```
