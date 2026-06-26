# Converting Depth Anything to run on the RDK X5 BPU

> **Version:** 1.0 &nbsp;|&nbsp; **Date:** 2026-06-26 &nbsp;|&nbsp; Target: RDK X5 (Bayes-e BPU, OpenExplorer / `hb_mapper`)

The RDK X5 BPU does **not** run PyTorch/ONNX directly. A model must be compiled to Horizon's
`.bin` (HBM) format with the **Horizon OpenExplorer (OE) toolchain** (`hb_mapper`). This is the same
pipeline that produced the working `yolo11m_detect_bayese_640x640_nv12.bin` already proven on this
board (see `../YOLO11_Webcam.md`). This guide adapts it for **Depth Anything (V2)**.

> ⚠️ Conversion runs on an **x86 Linux host with Docker** (the OE toolchain image), **not** on the X5.
> You only copy the resulting `.bin` to the board.

---

## 0. Pick the right model size (this is the #1 decision — Risk R2)
Depth Anything comes in several encoders. On a ~10-TOPS edge BPU, **smaller is the right call**:

| Variant | Params | Recommendation |
|---|---|---|
| Depth Anything **V2 Small (ViT-S)** | ~25 M | ✅ **Start here** — best chance of ≥5 FPS |
| Depth Anything Base (ViT-B) | ~98 M | maybe, after S works |
| Depth Anything Large (ViT-L) | ~335 M | ❌ too heavy for real-time here |

Use a **fixed, small input** (e.g. **384×384** or **256×384**), NV12-friendly, not dynamic shapes —
the BPU wants static shapes and the camera path is NV12.

> Note: Depth Anything outputs **relative (inverse) depth**, not metric. We calibrate it to approximate
> metric using the **TF-Luna** (known forward range) — a couple of scale/shift samples per session. The
> map only needs *consistent relative* depth + that scale; absolute accuracy isn't required.

---

## 1. Export PyTorch → ONNX (on host)
```bash
# in the Depth Anything V2 repo, with the ViT-S checkpoint
python export_onnx.py \
    --encoder vits \
    --input-size 384 \
    --output depth_anything_v2_vits_384.onnx
# simplify (folds constants, fixes shapes — helps the parser)
python -m onnxsim depth_anything_v2_vits_384.onnx depth_anything_v2_vits_384_sim.onnx
```
Verify it's static-shape and check op support:
```bash
python -c "import onnx; m=onnx.load('depth_anything_v2_vits_384_sim.onnx'); print(m.graph.input)"
```
If any op is unsupported by the BPU, it will fall back to CPU (slow) — the OE checker (step 3) reports
this. Common fix: replace fancy interpolation/`GridSample` with a supported resize.

## 2. Prepare INT8 calibration data
Collect **50–200 representative frames** from the actual robot cameras (the demo room, your lighting),
resized to the model input and stored as the calibration set:
```bash
# capture from the robot, then preprocess to the model input size
python tools/make_calib_set.py --src calib_raw/ --dst calib_data/ --size 384
```
Quantization quality depends heavily on this set matching deployment imagery.

## 3. Compile with hb_mapper (inside the OE Docker image)
Create `depth_anything_config.yaml`:
```yaml
model_parameters:
  onnx_model: 'depth_anything_v2_vits_384_sim.onnx'
  march: 'bayes-e'                       # RDK X5 BPU architecture
  output_model_file_prefix: 'depth_anything_vits_384_nv12'
  remove_node_type: ''
input_parameters:
  input_name: ''                         # auto from ONNX
  input_type_rt: 'nv12'                  # runtime input = NV12 (matches camera path)
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  norm_type: 'data_mean_and_scale'
  mean_value: '123.675 116.28 103.53'    # ImageNet mean (DA preprocessing)
  scale_value: '0.01712475 0.017507 0.01742919'
calibration_parameters:
  cal_data_dir: './calib_data'
  calibration_type: 'default'            # KL/max; try 'max' if accuracy drops
  optimization: 'set_model_output_int16' # keep depth output higher precision
compiler_parameters:
  compile_mode: 'latency'
  optimize_level: 'O3'
  core_num: 1                            # one BPU core
```
Run the checker, then compile:
```bash
hb_mapper checker  --model-type onnx --march bayes-e \
    --model depth_anything_v2_vits_384_sim.onnx        # lists any CPU-fallback ops
hb_mapper makertbin --model-type onnx --config depth_anything_config.yaml
# -> output/depth_anything_vits_384_nv12.bin
```

## 4. Deploy to the board & smoke-test
```bash
scp output/depth_anything_vits_384_nv12.bin  sunrise@<x5-ip>:~/rdk-x5-navbot/models/
```
On the X5, mirror the proven YOLO11 runner pattern (it already uses `hbm_runtime` / `dnn_node`):
```python
# pseudocode — reuse the inference wrapper from rdk_model_zoo/.../ultralytics_yolo
from hobot_dnn import pyeasy_dnn as dnn
models = dnn.load('models/depth_anything_vits_384_nv12.bin')
# feed NV12 384x384, get the depth tensor out, normalize for /perception/depth
```
Check the BPU banner appears (`[BPU_PLAT] ... soc info(x5)`) — that's the proof it ran on the BPU,
exactly as in `YOLO11_Webcam.md`.

## 5. Wrap as the `depth_bpu` ROS 2 node
- **Sub:** `cam_front/image_raw` → convert to NV12 (`hobot_codec` or in-node).
- **Pub:** `/perception/depth` (`sensor_msgs/Image`, `32FC1` metres after scale, or `16UC1` mm).
- **Drop-not-queue:** if the BPU is busy, skip the frame (never build a backlog) — matches the latency
  budget in PROPOSAL §2.3.
- **Scale calibration:** subscribe to `/tf_luna/range`; fit `metric ≈ a * model_inv_depth + b` on a few
  samples; store `a,b` in `config/depth_scale.yaml`.

---

## Acceptance (ties to ROADMAP W3 + Risk R2)
- [ ] `.bin` loads and runs **on BPU** (banner present), **≥ 5 FPS** at 384×384, 1 core
- [ ] No silent CPU fallback for heavy ops (checked in step 3)
- [ ] Depth vs TF-Luna error **≤ 15 %** at 1–3 m after scale calibration
- [ ] Detection (YOLO11) still gets **≥ 3 FPS** of spare BPU time
- [ ] If any fail → pivot per **Risk R2**: drop to TF-Luna + free-space-only mapping

## References
- Horizon OpenExplorer / `hb_mapper` docs (same toolchain as the model zoo `.bin` files)
- `rdk_model_zoo/samples/vision/ultralytics_yolo` — working `.bin` runner to copy from
- `../YOLO11_Webcam.md` — proof the BPU inference path works on this board
