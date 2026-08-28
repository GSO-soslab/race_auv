# AprilTag Pipeline Refactor Plan

Goal: replace the per-`(family, size)` `pupil_apriltags.Detector` setup with one
detector per family from the upstream `AprilRobotics/apriltag` library, and
inject the per-tag size *after* decode to compute each tag's pose via the
library's native `estimate_tag_pose()`. This eliminates redundant quad scans
across sizes and unifies the pose math in a single library.

## Status (current as of 2026-08-28)

* **Done previously**:
  * `cv2.cuda` (rectifier + JPEG encoder) and NVIDIA `nvjpeg`
    (`pyNvJPEG`) paths removed. See `image_processing.py` (top-of-file
    comment) and `apriltag_detector_node._publish_annotated` for the
    rationale.
  * `race_auv_camera_pkg/image_jpeg.py` deleted. JPEG encode is now
    inline `cv2.imencode` in
    `apriltag_detector_node._publish_annotated`.
  * CPU pipeline annotated; per-frame timings documented in `Jetson.md`.
* **Fuser hardening** (landed):
  * Fix `time.time()` -> ROS clock in `apriltag_fuser_node._tick`
    (the cache stamps were already ROS time; the comparison was not).
  * `solve_cam_to_base_ransac` added in `apriltag_geom.py`; the
    fuser uses it when >= 4 pairs are available, falls back to the
    exact joint Umeyama at 3 pairs.
  * Per-pair weighting in the SVD solve (`solve_cam_to_base` gained a
    `weights` parameter, default unweighted for back-compat). The
    fuser uses `1/d^2` weights via `_detection_weights`.
* **This plan's pupil_apriltags -> apriltag3 swap**: **implemented**.
  See `apriltag_processor.py` for the new detector and the node edits
  in `apriltag_detector_node.py`.

The line numbers throughout the rest of this document refer to the
state of the codebase after the GPU removal / fuser hardening landed.
If you re-run the plan, diff against current line numbers rather than
trusting the references below.

## Background

- The current code (`race_auv_camera_pkg/apriltag_processor.py`) creates one
  `pupil_apriltags.Detector` per `(family, tag_size)` bucket because
  pupil_apriltags' `detect()` only accepts a single `tag_size` per call.
- With the current YAML (`race_auv_bringup/config/apriltag.yaml`) we have 3
  buckets: `tag25h9 @ 0.21 m`, `tag36h11 @ 0.125 m`, `tag36h11 @ 0.04 m`.
  `_process_frame` (`apriltag_detector_node.py:555-563`) runs the quad-detect
  pass three times per frame.
- pupil_apriltags' `if/elif` chain in `bindings.py` only registers one family
  per instance despite its "space-separated families" docstring — so the
  claimed multi-family single-scan property never actually held.
- The upstream `AprilRobotics/apriltag` C extension (`apriltag_pywrap.c`)
  exposes two clean calls: `detect()` (no pose) and
  `estimate_tag_pose(det, tagsize, fx, fy, cx, cy)`. The latter matches the
  snippet in the upstream README and lets us inject the correct size per tag
  after decode.

## Architecture (after)

```
                              apriltag.yaml: tags:[{id,family,size}]
                                          |
                                          v
                         id_to_size: Dict[(family, id)] -> size_m
                                          |
                                          v
              one AprilTagDetector per node, holding
              Dict[family -> apriltag.apriltag(family=...)]
                                          |
              per frame: gray image                       |
                  |                                      |
                  v                                      |
        for each family det:                             |
            dets = det.detect(gray)        # one scan   |
            for d in dets:                                |
                if (family,id) not in id_to_size: drop   |
                size = id_to_size[(family,id)]            |
                pose = det.estimate_tag_pose(d,size,fx,fy,cx,cy)
                T = build_T(pose["R"], pose["t"])        |
                emit dict {family, tag_id, T, corners, bad_pose}
```

One Python detector instance per family (we accept one quad-scan per family —
this matches the original "one detector per family" ask from the user and
saves the (size) bucketing). Per-tag size is looked up post-decode and fed
into the same library's `estimate_tag_pose()`. No `cv2.aruco` or
`cv2.SOLVEPNP_IPPE_SQUARE` is required for pose.

## Decisions (locked in for this implementation)

| Question | Decision |
|---|---|
| Library | `AprilRobotics/apriltag` (upstream C extension). The repo has no `setup.py` / `pyproject.toml`, so we install via `cmake -B build && cmake --build build --target install` — see `Jetson.md` §0. |
| Multi-family single-quad-scan | **No** — both wrappers are single-family per instance. One `apriltag.apriltag(family=...)` per family. |
| Size table source | YAML `tags:` list (id + family + size) — no change. |
| Pose API | `apriltag3.estimate_tag_pose(det, tagsize, fx, fy, cx, cy)` — the native call, matches the README. |
| `cv2.drawFrameAxes` overlay | Already live at `apriltag_processor.py`; the "re-enable" bullet in the earlier plan was stale. Kept. |
| `decode_sharpening` | **Dropped** from YAML and from the processor's params template — was a `pupil_apriltags` knob. |
| Corner order in published dict | `lb-rb-rt-lt` (upstream's native order). Downstream fuser only reads `T`, so no consumer impact. |
| Verification | Foxglove visual on a live run. No recorded-bag regression script. |
| Fallback | Hard cutover — `python3-pupil-apriltags` removed from `package.xml`. |

## Code changes

### 1. `race_auv_camera_pkg/package.xml`

- Remove the `python3-pupil-apriltags` `exec_depend`. (No replacement
  rosdep key — install is documented in `Jetson.md` §0.)

### 2. `race_auv_camera_pkg/race_auv_camera_pkg/apriltag_processor.py`

- Replace `from pupil_apriltags import Detector` with `from apriltag import apriltag`.
- Replace the existing `AprilTagDetector` class:

  ```python
  class AprilTagDetector:
      def __init__(
          self,
          families: Sequence[str],
          id_to_size: Dict[Tuple[str, int], float],
          camera_intrinsics: dict,
          camera_distortion: Sequence[float],
          image_size: dict,
          logger,
          detector_params: dict,
      ):
          self.logger = logger
          self._detectors: Dict[str, object] = {}
          self._id_to_size: Dict[Tuple[str, int], float] = {
              (str(f), int(i)): float(s)
              for (f, i), s in id_to_size.items()
          }
          fx = float(camera_intrinsics["fx"])
          fy = float(camera_intrinsics["fy"])
          cx = float(camera_intrinsics["cx"])
          cy = float(camera_intrinsics["cy"])
          self._camera_params = (fx, fy, cx, cy)

          for fam in families:
              try:
                  self._detectors[fam] = apriltag(
                      family=fam,
                      threads=int(detector_params["nthreads"]),
                      decimate=float(detector_params["quad_decimate"]),
                      blur=float(detector_params["quad_sigma"]),
                      refine_edges=bool(detector_params["refine_edges"]),
                      maxhamming=2,
                  )
                  self.logger.info(
                      f"apriltag3 ready: family='{fam}' "
                      f"image={image_size['img_width']}x{image_size['img_height']} "
                      f"params={dict(detector_params)}"
                  )
              except Exception as e:
                  self.logger.error(f"apriltag3 init failed for {fam}: {e}")

      def detect(self, gray_image: np.ndarray) -> list[dict]:
          if gray_image is None:
              self.logger.warn("Received a null image for AprilTag detection.")
              return []
          out: list[dict] = []
          fx, fy, cx, cy = self._camera_params
          for family, det in self._detectors.items():
              try:
                  raw = det.detect(gray_image)
              except Exception as e:
                  self.logger.warn(
                      f"apriltag3 ({family}) detect failed: {e}; skipping."
                  )
                  continue
              for d in raw:
                  try:
                      tag_id = int(d["id"])
                  except (KeyError, TypeError, ValueError):
                      continue
                  key = (family, tag_id)
                  if key not in self._id_to_size:
                      continue
                  size = self._id_to_size[key]
                  try:
                      pose = det.estimate_tag_pose(d, size, fx, fy, cx, cy)
                      R_mat = np.asarray(pose["R"], dtype=np.float64)
                      t_vec = np.asarray(pose["t"], dtype=np.float64).reshape(3)
                      T = np.eye(4)
                      T[:3, :3] = R_mat
                      T[:3, 3] = t_vec
                      bad_pose = False
                  except Exception:
                      T = np.eye(4)
                      bad_pose = True
                  out.append({
                      "family": family,
                      "tag_id": tag_id,
                      "T": T,
                      "corners": np.asarray(d["lb-rb-rt-lt"], dtype=np.float32),
                      "bad_pose": bad_pose,
                  })
          return out
  ```

- Drop `_normalize_family` — the upstream wrapper's `detect()` does not return `tag_family`; the family is implicit in which detector instance fired.
- `annotate()`: re-enable the `cv2.drawFrameAxes` block (kept live; axis length is now per-detection since the same detector can serve mixed sizes).
- Detection dict now includes `"size"` so `annotate()` can size axes per tag (the old per-bucket `self.tag_size` is gone).

### 3. `race_auv_camera_pkg/race_auv_camera_pkg/apriltag_geom.py`

- **No change.** `estimate_tag_pose` lives in the C extension. Keep `sanitize_rotation`, `is_bad_rotation`, `solve_cam_to_base`, `matrix_to_pose_msg`, `matrix_to_transform_stamped`, `resolve_urdf_path`, `load_yaml_config`.

### 4. `race_auv_camera_pkg/race_auv_camera_pkg/apriltag_detector_node.py`

- Replace `_group_tags_by_family_size` with `_group_tags_by_family` returning `(per_family, id_to_size, sorted_families)`.
- The `self._detectors: Dict[Tuple[str, float], AprilTagDetector]` field is gone; in its place a single `self._detector: Optional[AprilTagDetector] = None`.
- In `_build_pipeline`: build one `AprilTagDetector(families=..., id_to_size=..., ...)`. Drop the loop over `(family, size)`. Child logger: `det_all` (no size suffix).
- `_process_frame`: single call to `self._detector.detect(gray)`. The existing bad-rotation / SVD sanitize / publish paths stay.
- `tags_override` parsing and YAML loading unchanged. `decode_sharpening` is dropped from `_detector_params_template`.
- Top-of-file "Multi-family / multi-size tags" docstring updated.

### 5. `race_auv_camera_pkg/Jetson.md`

- New section §0 "Install `apriltag3`" with the cmake build, the
  `LD_LIBRARY_PATH` / `PYTHONPATH` exports, and the `ninja` alternative.
  Updated from the plan's original `pip install git+...` recipe after
  discovering the upstream repo has no `setup.py` / `pyproject.toml`.

## Files not changed

- `race_auv_camera_pkg/race_auv_camera_pkg/apriltag_fuser_node.py`
- `race_auv_camera_pkg/race_auv_camera_pkg/urdf_tag_parser.py`
- `race_auv_camera_pkg/race_auv_camera_pkg/image_processing.py`
- `race_auv_bringup/config/apriltag.yaml` and any launch files.

(Note: `race_auv_camera_pkg/race_auv_camera_pkg/image_jpeg.py` was
removed as part of the GPU-path cleanup. The earlier revision of this
plan listed it under "Files not changed"; that reference is stale.)

## Risks (updated)

- The `apriltag_pywrap.c` build on Jetson is the biggest unknown. If it breaks, `pupil_apriltags` remains a fallback path — `AprilTagDetector`'s surface is small enough that the swap is contained to `apriltag_processor.py`.
- `estimate_tag_pose` and `cv2.SOLVEPNP_IPPE_SQUARE` use the same IPPE algorithm but can pick a different branch on near-degenerate tags. The fuser's joint Umeyama (or the post-hardening joint Umeyama + RANSAC) absorbs this; per-tag `T` still feeds `Detection3DArray` after the existing `is_bad_rotation` + `np.linalg.det > 0.5` checks at `apriltag_detector_node.py:657-666`.
- The most recent JetPack 6.x image on Orin ships OpenCV without `NVCOMPRESS`. That is what motivated the GPU-path removal and is unrelated to this plan, but worth recording: anyone re-introducing a `cv2.cuda` path would hit the same wall.

## Verification

Visual only, per the locked-in decision:

1. Launch the per-camera detector on the dock.
2. Confirm the annotated `apriltag_detection/image` topic in Foxglove
   shows bounding boxes + axis overlays + ID labels for every visible
   tag (both families: `tag25h9 @ 0.21` and `tag36h11 @ {0.125, 0.04}`).
3. Confirm `apriltag_detection/detections3d` reports one entry per
   visible tag with a sensible pose.
4. Confirm the multi-camera fuser's `object_base` TF tracks the dock
   in Foxglove / rviz2 within typical tolerance.

If any of (1)–(4) look wrong, fall back to debugging through the
detector's child logger `det_all`.

## Risks

- The `apriltag_pywrap.c` build on Jetson is the biggest unknown. If it breaks, `pupil_apriltags` remains a fallback path — `AprilTagDetector`'s surface is small enough that the swap is contained to `apriltag_processor.py`.
- `estimate_tag_pose` and `cv2.SOLVEPNP_IPPE_SQUARE` use the same IPPE algorithm but can pick a different branch on near-degenerate tags. The fuser's joint Umeyama absorb this; per-tag `T` still feeds `Detection3DArray` after the existing `is_bad_rotation` + `np.linalg.det > 0.5` checks at `apriltag_detector_node.py:578-586`.

## Rollback

Hard cutover: revert `apriltag_processor.py` + `apriltag_detector_node.py`
+ `package.xml` + `apriltag.yaml` + `Jetson.md` + this plan file. The
legacy `pupil_apriltags` dependency is no longer listed, so a rollback
must restore `python3-pupil-apriltags` in `package.xml` and `decode_sharpening`
in `apriltag.yaml` to land cleanly. No downstream code (fuser, URDF
parser, image processing) is touched.