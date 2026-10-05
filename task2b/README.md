# Task 2B - Sign & Obstacle Detection Boilerplate

Niti Vahan (NV), eYRC 2026-27.

## Files

| File | What it is |
|---|---|
| `sign_obstacle_detection.py` | The boilerplate. Fill in `detect_signs()` and `detect_obstacle()`; leave everything else alone. |
| `public/` | The public set of images to develop against. |
| `reference_images/` | Parking/Hospital reference crops (`P.jpg`, `Hospital.png`), shipped for implementations that want to match against them. Not part of the submission zip - see Submission Procedure. |

## Requirements

Your `NV_<Team-ID>` environment from Task 0 already has what you need:

```sh
conda activate NV_<Team-ID>
python -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
```

## What you implement

Two functions:

```python
def detect_signs(image):
    return [{"label": <one of SIGN_LABELS>, "area": <int>}, ...]

def detect_obstacle(image):
    return {"present": <bool>, "area": <int>}
```

- `detect_signs()` - one entry per sign visible in the image; an empty list if there
  are none. An image can hold more than one sign at once.
  `SIGN_LABELS` is `("School", "Speed40", "Parking", "Hospital")`.
- `detect_obstacle()` - whether the obstacle is visible, and its area in pixels if so
  (`0` when `present` is `False`).

Both functions must only compute and return. No `cv2.imshow()`, `cv2.waitKey()`,
`cv2.imwrite()` or `print()` inside them - reading images and reporting results is
the evaluator's job, not this file's. There is no `main()` here to run directly;
use the evaluation tool (`eyantra-autoeval evaluate --task 2b`) to run your code
against the public images.

Full instructions are in the theme book.
