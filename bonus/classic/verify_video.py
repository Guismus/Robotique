import cv2, json
from pathlib import Path

p = Path("/project/bonus/classic/results/pyramid_3.mp4")
cap = cv2.VideoCapture(str(p))
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = cap.get(cv2.CAP_PROP_FPS)
cap.set(cv2.CAP_PROP_POS_FRAMES, n - 1)
ok, frame = cap.read()
cap.release()
if not ok:
    raise RuntimeError("Video finale illisible")
cv2.imwrite("/project/bonus/classic/results/video_last_frame.png", frame)
print(
    json.dumps(
        {
            "frames": n,
            "fps": fps,
            "duration_seconds": n / fps,
            "bytes": p.stat().st_size,
        }
    )
)
