import time
import os
import cv2
from ultralytics import YOLO

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(CURRENT_DIR, "apple_defect_v5_local.pt")
TRACKER_PATH = os.path.join(CURRENT_DIR, "my_tracker.yaml")


def benchmark(video_name, process_every_n_frames=1, total_frames=60):
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(os.path.join(CURRENT_DIR, video_name))
    if not cap.isOpened():
        print(f"'{video_name}' 영상을 열 수 없습니다.")
        return

    start = time.perf_counter()
    count = 0
    prev_results = None

    while count < total_frames:
        ret, frame = cap.read()
        if not ret:
            break

        if count % process_every_n_frames == 0:
            prev_results = model.track(
                source=frame,
                conf=0.5,
                iou=0.45,
                agnostic_nms=True,
                persist=True,
                tracker=TRACKER_PATH,
                save=False,
                verbose=False,
            )

        count += 1

    elapsed = time.perf_counter() - start
    fps = count / elapsed if elapsed > 0 else 0
    print(f"process_every={process_every_n_frames} | frames={count} | elapsed={elapsed:.2f}s | FPS={fps:.2f}")
    cap.release()


if __name__ == "__main__":
    for step in [1, 2, 3]:
        benchmark("apple_a.mp4", process_every_n_frames=step, total_frames=60)
