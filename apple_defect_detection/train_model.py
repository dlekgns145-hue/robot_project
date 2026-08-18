import os
from ultralytics import YOLO


def main():
    device = 0 if os.environ.get("CUDA_VISIBLE_DEVICES") is not None else 0
    # CUDA가 실제로 사용 가능한지 체크
    try:
        import torch
        device = 0 if torch.cuda.is_available() else "cpu"
    except Exception:
        device = 0

    model = YOLO('yolov8n.pt')

    results = model.train(
        data='Apple-Dataset-3/data.yaml',
        epochs=120,
        patience=20,
        imgsz=640,
        batch=8,
        device=device,
        workers=min(4, (os.cpu_count() or 1)),
        augment=True,
        degrees=15,
        hsv_h=0.01,
        hsv_s=0.35,
        hsv_v=0.25,
        fliplr=0.5,
        flipud=0.1,
        mixup=0.05,
        copy_paste=0.1,
        scale=0.5,
    )

    best_model_path = f"{results.save_dir}/weights/best.pt"
    print("학습 완료! 모델 위치:", best_model_path)


if __name__ == '__main__':
    main()