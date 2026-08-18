from ultralytics import YOLO
import cv2
import os
import numpy as np
from PIL import ImageFont, ImageDraw, Image

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

CLASS_COLORS = {
    "apple": (0, 255, 0),
    "damaged_apple": (0, 0, 255),
}

DEFECT_CONF_THRESHOLD = 0.5
FONT_CACHE = {}


def get_korean_font(size):
    if size not in FONT_CACHE:
        try:
            FONT_CACHE[size] = ImageFont.truetype("malgun.ttf", size)
        except IOError:
            FONT_CACHE[size] = ImageFont.load_default()
    return FONT_CACHE[size]


def put_korean_text(img, text, position, font_size, color):
    img_pil = Image.fromarray(img)
    draw = ImageDraw.Draw(img_pil)
    font = get_korean_font(font_size)
    draw.text(position, text, font=font, fill=color)
    return np.array(img_pil)


def estimate_wound_ratio(img, x1, y1, x2, y2):
    crop = img[y1:y2, x1:x2]
    if crop.size == 0:
        return 0.0

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    lower_dark = np.array([0, 0, 0], dtype=np.uint8)
    upper_dark = np.array([180, 255, 100], dtype=np.uint8)
    mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask_dark = cv2.morphologyEx(mask_dark, cv2.MORPH_OPEN, kernel)
    mask_dark = cv2.morphologyEx(mask_dark, cv2.MORPH_CLOSE, kernel)

    wound_pixels = cv2.countNonZero(mask_dark)
    total_pixels = crop.shape[0] * crop.shape[1]
    return wound_pixels / total_pixels if total_pixels > 0 else 0.0


def analyze_apple_image(image_name):
    print("사과 이미지 분석 AI 로딩 중...")
    model = YOLO("apple_defect_v5_local.pt")

    image_path = os.path.join(CURRENT_DIR, image_name)
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    if img is None:
        print(f"'{image_name}' 사진을 찾을 수 없습니다. 경로와 이름을 확인해 주세요.")
        return

    print(f"[{image_name}] 사진 분석을 시작합니다.")

    results = model.predict(source=img, conf=DEFECT_CONF_THRESHOLD, iou=0.45)
    apple_count = 0
    defect_count = 0

    boxes = results[0].boxes
    if boxes is not None:
        for box in boxes:
            apple_count += 1
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            confidence = float(box.conf[0])
            color = CLASS_COLORS.get(label, (0, 255, 0))
            korean_label = "정상 사과" if label == "apple" else "손상된 사과"
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            if label == "damaged_apple":
                defect_count += 1
                wound_ratio = estimate_wound_ratio(img, x1, y1, x2, y2)
                img = put_korean_text(img, f"상처: {wound_ratio * 100:.0f}%", (x1, y2 + 5), 15, (0, 0, 255))

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            img = put_korean_text(img, f"{korean_label} {confidence:.2f}", (x1, y1 - 20), 16, color)

    overlay_text = f"검사한 사과: {apple_count}개  |  손상된 사과: {defect_count}개"
    cv2.rectangle(img, (0, 0), (img.shape[1], 40), (0, 0, 0), -1)
    img = put_korean_text(img, overlay_text, (10, 10), 18, (255, 255, 255))

    cv2.imshow("Apple Image Analyzer", img)
    print("사진 창이 열렸습니다. 아무 키나 누르면 종료됩니다.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    analyze_apple_image("test2.jpg")