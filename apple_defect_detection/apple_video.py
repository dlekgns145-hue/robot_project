from ultralytics import YOLO
import cv2
import os
import time
import numpy as np
import gc
import glob

# 현재 폴더 경로 자동 인식
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
TRACKER_CONFIG_PATH = os.path.join(CURRENT_DIR, "my_tracker.yaml")

# 클래스별 색상 (B, G, R 순서)
CLASS_COLORS = {
    "apple": (0, 255, 0),
    "damaged_apple": (0, 0, 255),
}

# 손상 판별 신뢰도 임계값
DEFECT_CONF_THRESHOLD = 0.5
DEFAULT_PROCESS_EVERY_N_FRAMES = 2
DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720
MAX_FRAME_SIZE = 1920  # 이 이상이면 자동 다운샘플링


def auto_scale_frame(frame, max_width=MAX_FRAME_SIZE):
    """프레임 크기가 너무 크면 비율 유지하며 축소해 성능 개선"""
    h, w = frame.shape[:2]
    if w > max_width:
        scale = max_width / w
        new_w = max_width
        new_h = max(1, int(h * scale))
        return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    return frame


def resize_for_display(frame, target_w=DISPLAY_WIDTH, target_h=DISPLAY_HEIGHT):
    """영상 비율을 유지한 채 지정 크기 창에 맞춰 보여주기 (letterbox 방식)"""
    h, w = frame.shape[:2]
    scale = min(target_w / w, target_h / h)
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas = np.zeros((target_h, target_w, 3), dtype=frame.dtype)
    offset_x = (target_w - new_w) // 2
    offset_y = (target_h - new_h) // 2
    canvas[offset_y:offset_y + new_h, offset_x:offset_x + new_w] = resized
    return canvas


def estimate_wound_ratio(frame, box_xyxy):
    """박스 안에서 상처 영역 비율을 추정한다 (자동 임계값 조정)."""
    x1, y1, x2, y2 = map(int, box_xyxy)
    x1, y1, x2, y2 = max(0, x1), max(0, y1), max(0, x2), max(0, y2)

    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return 0.0, None, "healthy"

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    
    # Otsus 자동 이진화로 조명에 적응
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # 어두운 영역만 추출 (동적 임계값)
    threshold_val = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)[0]
    lower_dark = np.array([0, 0, max(0, threshold_val - 60)], dtype=np.uint8)
    upper_dark = np.array([180, 255, threshold_val + 30], dtype=np.uint8)
    mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)

    kernel_size = min(5, max(3, crop.shape[0] // 50))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask_dark = cv2.morphologyEx(mask_dark, cv2.MORPH_OPEN, kernel)
    mask_dark = cv2.morphologyEx(mask_dark, cv2.MORPH_CLOSE, kernel)

    wound_pixels = cv2.countNonZero(mask_dark)
    total_pixels = crop.shape[0] * crop.shape[1]
    ratio = wound_pixels / total_pixels if total_pixels > 0 else 0.0

    # 다중 분류 (healthy / minor / severe)
    if ratio < 0.15:
        damage_level = "healthy"
    elif ratio < 0.40:
        damage_level = "minor"
    else:
        damage_level = "severe"

    highlight = crop.copy()
    contours, _ = cv2.findContours(mask_dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 20:
            cv2.drawContours(highlight, [cnt], -1, (0, 0, 255), 2)

    return ratio, highlight, damage_level


def analyze_apple_video_file(video_name, show_window=True, process_every_n_frames=DEFAULT_PROCESS_EVERY_N_FRAMES, save_output=False):
    if process_every_n_frames < 1:
        process_every_n_frames = 1

    print(f"\n[시작] '{video_name}' 영상 분석 중...")
    model = YOLO("apple_defect_v5_local.pt")

    video_path = os.path.join(CURRENT_DIR, video_name)
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"'{video_name}' 영상을 찾을 수 없습니다.")
        return {"status": "failed", "video": video_name}

    fps = 0.0
    frame_idx = 0
    processed_frames = 0
    last_fps_time = time.perf_counter()
    checked_apple_ids = set()
    defective_apple_ids = set()
    prev_results = None
    current_conf_threshold = DEFECT_CONF_THRESHOLD
    adaptive_step = DEFAULT_PROCESS_EVERY_N_FRAMES
    start_time = time.perf_counter()

    if save_output:
        output_path = os.path.join(CURRENT_DIR, f"{os.path.splitext(video_name)[0]}_annotated.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        writer = cv2.VideoWriter(output_path, fourcc, 20.0, (width, height))
    else:
        writer = None

    if show_window:
        cv2.namedWindow("Orchard Video Analyzer", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Orchard Video Analyzer", DISPLAY_WIDTH, DISPLAY_HEIGHT)
        print("키 조작: U/D = 임계값 ±0.05 | Q = 종료")

    print(f"분석 시작: {video_name}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = auto_scale_frame(frame, MAX_FRAME_SIZE)
        frame_apple_count = 0
        frame_defect_count = 0
        best_wound_ratio = -1.0
        best_wound_crop = None

        should_process = (frame_idx % adaptive_step == 0)
        if should_process:
            results = model.track(
                source=frame,
                conf=current_conf_threshold,
                iou=0.45,
                agnostic_nms=True,
                persist=True,
                tracker=TRACKER_CONFIG_PATH,
                save=False,
                verbose=False,
            )
            prev_results = results
            processed_frames += 1
        else:
            results = prev_results
        
        # 적응형 프레임 스킵: FPS 기반 자동 조정
        if fps > 20:
            adaptive_step = 3  # 빠르면 더 스킵
        elif fps > 15:
            adaptive_step = 2
        else:
            adaptive_step = 1  # 느리면 매 프레임

        if results is not None and len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    frame_apple_count += 1

                    track_id = int(box.id[0]) if box.id is not None else None
                    cls_id = int(box.cls[0])
                    label = model.names[cls_id]
                    confidence = float(box.conf[0])
                    color = CLASS_COLORS.get(label, (0, 255, 0))
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    is_new_apple = track_id is not None and track_id not in checked_apple_ids
                    if is_new_apple and confidence >= DEFECT_CONF_THRESHOLD:
                        checked_apple_ids.add(track_id)
                        if label == "damaged_apple":
                            defective_apple_ids.add(track_id)

                    if label == "damaged_apple":
                        frame_defect_count += 1
                        wound_ratio, wound_crop, damage_level = estimate_wound_ratio(frame, box.xyxy[0])
                        
                        # Damage level display with different colors
                        if damage_level == "severe":
                            color_text = (0, 0, 255)  # Red
                            status_text = "Need Review (Severe)"
                        elif damage_level == "minor":
                            color_text = (0, 165, 255)  # Orange
                            status_text = "Need Review"
                        else:
                            color_text = (0, 255, 0)  # Green
                            status_text = "Normal"
                        
                        cv2.putText(frame, f"{status_text} {wound_ratio * 100:.0f}%", (x1, y2 + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_text, 2)

                        if wound_crop is not None and wound_ratio > best_wound_ratio:
                            best_wound_ratio = wound_ratio
                            best_wound_crop = wound_crop

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    display_label = "Need Review" if label == "damaged_apple" else label
                    cv2.putText(frame, f"{display_label} {confidence:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        overlay_text_1 = f"Apples: {frame_apple_count} | Need Review: {frame_defect_count}"
        overlay_text_2 = f"Checked: {len(checked_apple_ids)} | Need Review: {len(defective_apple_ids)}"
        overlay_text_3 = f"FPS: {fps:.1f} | Conf: {current_conf_threshold:.2f} | Adaptive Step: {adaptive_step}"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 78), (0, 0, 0), -1)
        cv2.putText(frame, overlay_text_1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, overlay_text_2, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, overlay_text_3, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if writer is not None:
            writer.write(frame)

        if show_window:
            display_frame = resize_for_display(frame, DISPLAY_WIDTH, DISPLAY_HEIGHT)
            
            if best_wound_crop is not None:
                panel_size = 200
                panel_x = DISPLAY_WIDTH - panel_size - 15
                panel_y = DISPLAY_HEIGHT - panel_size - 15
                
                crop_resized = cv2.resize(best_wound_crop, (panel_size, panel_size))
                display_frame[panel_y:panel_y + panel_size, panel_x:panel_x + panel_size] = crop_resized
                cv2.rectangle(display_frame, (panel_x, panel_y), (panel_x + panel_size, panel_y + panel_size), (0, 0, 255), 3)
                cv2.putText(display_frame, f"Worst: {best_wound_ratio * 100:.0f}%", (panel_x, panel_y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.imshow("Orchard Video Analyzer", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("u"):
                current_conf_threshold = min(0.95, current_conf_threshold + 0.05)
                print(f"임계값 증가: {current_conf_threshold:.2f}")
            elif key == ord("d"):
                current_conf_threshold = max(0.05, current_conf_threshold - 0.05)
                print(f"임계값 감소: {current_conf_threshold:.2f}")

        frame_idx += 1
        elapsed = time.perf_counter() - last_fps_time
        if elapsed >= 0.5:
            fps = processed_frames / elapsed
            processed_frames = 0
            last_fps_time = time.perf_counter()

        if frame_idx % 200 == 0:
            gc.collect()

    cap.release()
    if writer is not None:
        writer.release()
        print(f"영상 저장 완료: {output_path}")
    if show_window:
        cv2.destroyAllWindows()

    # Calculate total elapsed time
    elapsed_time = time.perf_counter() - start_time
    total_apples = len(checked_apple_ids)
    damaged_apples = len(defective_apple_ids)
    damage_rate = (damaged_apples / total_apples * 100) if total_apples > 0 else 0

    print("-" * 50)
    print(f"완료: {video_name}")
    print(f"  • 검사한 사과: {total_apples}개")
    print(f"  • 손상된 사과: {damaged_apples}개")
    if total_apples > 0:
        print(f"  • 손상률: {damage_rate:.1f}%")
    print(f"  • 처리 시간: {elapsed_time:.2f}초")
    print(f"  • 평균 FPS: {frame_idx/elapsed_time if elapsed_time > 0 else 0:.2f}")
    print("-" * 50)
    
    return {
        "status": "success",
        "video": video_name,
        "total_apples": total_apples,
        "damaged_apples": damaged_apples,
        "damage_rate": damage_rate,
        "elapsed_time": elapsed_time,
        "avg_fps": frame_idx/elapsed_time if elapsed_time > 0 else 0
    }


def process_all_videos_in_folder(folder_path=CURRENT_DIR, show_window=True, process_every_n_frames=DEFAULT_PROCESS_EVERY_N_FRAMES):
    """폴더 내 모든 영상을 배치 처리"""
    video_files = glob.glob(os.path.join(folder_path, "*.mp4")) + \
                  glob.glob(os.path.join(folder_path, "*.avi")) + \
                  glob.glob(os.path.join(folder_path, "*.mov"))
    
    if not video_files:
        print(f"'{folder_path}' 폴더에 영상 파일이 없습니다.")
        return

    print(f"\n{'='*50}")
    print(f"배치 처리 시작: {len(video_files)}개의 영상")
    print(f"{'='*50}\n")

    results = []
    for video_file in sorted(video_files):
        video_name = os.path.basename(video_file)
        result = analyze_apple_video_file(
            video_name,
            show_window=show_window,
            process_every_n_frames=process_every_n_frames,
            save_output=False
        )
        results.append(result)
        gc.collect()

    print(f"\n{'='*50}")
    print("배치 처리 완료 요약")
    print(f"{'='*50}")
    total_apples = 0
    total_damaged = 0
    for result in results:
        if result["status"] == "success":
            print(f"{result['video']:30} | 사과: {result['total_apples']:3}개 | 재검사필요: {result['damaged_apples']:3}개")
            total_apples += result["total_apples"]
            total_damaged += result["damaged_apples"]
    print(f"{'='*50}")
    if total_apples > 0:
        print(f"전체 합계: 사과 {total_apples}개 중 손상 {total_damaged}개 ({total_damaged/total_apples*100:.1f}%)")
    else:
        print("처리된 영상이 없습니다.")
    print(f"{'='*50}\n")


if __name__ == "__main__":
    # 단일 분석
    # analyze_apple_video_file("apple_a.mp4", show_window=True, process_every_n_frames=2)
    
    # 배치 처리 (모든 영상)
    process_all_videos_in_folder(CURRENT_DIR, show_window=True, process_every_n_frames=2)
