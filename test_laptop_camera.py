"""Laptop-webcam test for the target tracking logic in perception/detect.py.

This runs without ROS, GUI, or a robot. It verifies person Track IDs and
hybrid face + body-appearance target reacquisition with the built-in laptop
camera.
"""

from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


MODEL_PATH = Path(__file__).with_name("yolov8n-pose.pt")
FACE_DETECTOR_PATH = Path(__file__).with_name("face_detection_yunet.onnx")
FACE_RECOGNIZER_PATH = Path(__file__).with_name("face_recognition_sface.onnx")
CAMERA_INDEX = 0
TRACKER_CONFIG = "botsort_reid.yaml"  # with_reid enabled - see that file for why

# Re-entry lighting/angle changes can drop even the true match's score into the 0.4s, so keep the
# absolute floor low and lean on the margin (how clearly the winner beats the runner-up) for safety.
# When no face is available on either side though (e.g. the target has their back turned), body/
# clothing alone can't reliably tell people apart - use a much stricter floor there and prefer
# losing the target over silently following the wrong person.
REACQUIRE_SIMILARITY_THRESHOLD = 0.4
REACQUIRE_THRESHOLD_NO_FACE = 0.65
REACQUIRE_MARGIN = 0.15  # best candidate must beat the runner-up by this much, or we refuse to guess
REACQUIRE_CONFIRM_FRAMES = 2  # a switch to a different track ID must win this many frames in a row
FEATURE_UPDATE_ALPHA = 0.3

# Body score (used whenever no face is visible for one/both sides) = weighted ReID + clothing colour.
APPEARANCE_WEIGHT = 0.6
CLOTHING_WEIGHT = 0.4

# Face score dominates whenever both the target and a candidate have a visible face this frame -
# it's far more identity-specific than clothing/build, which is what kept getting confused between
# people. cv2's SFace uses ~0.363 cosine similarity as its "same person" reference threshold; below
# FACE_MISMATCH_VETO we treat it as a confident non-match and veto the candidate outright, regardless
# of how good their body score looks.
FACE_SCORE_WEIGHT = 0.8
FACE_MISMATCH_VETO = 0.15

_face_recognizer = None  # set once in main(); combined_similarity() reads it for cv2's match()


def _hsv_hist(crop: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], None, [32, 32], [0, 180, 0, 256])
    cv2.normalize(histogram, histogram, 0, 1, cv2.NORM_MINMAX)
    return histogram.flatten()


def extract_clothing_feature(frame: np.ndarray, x1: float, y1: float, x2: float, y2: float) -> np.ndarray:
    """Return an HSV histogram of the person's upper body (shirt colour), matching detect.py."""
    height, width = frame.shape[:2]
    x1, y1 = int(max(x1, 0)), int(max(y1, 0))
    x2, y2 = int(min(x2, width)), int(min(y2, height))
    if x2 <= x1 or y2 <= y1:
        return np.zeros(32 * 32, dtype=np.float32)

    box_height, box_width = y2 - y1, x2 - x1
    upper_y2 = y1 + int(box_height * 0.6)
    inner_x1 = x1 + int(box_width * 0.2)
    inner_x2 = x2 - int(box_width * 0.2)
    crop = frame[y1:upper_y2, inner_x1:inner_x2]
    if crop.size == 0:
        crop = frame[y1:y2, x1:x2]
    return _hsv_hist(crop)


def get_reid_embeddings(model: YOLO) -> dict:
    """Pull BoT-SORT's internal per-track OSNet ReID embeddings (track_id -> L2-normalised vector).

    These aren't exposed through the normal results API, so this reaches into the tracker
    ultralytics attaches to the predictor (model.predictor.trackers[0]). Returns {} if reid is
    unavailable (e.g. with_reid off, or an ultralytics version that changed this internal layout).
    """
    predictor = getattr(model, "predictor", None)
    trackers = getattr(predictor, "trackers", None) if predictor is not None else None
    if not trackers:
        return {}
    embeddings = {}
    for strack in getattr(trackers[0], "tracked_stracks", []):
        feat = getattr(strack, "smooth_feat", None)
        if feat is None:
            feat = getattr(strack, "curr_feat", None)
        if feat is not None:
            embeddings[strack.track_id] = feat
    return embeddings


def load_face_models():
    if not FACE_DETECTOR_PATH.is_file() or not FACE_RECOGNIZER_PATH.is_file():
        return None, None
    detector = cv2.FaceDetectorYN_create(str(FACE_DETECTOR_PATH), "", (320, 320), score_threshold=0.7)
    recognizer = cv2.FaceRecognizerSF_create(str(FACE_RECOGNIZER_PATH), "")
    return detector, recognizer


def detect_faces(detector, frame: np.ndarray) -> np.ndarray:
    """Return YuNet's raw Nx15 rows: [x, y, w, h, 5x(landmark x,y), score]."""
    if detector is None:
        return np.empty((0, 15), dtype=np.float32)
    height, width = frame.shape[:2]
    detector.setInputSize((width, height))
    _, faces = detector.detect(frame)
    return faces if faces is not None else np.empty((0, 15), dtype=np.float32)


def _face_center_in_box(face_row: np.ndarray, x1: float, y1: float, x2: float, y2: float) -> bool:
    fx, fy, fw, fh = face_row[:4]
    cx, cy = fx + fw / 2, fy + fh / 2
    return x1 <= cx <= x2 and y1 <= cy <= y2


def get_face_feature(recognizer, frame: np.ndarray, faces: np.ndarray, x1, y1, x2, y2):
    """Find the face (if any) whose center falls inside this person's box and embed it."""
    if recognizer is None:
        return None
    for face_row in faces:
        if _face_center_in_box(face_row, x1, y1, x2, y2):
            aligned = recognizer.alignCrop(frame, face_row)
            return recognizer.feature(aligned)
    return None


def combined_similarity(target_features, candidate_features) -> float:
    """Face score (when both sides have one) dominates; otherwise ReID + clothing colour.

    A confident face mismatch (both sides have a face, but it's clearly not the same one)
    vetoes the candidate outright (-1.0), regardless of how similar their clothes/build look -
    this is what actually distinguishes two different people wearing similar clothes.
    """
    target_face, target_reid, target_clothing = target_features
    candidate_face, candidate_reid, candidate_clothing = candidate_features

    clothing_score = cv2.compareHist(target_clothing, candidate_clothing, cv2.HISTCMP_CORREL)
    if target_reid is not None and candidate_reid is not None:
        reid_score = float(np.dot(target_reid, candidate_reid))
        body_score = APPEARANCE_WEIGHT * reid_score + CLOTHING_WEIGHT * clothing_score
    else:
        body_score = clothing_score

    if target_face is not None and candidate_face is not None and _face_recognizer is not None:
        face_score = _face_recognizer.match(target_face, candidate_face, cv2.FaceRecognizerSF_FR_COSINE)
        if face_score < FACE_MISMATCH_VETO:
            return -1.0
        return FACE_SCORE_WEIGHT * face_score + (1 - FACE_SCORE_WEIGHT) * body_score

    return body_score


def select_target(people, target_id, target_features, pending, known_other_ids):
    """Same target choice policy as YoloDetectNode._select_target.

    Track-ID continuity is NOT trusted or bonused: BoT-SORT can hand the same ID to
    the wrong person after a crossing/occlusion, and a same-ID bonus would only make
    that worse (it would tip the score toward whoever now wears the old ID, exactly
    the failure this appearance check exists to catch). Every visible person is
    scored purely on appearance (face first, then body) every frame.

    Switching to a different track ID isn't committed on a single frame either -
    a brief crossing can produce one noisy match. `pending` (dict with "id"/"count",
    mutated in place) requires the same candidate to win REACQUIRE_CONFIRM_FRAMES
    times in a row before the target actually changes.

    `known_other_ids` is a set of track IDs proven NOT to be the target (seen in the
    same frame as the target while it was actively tracked - two different track IDs
    in one frame can't be the same physical person). They're never eligible to become
    the target via appearance guessing, no matter how high their score is - this is
    what stops the target from jumping onto whoever's left in frame once it walks out.
    """
    if target_id is None:
        pending["id"], pending["count"] = None, 0
        return max(people, key=lambda person: (person[3] - person[1]) * (person[4] - person[2])), "new", None

    if target_features is None:
        for person in people:
            if person[0] == target_id:
                return person, "tracking", None
        return None, "lost", None

    candidates = [p for p in people if p[0] == target_id or p[0] not in known_other_ids]
    if not candidates:
        pending["id"], pending["count"] = None, 0
        return None, "lost", None

    target_face = target_features[0]
    scored = []
    for person in candidates:
        score = combined_similarity(target_features, (person[5], person[6], person[7]))
        used_face = target_face is not None and person[5] is not None
        scored.append((score, used_face, person))
    scored.sort(key=lambda item: item[0], reverse=True)

    best_score, best_used_face, best_person = scored[0]
    runner_up = scored[1][0] if len(scored) > 1 else -1.0
    threshold = REACQUIRE_SIMILARITY_THRESHOLD if best_used_face else REACQUIRE_THRESHOLD_NO_FACE
    if not (best_score >= threshold and best_score - runner_up >= REACQUIRE_MARGIN):
        pending["id"], pending["count"] = None, 0
        return None, "lost", None

    if best_person[0] == target_id:
        pending["id"], pending["count"] = None, 0
        return best_person, "tracking", best_score

    # Candidate wants to switch the target to a different track ID - require confirmation.
    if best_person[0] == pending["id"]:
        pending["count"] += 1
    else:
        pending["id"], pending["count"] = best_person[0], 1

    if pending["count"] < REACQUIRE_CONFIRM_FRAMES:
        return None, "confirming", best_score

    pending["id"], pending["count"] = None, 0
    return best_person, "reacquired", best_score


def _blend_reid(old_feat, new_feat, alpha):
    if old_feat is None:
        return new_feat
    if new_feat is None:
        return old_feat
    blended = (1 - alpha) * old_feat + alpha * new_feat
    norm = np.linalg.norm(blended)
    return blended / norm if norm > 0 else blended


def _blend_clothing(old_feat, new_feat, alpha):
    blended = (1 - alpha) * old_feat + alpha * new_feat
    cv2.normalize(blended, blended, 0, 1, cv2.NORM_MINMAX)
    return blended


def _blend_face(old_feat, new_feat, alpha):
    if old_feat is None:
        return new_feat
    if new_feat is None:
        return old_feat
    return (1 - alpha) * old_feat + alpha * new_feat


def main() -> None:
    global _face_recognizer

    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"YOLO model not found: {MODEL_PATH}")

    model = YOLO(MODEL_PATH)
    face_detector, _face_recognizer = load_face_models()
    if face_detector is None:
        print("Face model files not found next to this script - running body-appearance only.")

    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not camera.isOpened():
        raise RuntimeError(f"Camera {CAMERA_INDEX} could not be opened. Try CAMERA_INDEX = 1.")

    target_id = None
    target_features = None  # (face_embedding_or_None, reid_embedding_or_None, clothing_feature)
    pending = {"id": None, "count": 0}
    known_other_ids = set()  # track IDs proven NOT to be the target (seen alongside it)
    previous_mode = None
    print("Target tracking test started. Q/Esc: quit, R: clear selected target.")
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Could not read a frame from the camera.")

            result = model.track(
                frame, persist=True, classes=[0], tracker=TRACKER_CONFIG, verbose=False, imgsz=480
            )[0]
            reid_embeddings = get_reid_embeddings(model)
            faces = detect_faces(face_detector, frame)

            people = []
            face_hits = 0
            if result.boxes is not None:
                for box in result.boxes:
                    if box.id is None:
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
                    track_id = int(box.id[0])
                    clothing_feature = extract_clothing_feature(frame, x1, y1, x2, y2)
                    reid_feature = reid_embeddings.get(track_id)
                    face_feature = get_face_feature(_face_recognizer, frame, faces, x1, y1, x2, y2)
                    face_hits += face_feature is not None
                    people.append((track_id, x1, y1, x2, y2, face_feature, reid_feature, clothing_feature))

            # If the target is genuinely present this frame, everyone else visible right now
            # is proven to not be the target (different simultaneous track IDs = different people).
            ids_this_frame = {p[0] for p in people}
            if target_id is not None and target_id in ids_this_frame:
                known_other_ids.update(ids_this_frame - {target_id})

            chosen = None
            mode = "no person"
            similarity = None
            if people:
                chosen, mode, similarity = select_target(people, target_id, target_features, pending, known_other_ids)
                if chosen is not None:
                    is_new_identity = chosen[0] != target_id
                    target_id = chosen[0]
                    new_features = (chosen[5], chosen[6], chosen[7])
                    if target_features is None or is_new_identity:
                        # New identity confirmed - replace outright instead of blending with a
                        # profile that might have drifted toward the wrong person.
                        target_features = new_features
                    else:
                        old_face, old_reid, old_clothing = target_features
                        new_face, new_reid, new_clothing = new_features
                        target_features = (
                            _blend_face(old_face, new_face, FEATURE_UPDATE_ALPHA),
                            _blend_reid(old_reid, new_reid, FEATURE_UPDATE_ALPHA),
                            _blend_clothing(old_clothing, new_clothing, FEATURE_UPDATE_ALPHA),
                        )

            if mode != previous_mode:
                detail = "" if similarity is None else f" (score={similarity:.2f})"
                print(f"Target status: {mode}{detail} | faces seen this frame: {face_hits}")
                previous_mode = mode

            for person in people:
                track_id, x1, y1, x2, y2, face_feature, _, _ = person
                is_target = chosen is not None and track_id == chosen[0]
                color = (0, 255, 0) if is_target else (0, 180, 255)
                face_tag = "+face" if face_feature is not None else ""
                label = f"TARGET ID {track_id}{face_tag}" if is_target else f"person ID {track_id}{face_tag}"
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, label, (int(x1), max(25, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

            message = f"{mode.upper()} | target ID: {target_id if target_id is not None else '-'} | R: reset | Q: quit"
            cv2.putText(frame, message, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.imshow("Laptop camera - target tracking test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                target_id, target_features, previous_mode = None, None, None
                pending["id"], pending["count"] = None, 0
                known_other_ids.clear()
                print("Target cleared. The largest tracked person will be selected again.")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
