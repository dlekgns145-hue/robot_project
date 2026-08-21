#!/usr/bin/env python3
"""
YOLO 기반 사람 인식 노드 (detect.py) - 확정 버전
--------------------------------------------------
카메라 이미지를 받아 YOLO 모델로 사람을 탐지하고,
탐지 결과(중심좌표, 박스 크기)를 /person_detection 토픽으로 발행한다.

타겟(따라갈 사람) 선정 방식:
  0. 화면에 사람이 딱 한 명뿐이면 헷갈릴 대상이 아예 없으므로 얼굴/외형/옷색깔
     매칭(아래 2번)을 전부 건너뛰고 그 사람을 곧바로 타겟으로 삼는다. YOLO+트래킹은
     인원수를 알아야 하니 어차피 매 프레임 돌지만, 무거운 나머지 단계는 실제로
     구분이 필요한 상황(사람이 2명 이상 잡힘)에서만 켠다.
  1. 처음엔 화면에서 가장 가까운(박스가 가장 큰) 사람을 타겟으로 지정한다.
  2. 사람이 2명 이상 잡히면 매 프레임마다 화면에 보이는 모든 사람에게 가중치 점수를 매긴다 (트랙 ID는
     신뢰하지 않는다 - BoT-SORT가 겹침/교차 상황에서 ID를 엉뚱한 사람에게 붙이는 경우가
     있어서, ID가 같다고 가산점을 주면 그 오류를 오히려 못 잡아낸다):
       - 얼굴 점수: OpenCV YuNet(검출)+SFace(인식) 얼굴 임베딩 코사인 유사도
         - 타겟과 후보 둘 다 얼굴이 보일 때만 계산되고, 이때는 이 점수가 최우선이다
           (옷/체형보다 사람을 훨씬 정확하게 구분한다). 확실히 다른 얼굴이면
           몸 점수가 아무리 높아도 그 후보를 바로 탈락시킨다(거부권).
       - 외형 점수: BoT-SORT가 내부적으로 계산하는 OSNet ReID 임베딩(코사인 유사도)
         - 단순 색상보다 옷 재질/체형까지 반영해 구분력이 높다
       - 옷 색깔 점수: 상체(위 60%) HSV 색상 히스토그램 유사도
       - 얼굴이 없으면 외형가중치×외형점수 + 옷색깔가중치×옷색깔점수로 대체한다
         (사람이 뒤돌아 있는 등 얼굴이 안 보이는 프레임을 위한 폴백)
     점수가 가장 높은 사람이 threshold를 넘고 2등과 확실히 벌어져 있을 때만
     그 사람을 타겟으로 인정한다. (애매하면 이번 프레임은 '사람 없음'으로 처리해 오인식을 피한다)
  3. 타겟과 동시에 화면에 잡혔던 다른 사람은 "확실히 타겟이 아님"으로 확정해 블랙리스트에
     올린다 - 나중에 타겟이 사라져도 이 사람들은 재획득 후보에서 아예 제외된다.
  4. 트랙 ID가 바뀌는 재획득은 같은 후보가 reacquire_confirm_frames 프레임 연속으로
     나와야 확정한다 (스쳐 지나가는 애매한 프레임 한 번으로는 안 바뀜).

발행 메시지 포맷 (follow_person.py와 호환 - 변경 없음):
    [found, center_x, center_y, box_width, box_height, frame_width, frame_height]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'perception/best.pt')
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('tracker_config', 'perception/botsort_reid.yaml')  # with_reid 켠 설정 (botsort.yaml은 외형 안 봐서 ID 스위칭 잦음)
        self.declare_parameter('face_detector_path', 'perception/face_detection_yunet.onnx')
        self.declare_parameter('face_recognizer_path', 'perception/face_recognition_sface.onnx')
        # 재진입 시 조명/각도 차이로 같은 사람도 절대 점수가 낮게 나올 수 있어서(예: 0.4대),
        # 절대 기준은 낮게 잡고 대신 "1등이 2등을 얼마나 확실히 이기는가(마진)"로 안전성을 확보한다.
        # 단, 얼굴이 안 보여서 몸 점수만으로 판단하는 경우는 훨씬 엄격한 기준을 쓴다 - 몸/옷만으로는
        # 비슷한 사람과 구분이 잘 안 되므로, 애매하면 엉뚱한 사람을 따라가느니 "놓침" 처리하는 쪽을 택한다.
        self.declare_parameter('reacquire_similarity_threshold', 0.4)  # 재획득 최소 점수 - 얼굴 매칭 있을 때(-1~1)
        self.declare_parameter('reacquire_threshold_no_face', 0.65)  # 재획득 최소 점수 - 몸 점수만 있을 때(뒷모습 등)
        self.declare_parameter('reacquire_margin', 0.15)  # 1등이 2등보다 이만큼은 앞서야 재획득 (애매하면 포기)
        self.declare_parameter('reacquire_confirm_frames', 2)  # 새 track_id로 전환하려면 몇 프레임 연속으로 같은 후보가 나와야 하는지
        self.declare_parameter('feature_update_alpha', 0.3)  # 타겟 외형 특징 갱신 비율(같은 트랙 계속 추적 중일 때만 적용)
        self.declare_parameter('appearance_weight', 0.6)  # 외형(ReID 임베딩) 점수 가중치 (얼굴이 없을 때)
        self.declare_parameter('clothing_weight', 0.4)    # 옷 색깔(상체 HSV) 점수 가중치 (얼굴이 없을 때)
        self.declare_parameter('face_score_weight', 0.8)  # 얼굴이 둘 다 보일 때 얼굴 점수 가중치 (나머지는 몸 점수)
        self.declare_parameter('face_mismatch_veto', 0.15)  # 얼굴 유사도가 이보다 낮으면 몸 점수와 무관하게 그 후보를 탈락시킴

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.show_debug = self.get_parameter('show_debug_window').get_parameter_value().bool_value
        self.tracker_config = self.get_parameter('tracker_config').get_parameter_value().string_value
        face_detector_path = self.get_parameter('face_detector_path').get_parameter_value().string_value
        face_recognizer_path = self.get_parameter('face_recognizer_path').get_parameter_value().string_value
        self.reacquire_threshold = self.get_parameter('reacquire_similarity_threshold').value
        self.reacquire_threshold_no_face = self.get_parameter('reacquire_threshold_no_face').value
        self.reacquire_margin = self.get_parameter('reacquire_margin').value
        self.reacquire_confirm_frames = self.get_parameter('reacquire_confirm_frames').value
        self.feature_alpha = self.get_parameter('feature_update_alpha').value
        self.appearance_weight = self.get_parameter('appearance_weight').value
        self.clothing_weight = self.get_parameter('clothing_weight').value
        self.face_score_weight = self.get_parameter('face_score_weight').value
        self.face_mismatch_veto = self.get_parameter('face_mismatch_veto').value

        self.bridge = CvBridge()

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO 모델 로드 완료 (model_path={model_path})')

        self.face_detector, self.face_recognizer = self._load_face_models(
            face_detector_path, face_recognizer_path
        )
        if self.face_detector is None:
            self.get_logger().warning(
                f'얼굴 모델을 찾을 수 없어 외형/옷색깔 점수만 사용합니다 '
                f'(face_detector_path={face_detector_path})'
            )

        self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/person_detection', 10)
        # Loading the model is cheap compared with running inference on every
        # frame. Keep inference asleep until follow mode explicitly resets a
        # target, and let mapping pause it without stopping the container.
        self.enabled = False

        # ---- 타겟(따라갈 사람) 상태 ----
        self.target_id = None          # BoT-SORT가 부여한 track id
        self.target_features = None    # (얼굴 임베딩 또는 None, ReID 임베딩 또는 None, 옷 색깔 히스토그램)
        self.pending_switch_id = None  # 새 타겟으로 바뀌려고 대기 중인 track id (연속 확인용)
        self.pending_switch_count = 0
        self.known_other_ids = set()   # 타겟과 동시에 잡혀서 "확실히 타겟이 아님"이 확정된 track id들

        # "따라와" 트리거가 들어올 때마다 호출됨 -- 타겟 상태를 초기화해서 다음 프레임에
        # 카메라에서 제일 가까운 사람을 새 타겟으로 다시 잡게 한다 (말한 사람 = 제일 가까운
        # 사람이라고 가정. 마이크 방향으로 화자를 특정하는 기능은 없음).
        self.create_service(Trigger, '/person_detection/reset_target', self._handle_reset_target)
        self.create_service(Trigger, '/person_detection/pause', self._handle_pause)

        self.get_logger().info(f'YOLO Detect Node 시작됨 (camera_topic={camera_topic})')

    def _handle_reset_target(self, request, response):
        self.enabled = True
        self.target_id = None
        self.target_features = None
        self.pending_switch_id = None
        self.pending_switch_count = 0
        self.known_other_ids = set()
        self.get_logger().info('타겟 리셋됨 -- 다음 프레임에서 가장 가까운 사람을 새로 잡음')
        response.success = True
        response.message = 'target reset'
        return response

    def _handle_pause(self, request, response):
        self.enabled = False
        self.target_id = None
        self.target_features = None
        self.pending_switch_id = None
        self.pending_switch_count = 0
        self.known_other_ids = set()
        self.get_logger().info('사람 인식 일시정지 -- 매핑/유휴 CPU 확보')
        response.success = True
        response.message = 'person detection paused'
        return response

    @staticmethod
    def _load_face_models(detector_path, recognizer_path):
        import os
        if not os.path.isfile(detector_path) or not os.path.isfile(recognizer_path):
            return None, None
        detector = cv2.FaceDetectorYN_create(detector_path, '', (320, 320), score_threshold=0.7)
        recognizer = cv2.FaceRecognizerSF_create(recognizer_path, '')
        return detector, recognizer

    def image_callback(self, msg: Image):
        if not self.enabled:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        found, cx, cy, bw, bh = self.run_inference(frame)

        result = Float32MultiArray()
        result.data = [float(found), float(cx), float(cy), float(bw), float(bh), float(w), float(h)]
        self.pub.publish(result)

        if self.show_debug:
            if found:
                cv2.rectangle(
                    frame,
                    (int(cx - bw / 2), int(cy - bh / 2)),
                    (int(cx + bw / 2), int(cy + bh / 2)),
                    (0, 255, 0), 2,
                )
            cv2.imshow('YOLO Detect Debug', frame)
            cv2.waitKey(1)

    def run_inference(self, frame):
        # The camera's native frame is 320x240. Without an explicit imgsz,
        # ultralytics defaults to 640 and upscales into it before every
        # inference -- pure waste, since upscaling a 320-wide source can't
        # add real detail, and it was costing ~3x the compute for it
        # (profiled on the real robot, 2026-08-20: 762ms/frame at the
        # default 640 vs 253ms/frame at 320, same detections). This was the
        # dominant cost in the whole pipeline by a wide margin -- face
        # detection/recognition and the clothing histogram are single-digit
        # to low-double-digit ms by comparison.
        results = self.model.track(
            frame, persist=True, classes=[0], tracker=self.tracker_config,
            verbose=False, imgsz=320,
        )[0]

        # 얼굴/외형/옷색깔 추출은 뒤에서 실제로 필요할 때만 한다 (아래 1명 케이스
        # 참고) -- 박스 좌표와 track_id만 먼저 뽑아둔다.
        boxes = []
        for unconfirmed_index, box in enumerate(results.boxes):
            if box.id is None:
                # BoT-SORT hasn't confirmed a persistent track yet -- expected
                # on a fresh track, and near-constant when frame processing is
                # slow (CPU-bound YOLO+ReID+face-rec on a Pi, ~1 fps observed)
                # since large motion between processed frames keeps breaking
                # track association before it confirms. Don't drop the
                # detection: identity re-check is purely appearance-based, not
                # track ID (see _select_target's docstring), so an
                # unstable/placeholder ID here doesn't weaken identity
                # tracking -- it only used to block otherwise-good detections
                # outright (found on the real robot, 2026-08-20: a
                # clearly-visible person never registered because box.id was
                # never confirmed).
                track_id = -1 - unconfirmed_index
            else:
                track_id = int(box.id[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            boxes.append((track_id, x1, y1, x2, y2))

        if not boxes:
            return 0, 0, 0, 0, 0

        if len(boxes) == 1:
            # 화면에 사람이 딱 한 명뿐이면 헷갈릴 상대가 없으므로 얼굴 검출/인식,
            # ReID, 옷 색깔 히스토그램을 전부 건너뛰고 곧바로 그 사람을 타겟으로
            # 삼는다 -- 이게 파이프라인에서 YOLO 다음으로 무거운 부분들이라, 흔한
            # "사람 한 명" 상황에서 매 프레임 CPU를 크게 아낀다. target_features/
            # known_other_ids는 그대로 남겨둬서, 나중에 2번째 사람이 나타나
            # 특정인물 로직이 다시 켜질 때 마지막으로 확인된 외형 정보를 바로
            # 쓸 수 있게 한다.
            track_id, x1, y1, x2, y2 = boxes[0]
            self.target_id = track_id
            self.pending_switch_id, self.pending_switch_count = None, 0
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            bw, bh = x2 - x1, y2 - y1
            return 1, cx, cy, bw, bh

        # 사람이 2명 이상 -> 누가 타겟인지 구분해야 하므로 특정인물 식별 로직(얼굴/
        # 외형/옷색깔)을 켠다.
        reid_embeddings = self._get_reid_embeddings()
        faces = self._detect_faces(frame)

        people = []  # [(track_id, x1, y1, x2, y2, face_feature_or_None, reid_feature_or_None, clothing_feature)]
        for track_id, x1, y1, x2, y2 in boxes:
            clothing_feature = self._extract_clothing_feature(frame, x1, y1, x2, y2)
            reid_feature = reid_embeddings.get(track_id)
            face_feature = self._get_face_feature(frame, faces, x1, y1, x2, y2)
            people.append((track_id, x1, y1, x2, y2, face_feature, reid_feature, clothing_feature))

        # 타겟이 이 프레임에 실제로 잡혀 있다면, 동시에 같이 잡힌 다른 track_id는
        # "절대 타겟이 아닌 사람"으로 확정 -> 나중에 타겟이 사라져도 얘네는 재획득 후보에서 제외한다.
        ids_this_frame = {p[0] for p in people}
        if self.target_id is not None and self.target_id in ids_this_frame:
            self.known_other_ids.update(ids_this_frame - {self.target_id})

        chosen = self._select_target(people)
        if chosen is None:
            return 0, 0, 0, 0, 0

        track_id, x1, y1, x2, y2, face_feature, reid_feature, clothing_feature = chosen
        is_new_identity = track_id != self.target_id
        self._lock_target(track_id, face_feature, reid_feature, clothing_feature, replace=is_new_identity)

        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        bw = x2 - x1
        bh = y2 - y1
        return 1, cx, cy, bw, bh

    def _select_target(self, people):
        """매 프레임 외형(얼굴 우선, 없으면 몸)만으로 타겟을 재확인한다 (트랙 ID는 힌트로도 안 씀).

        BoT-SORT가 사람이 겹치거나 지나칠 때 트랙 ID를 엉뚱한 사람에게 붙이는 경우가
        있어서, ID가 같다고 가산점을 주면 오히려 그 오류를 못 잡아낸다 - 그래서 순수
        외형 점수로만 채점한다.

        다른 track_id로 갈아타는 결정은 스쳐 지나가는 애매한 프레임 한 번의 착각일 수
        있어서 바로 확정하지 않고, 같은 후보가 reacquire_confirm_frames 프레임 연속으로
        나와야만 실제로 타겟을 바꾼다.
        """
        if self.target_id is None:
            # 아직 타겟이 없으면 가장 가까운(박스가 큰) 사람을 최초 타겟으로 지정
            self.pending_switch_id, self.pending_switch_count = None, 0
            return max(people, key=lambda p: (p[3] - p[1]) * (p[4] - p[2]))

        if self.target_features is None:
            # 외형 정보가 아직 없는 극초반 -> 트랙 ID만으로 판단
            for person in people:
                if person[0] == self.target_id:
                    return person
            return None

        # 타겟과 동시에 잡혀서 "확실히 타겟이 아님"이 확정된 사람은 재획득 후보에서 아예 제외
        candidates = [p for p in people if p[0] == self.target_id or p[0] not in self.known_other_ids]
        if not candidates:
            self.pending_switch_id, self.pending_switch_count = None, 0
            return None

        target_face = self.target_features[0]
        scored = []
        for person in candidates:
            score = self._combined_similarity(self.target_features, (person[5], person[6], person[7]))
            used_face = target_face is not None and person[5] is not None
            scored.append((score, used_face, person))
        scored.sort(key=lambda item: item[0], reverse=True)

        best_score, best_used_face, best_person = scored[0]
        runner_up = scored[1][0] if len(scored) > 1 else -1.0
        # 얼굴로 확인된 경우엔 관대한 기준, 몸 점수만으로 판단할 땐(뒷모습 등) 훨씬 엄격한 기준을 쓴다
        threshold = self.reacquire_threshold if best_used_face else self.reacquire_threshold_no_face
        # 1등이 threshold를 넘고, 2등과도 확실히 벌어져 있어야만 후보로 인정 (애매하면 오인식 방지 위해 포기)
        if not (best_score >= threshold and best_score - runner_up >= self.reacquire_margin):
            self.pending_switch_id, self.pending_switch_count = None, 0
            return None  # 확실히 같은 사람이라 보기 어려움 -> 이번 프레임은 '없음'

        if best_person[0] == self.target_id:
            # 원래 타겟과 같은 트랙 ID로 계속 이어짐 -> 바로 인정, 대기 상태 초기화
            self.pending_switch_id, self.pending_switch_count = None, 0
            return best_person

        # 다른 track_id로 갈아타려는 상황 -> 연속으로 같은 후보가 나올 때만 확정
        if best_person[0] == self.pending_switch_id:
            self.pending_switch_count += 1
        else:
            self.pending_switch_id, self.pending_switch_count = best_person[0], 1

        if self.pending_switch_count < self.reacquire_confirm_frames:
            return None  # 아직 확정 전 -> 이번 프레임은 '없음'으로 처리, 다음 프레임에 다시 확인

        self.get_logger().info(
            f'타겟 재획득 (새 track_id={best_person[0]}, score={best_score:.2f})'
        )
        self.pending_switch_id, self.pending_switch_count = None, 0
        return best_person

    def _lock_target(self, track_id, face_feature, reid_feature, clothing_feature, replace=False):
        self.target_id = track_id
        if self.target_features is None or replace:
            # 새 사람으로 확정된 경우 이전(오염됐을 수 있는) 특징과 섞지 않고 그대로 교체
            self.target_features = (face_feature, reid_feature, clothing_feature)
        else:
            old_face, old_reid, old_clothing = self.target_features
            self.target_features = (
                self._blend_face(old_face, face_feature),
                self._blend_reid(old_reid, reid_feature),
                self._blend_clothing(old_clothing, clothing_feature),
            )

    def _blend_face(self, old_feat, new_feat):
        if old_feat is None:
            return new_feat
        if new_feat is None:
            return old_feat
        return (1 - self.feature_alpha) * old_feat + self.feature_alpha * new_feat

    def _blend_reid(self, old_feat, new_feat):
        if old_feat is None:
            return new_feat
        if new_feat is None:
            return old_feat
        blended = (1 - self.feature_alpha) * old_feat + self.feature_alpha * new_feat
        norm = np.linalg.norm(blended)
        return blended / norm if norm > 0 else blended

    def _blend_clothing(self, old_feat, new_feat):
        blended = (1 - self.feature_alpha) * old_feat + self.feature_alpha * new_feat
        cv2.normalize(blended, blended, 0, 1, cv2.NORM_MINMAX)
        return blended

    def _get_reid_embeddings(self):
        """BoT-SORT가 내부적으로 계산한 트랙별 OSNet ReID 임베딩을 꺼내온다 (track_id -> L2정규화 벡터).

        일반 results API로는 노출되지 않아서 predictor에 붙은 트래커 내부 상태를 직접 읽는다.
        with_reid가 꺼져 있거나 ultralytics 내부 구조가 바뀌면 빈 dict를 반환한다(그러면
        _combined_similarity가 옷 색깔 점수만으로 자동 대체한다).
        """
        predictor = getattr(self.model, 'predictor', None)
        trackers = getattr(predictor, 'trackers', None) if predictor is not None else None
        if not trackers:
            return {}
        embeddings = {}
        for strack in getattr(trackers[0], 'tracked_stracks', []):
            feat = getattr(strack, 'smooth_feat', None)
            if feat is None:
                feat = getattr(strack, 'curr_feat', None)
            if feat is not None:
                embeddings[strack.track_id] = feat
        return embeddings

    def _detect_faces(self, frame):
        """YuNet으로 프레임의 모든 얼굴을 검출한다. 각 행은 [x, y, w, h, 랜드마크 5쌍, score]."""
        if self.face_detector is None:
            return np.empty((0, 15), dtype=np.float32)
        h, w = frame.shape[:2]
        self.face_detector.setInputSize((w, h))
        _, faces = self.face_detector.detect(frame)
        return faces if faces is not None else np.empty((0, 15), dtype=np.float32)

    def _get_face_feature(self, frame, faces, x1, y1, x2, y2):
        """이 사람 박스 안에 중심이 들어오는 얼굴을 찾아 SFace 임베딩을 추출한다."""
        if self.face_recognizer is None:
            return None
        for face_row in faces:
            fx, fy, fw, fh = face_row[:4]
            cx, cy = fx + fw / 2, fy + fh / 2
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                aligned = self.face_recognizer.alignCrop(frame, face_row)
                return self.face_recognizer.feature(aligned)
        return None

    @staticmethod
    def _hsv_hist(crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [32, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist.flatten()

    @classmethod
    def _extract_clothing_feature(cls, frame, x1, y1, x2, y2):
        """옷 색깔 점수용(상체) HSV 히스토그램을 뽑는다."""
        h, w, _ = frame.shape
        x1, y1 = int(max(x1, 0)), int(max(y1, 0))
        x2, y2 = int(min(x2, w)), int(min(y2, h))
        if x2 <= x1 or y2 <= y1:
            return np.zeros(32 * 32, dtype=np.float32)

        box_h, box_w = y2 - y1, x2 - x1
        ty2 = y1 + int(box_h * 0.6)          # 상체(위 60%)
        tx1 = x1 + int(box_w * 0.2)          # 좌우 20%씩 제외
        tx2 = x2 - int(box_w * 0.2)
        crop = frame[y1:ty2, tx1:tx2] if tx2 > tx1 else frame[y1:y2, x1:x2]
        if crop.size == 0:
            crop = frame[y1:y2, x1:x2]
        return cls._hsv_hist(crop)

    def _combined_similarity(self, target_features, candidate_features):
        """얼굴 점수가 있으면 최우선, 없으면 외형(ReID)+옷색깔로 대체.

        얼굴이 둘 다 있는데 확실히 다른 사람이면(face_mismatch_veto 미만) 몸 점수와
        무관하게 -1.0을 반환해 그 후보를 탈락시킨다 - 이게 옷 색깔/체형이 비슷한
        서로 다른 사람을 실제로 구분해내는 부분이다.
        """
        target_face, target_reid, target_clothing = target_features
        candidate_face, candidate_reid, candidate_clothing = candidate_features

        clothing_score = cv2.compareHist(target_clothing, candidate_clothing, cv2.HISTCMP_CORREL)
        if target_reid is not None and candidate_reid is not None:
            reid_score = float(np.dot(target_reid, candidate_reid))
            body_score = self.appearance_weight * reid_score + self.clothing_weight * clothing_score
        else:
            body_score = clothing_score

        if target_face is not None and candidate_face is not None and self.face_recognizer is not None:
            face_score = self.face_recognizer.match(
                target_face, candidate_face, cv2.FaceRecognizerSF_FR_COSINE
            )
            if face_score < self.face_mismatch_veto:
                return -1.0
            return self.face_score_weight * face_score + (1 - self.face_score_weight) * body_score

        return body_score


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
