# perception/

여기에 팀원이 만든 YOLO 모델 파일을 넣으세요:

- `best.pt`

`detect.py`의 `model_path` 파라미터 기본값이 `perception/best.pt`로 되어 있어서,
이 폴더 안에 `best.pt`만 넣으면 별도 경로 수정 없이 바로 동작합니다.

다른 위치에 두고 싶으면, 노드 실행 시 파라미터로 경로를 직접 지정하면 됩니다:

```bash
ros2 run robot_project detect --ros-args -p model_path:=/절대/경로/best.pt
```
