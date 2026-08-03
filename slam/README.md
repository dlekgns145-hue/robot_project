# slam/

STEP 2(SLAM)를 완료하면 여기에 다음 파일들이 생성/저장됩니다.

- `map.yaml`
- `map.pgm`

저장 명령 예시 (SLAM Toolbox + RViz로 지도를 다 그린 뒤):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_project/slam/map
```

이후 STEP 3(Navigation)에서 `nav2_bringup`의 map_server가 이 `map.yaml`을 불러와서 사용합니다.
