uint32 id
string class_name
float32 confidence
geometry_msgs/PoseWithCovariance pose   # map 기준
geometry_msgs/Vector3 size              # m
builtin_interfaces/Time last_seen
string camera                           # 본 카메라 (pole_cam | gripper_cam) — VLM 이미지 마킹용
int32[2] px                             # 그 카메라 영상 속 박스 중심 픽셀 [u, v]
