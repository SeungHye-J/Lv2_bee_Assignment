uint32 step
string skill        # navigate | pick | place | inspect | say | pose | load | unload  (정의: documents/project/ros/05-arm.md)
string target       # 물체 id (/tracked_objects의 id) · "x,y"(navigate) · 자세 이름(pose) · 칸 0~11 · next(load) · last(unload) · "tray"(pick·place — unload last·load next와 같음) · 말할 내용(say)
string relation     # front | on | ...
float32 offset_m
