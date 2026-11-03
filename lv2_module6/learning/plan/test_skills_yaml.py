"""skills.yaml 형식 확인 — task가 쓰는 skill이 목록에 있고, rl skill마다 보상, yolo·rule마다 평가 지표가 있는지"""
from pathlib import Path

import yaml

Y = yaml.safe_load((Path(__file__).resolve().parent.parent / "skills.yaml").read_text())


def test_catalog():
    skills = Y["skills"]
    for task, names in Y["tasks"].items():
        for n in names:
            assert n in skills, f"{task}: {n} 없음"
    for n, s in skills.items():
        assert s["method"] in ("rl", "yolo", "rule", "measured", "none", "keep"), n
        if s["method"] == "rl":
            assert s.get("reward") and s.get("success"), n
        if s["method"] == "measured":
            assert s.get("loss"), n
        if s["method"] in ("yolo", "rule"):
            assert s.get("check"), n


if __name__ == "__main__":
    test_catalog()
    print("ok", len(Y["skills"]), "skills")
