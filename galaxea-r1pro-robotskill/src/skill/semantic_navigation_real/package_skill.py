"""构建可独立导入的 Robot Skill ZIP，随包带入已有共享 solver。"""

from __future__ import annotations

import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

SOLVER_FILES = (
    "__init__.py",
    "Nav/__init__.py",
    "Nav/navigation.py",
    "capture_point_solver/__init__.py",
    "capture_point_solver/capture_point_solver.py",
    "target_point_solver/__init__.py",
    "target_point_solver/target_point_solver.py",
)


def build_package(output: Path) -> Path:
    skill_dir = Path(__file__).resolve().parent
    shared_solver = skill_dir.parent.parent / "solver"
    files = [(skill_dir / name, name) for name in ("SKILL.md", "requirements.lock")]
    files.extend((path, f"scripts/{path.name}") for path in sorted((skill_dir / "scripts").glob("*.py")))
    files.extend((shared_solver / name, f"solver/{name}") for name in SOLVER_FILES)
    for source, _ in files:
        if not source.is_file():
            raise FileNotFoundError(f"缺少打包源文件：{source}")
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    # 显式拒绝覆盖已有包；已发布版本的内容不能被无意替换。
    with ZipFile(output, "x", compression=ZIP_DEFLATED) as archive:
        for source, destination in files:
            archive.write(source, destination)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(build_package(args.output))


if __name__ == "__main__":
    main()
