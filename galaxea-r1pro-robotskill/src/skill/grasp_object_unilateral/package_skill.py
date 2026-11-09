"""生成自包含的 Skill ZIP；不发布、不运行机器人。"""
import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

# scripts/solver.py 依赖共享的顶层 solver 包（CapturePointSolver / TargetPointSolver）。
# 漏掉它会让 Skill 在 Pilot 环境里直接 ModuleNotFoundError: solver。
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
    root = Path(__file__).resolve().parent
    shared_solver = root.parent.parent / "solver"
    files = [(root / name, name) for name in ("SKILL.md", "requirements.lock", "SOURCES.md")]
    files.extend((path, str(path.relative_to(root))) for path in sorted((root / "scripts").rglob("*.py")))
    files.extend((shared_solver / name, f"solver/{name}") for name in SOLVER_FILES)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "x", compression=ZIP_DEFLATED) as archive:
        for source, destination in files:
            archive.write(source, destination)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    print(build_package(parser.parse_args().output))
