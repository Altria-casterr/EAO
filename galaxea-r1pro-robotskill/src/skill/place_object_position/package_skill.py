"""生成含共享 solver 的独立 ZIP，不发布或启动机器人。"""
import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

SOLVER_FILES = (
    '__init__.py',
    'Nav/__init__.py',
    'Nav/navigation.py',
    'capture_point_solver/__init__.py',
    'capture_point_solver/capture_point_solver.py',
    'target_point_solver/__init__.py',
    'target_point_solver/target_point_solver.py',
)


def build_package(output: Path) -> Path:
    root = Path(__file__).resolve().parent
    shared = root.parent.parent / 'solver'
    files = [(root / name, name) for name in ('SKILL.md', 'SOURCES.md', 'requirements.lock')]
    files.extend((path, str(path.relative_to(root))) for path in sorted((root / 'scripts').glob('*.py')))
    files.extend((shared / name, f'solver/{name}') for name in SOLVER_FILES)
    for source, _ in files:
        if not source.is_file():
            raise FileNotFoundError(source)
    if not any(name == 'solver/target_point_solver/target_point_solver.py' for _, name in files):
        raise FileNotFoundError('共享 TargetPointSolver 缺失')
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, 'x', compression=ZIP_DEFLATED) as archive:
        for source, name in files:
            archive.write(source, name)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(build_package(parser.parse_args().output))
