"""生成自包含的 Skill ZIP；不发布、不运行机器人。"""
import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def build_package(output: Path) -> Path:
    root = Path(__file__).resolve().parent
    files = [root / name for name in ("SKILL.md", "requirements.lock", "SOURCES.md")]
    files.extend(sorted((root / "scripts").rglob("*.py")))
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "x", compression=ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(root))
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    print(build_package(parser.parse_args().output))
