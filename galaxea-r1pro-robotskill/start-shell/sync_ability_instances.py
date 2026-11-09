#!/usr/bin/env python3
"""按当前 AbilityFramework 实际运行的实例刷新部署文件里的 instance_id。

背景：Ability 实例 UUID 在 AF 每次重启（或实例重新 activate）后都会变化，
而 `semantic-pilot` 的绑定规则是「写了 instance_id 就必须精确命中，否则该角色
绑不上」，`start-shell/run_*.sh` 的前置检查也会因此直接失败。

用法：
    python3 sync_ability_instances.py --check            # 只显示将要做的修改
    python3 sync_ability_instances.py                    # 就地更新 src/config/*.yaml
    python3 sync_ability_instances.py a.yaml b.yaml      # 只处理指定文件

默认从 http://127.0.0.1:8080 读取 /api/ability-heartbeat，可用 --af 覆盖。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.request
from pathlib import Path

# abilityRole -> abilityName，来自 framework/packages/*/*/ability.manifest.yaml。
# navigation 与 navigation_move 都会声明 navigation.stop，必须按角色区分，不能按 action 猜。
ROLE_ABILITY = {
    "robot": "RobotObservation.Demo",
    "end_effector": "Gripper.Demo",
    "torso": "TorsoControl.Demo",
    "joint_motion": "JointControl.Demo",
    "manipulator_motion": "JointEEFPoseControl.Demo",
    "navigation": "WooshNavigation.Demo",
    "navigation_move": "WooshMove.Demo",
    "perception": "YoloDetect.Demo",
}

CONFIG_DIR = Path(__file__).resolve().parent.parent / "src" / "config"
UUID_RE = re.compile(r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}")
ROLE_LINE_RE = re.compile(r"^(?P<indent>\s+)(?P<role>[a-z_]+):(?P<rest>.*)$")


def running_instances(endpoint: str) -> dict[str, str]:
    url = endpoint.rstrip("/") + "/api/ability-heartbeat"
    with urllib.request.urlopen(url, timeout=5) as response:
        items = json.load(response)
    result: dict[str, str] = {}
    for item in items:
        if str(item.get("state", "")).lower() not in {"running", "standby"}:
            continue
        ability = str(item.get("abilityName") or "")
        instance = str(item.get("id") or "")
        if not ability or not instance:
            continue
        if ability in result and result[ability] != instance:
            print(f"警告：{ability} 有多个健康实例，保留第一个 {result[ability]}", file=sys.stderr)
            continue
        result[ability] = instance
    return result


def abilities_span(lines: list[str]) -> tuple[int, int]:
    """返回 abilities: 块的 [start, end) 行号区间。"""
    start = next((i for i, line in enumerate(lines) if line.rstrip() == "abilities:"), None)
    if start is None:
        return (-1, -1)
    for index in range(start + 1, len(lines)):
        line = lines[index]
        if line.strip() and not line.startswith((" ", "\t", "#")):
            return (start + 1, index)
    return (start + 1, len(lines))


def patch_file(path: Path, instances: dict[str, str], check: bool) -> list[str]:
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    start, end = abilities_span(lines)
    if start < 0:
        return [f"{path.name}: 没有 abilities: 段，跳过"]
    changes: list[str] = []
    index = start
    while index < end:
        match = ROLE_LINE_RE.match(lines[index].rstrip("\n"))
        if not match or match.group("role") not in ROLE_ABILITY:
            index += 1
            continue
        role = match.group("role")
        ability = ROLE_ABILITY[role]
        wanted = instances.get(ability)
        if not wanted:
            changes.append(f"{path.name}: {role} 期望 {ability}，但 AF 中没有健康实例 → 未修改")
            index += 1
            continue
        # 该角色条目可能占一行（inline）或多行，取到下一个同级/更高级键为止。
        indent = len(match.group("indent"))
        block_end = index + 1
        while block_end < end:
            nxt = lines[block_end]
            if nxt.strip() and not nxt.startswith(" " * (indent + 1)):
                break
            block_end += 1
        segment = "".join(lines[index:block_end])
        found = UUID_RE.search(segment)
        if found is None:
            changes.append(f"{path.name}: {role} 条目没有 instance_id → 跳过")
            index = block_end
            continue
        if found.group(0) == wanted:
            index = block_end
            continue
        new_segment = UUID_RE.sub(wanted, segment)
        lines[index:block_end] = new_segment.splitlines(keepends=True)
        changes.append(f"{path.name}: {role} ({ability}) {found.group(0)} → {wanted}")
        index = block_end
    if changes and not check:
        path.write_text("".join(lines), encoding="utf-8")
    return changes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", type=Path, help="部署 yaml；默认处理 src/config/*.yaml")
    parser.add_argument("--af", default="http://127.0.0.1:8080", help="AbilityFramework endpoint")
    parser.add_argument("--check", action="store_true", help="只打印改动，不写文件")
    args = parser.parse_args()

    instances = running_instances(args.af)
    if not instances:
        print(f"错误：{args.af}/api/ability-heartbeat 没有 Running 实例", file=sys.stderr)
        return 1
    print("当前 AF 运行实例：")
    for ability, instance in sorted(instances.items()):
        print(f"  {ability:<28} {instance}")
    print()

    targets = args.files or sorted(CONFIG_DIR.glob("*.yaml")) + sorted(CONFIG_DIR.glob("*.yml"))
    changed = 0
    for path in targets:
        for line in patch_file(path, instances, args.check):
            print(("  [check] " if args.check else "  ") + line)
            changed += 1
    if not changed:
        print("所有 instance_id 都已与当前 AF 一致，无需修改。")
    elif args.check:
        print("\n以上为预演结果；去掉 --check 即会写入。")
    else:
        print("\n已更新。Ability 实例变化后请重新执行本脚本。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
