# -*- coding: utf-8 -*-
"""检查源码是否均为 UTF-8 编码。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SUFFIXES = {".py", ".md", ".yaml", ".yml", ".xml", ".cfg", ".lua"}


def main() -> None:
    bad = []
    for path in ROOT.rglob("*"):
        if path.is_file() and path.suffix.lower() in SUFFIXES:
            try:
                path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                bad.append(path)
    if bad:
        raise SystemExit("非 UTF-8 文件: " + ", ".join(str(p) for p in bad))
    print(f"UTF-8 检查通过，共检查 {len(list(ROOT.rglob('*')))} 个路径。")


if __name__ == "__main__":
    main()
