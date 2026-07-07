# -*- coding: utf-8 -*-
"""检查源码是否均为 UTF-8 编码。

遍历仓库根目录下的常见文本文件（.py/.md/.yaml/.xml 等），
尝试以 UTF-8 编码读取；若出现解码错误，则报告非 UTF-8 文件列表。

可在无 ROS 环境下直接运行：
    python3 tests/check_utf8.py
"""

from pathlib import Path


# 仓库根目录（tests 目录的上一级）
ROOT = Path(__file__).resolve().parents[1]

# 需要检查编码的文件后缀集合
SUFFIXES = {".py", ".md", ".yaml", ".yml", ".xml", ".cfg", ".lua"}


def main() -> None:
    """执行 UTF-8 编码检查。"""
    bad = []
    for path in ROOT.rglob("*"):
        # 仅检查指定后缀的文本文件
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
