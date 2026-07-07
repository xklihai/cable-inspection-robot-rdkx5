# -*- coding: utf-8 -*-
"""检查源码是否均为 UTF-8 编码。

本脚本遍历仓库根目录下的常见文本文件（.py/.md/.yaml/.xml/.cfg/.lua 等），
尝试以 UTF-8 编码读取每个文件；若出现 UnicodeDecodeError 解码错误，
则报告非 UTF-8 文件列表。

目的：
    确保仓库中所有源码、文档、配置文件均采用 UTF-8 编码，
    避免在跨平台协作或中文注释场景下出现乱码问题。

运行方式：
    python3 tests/check_utf8.py

预期输出：
    UTF-8 检查通过，共检查 N 个路径。
"""

# 从 pathlib 导入 Path 类，用于路径操作
from pathlib import Path


# 定义仓库根目录为当前文件（tests/check_utf8.py）的上一级目录
# __file__ 表示当前脚本路径；parents[1] 表示向上回溯一级到仓库根目录
ROOT = Path(__file__).resolve().parents[1]

# 定义需要检查编码的文件后缀集合
# 这些后缀涵盖了本仓库中的源码、文档与配置文件类型
SUFFIXES = {".py", ".md", ".yaml", ".yml", ".xml", ".cfg", ".lua"}


def main() -> None:
    """执行 UTF-8 编码检查的主函数。

    遍历仓库根目录下所有文件，对符合后缀要求的文件尝试 UTF-8 解码，
    并汇总报告解码失败的文件。
    """

    # 用于保存解码失败的文件路径列表
    bad = []

    # 递归遍历仓库根目录下的所有路径
    for path in ROOT.rglob("*"):
        # 仅处理普通文件，忽略目录
        # 仅检查后缀在 SUFFIXES 集合中的文件
        if path.is_file() and path.suffix.lower() in SUFFIXES:
            try:
                # 尝试以 UTF-8 编码读取文件内容
                path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                # 解码失败，将该文件路径加入 bad 列表
                bad.append(path)

    # 如果存在解码失败的文件，则输出错误信息并退出
    if bad:
        raise SystemExit("非 UTF-8 文件: " + ", ".join(str(p) for p in bad))

    # 所有文件均通过 UTF-8 检查，输出成功信息
    # 注意：len(list(ROOT.rglob('*'))) 会重新遍历一次，统计所有路径数量
    print(f"UTF-8 检查通过，共检查 {len(list(ROOT.rglob('*')))} 个路径。")


# 当该脚本直接运行时执行检查
if __name__ == "__main__":
    main()
