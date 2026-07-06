# 开源发布检查清单

## 代码

- [ ] 仓库公开可访问。
- [ ] 根目录包含 `README_cn.md`、`README.MD`、`LICENSE`、`.gitignore`、`.gitattributes`。
- [ ] 源码中不包含个人身份信息、单位信息、联系方式、平台密钥或本地绝对路径。
- [ ] 不提交 `build/`、`install/`、`log/`、`__pycache__/`、`.pyc`、ROS bag 和临时视频。
- [ ] `tests/check_utf8.py` 通过。
- [ ] `tests/test_pid.py` 通过。

## 文档

- [ ] 中文 `README_cn.md` 和英文 `README.MD` 均包含项目简介、硬件平台、系统架构、节点话题、快速运行和许可证。
- [ ] `docs/DEPLOYMENT.md` 包含 RDK X5 部署、C30D 串口、TROS 推理替换点和建图导航说明。
- [ ] `docs/NODEHUB_SUBMISSION.md` 已替换实际仓库、视频、报告链接。
- [ ] 项目截图能在社区页面正常显示。

## NodeHub

- [ ] 项目标题清晰，包含 RDK X5 和线缆巡检关键词。
- [ ] 项目简介说明“实际完成了什么”，避免只写概念。
- [ ] 标签包含 RDK X5、ROS2、TROS、YOLOv8、LiDAR、Cartographer、Nav2。
- [ ] 填入开源仓库地址。
- [ ] 附上演示视频和作品报告。
- [ ] 发布后把 NodeHub 项目链接回填到比赛报名/提交页面。
