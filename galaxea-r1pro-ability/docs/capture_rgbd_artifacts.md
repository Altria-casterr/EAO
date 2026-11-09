# robot.capture_rgbd 图片证据

RGB（默认 axis=rgb）和 both 模式将 SDK 返回的 BGR uint8 图像编码为 JPEG，原子保存到交换目录。depth 模式保持原有元信息输出，不生成深度图片。

Ability 进程必须设置 SEMANTIC_ABILITY_ARTIFACT_ROOT，与 Pilot 使用同一个可访问目录（容器部署时需要共享挂载）。目录未配置、图像无效、编码或写入失败会明确报错，不返回虚假的图片成功记录。该路径由部署设置，不由模型或 Skill 输入。

返回的执行记录 result.artifact_candidates 包含 candidate_id、exchange_path（相对路径）、media_type、宽高、采集时间及 publication_state=ready_for_pilot。observations 中 RGB 条目为 kind=sensor.frame，value 包含对应候选。Ability 不生成 artifact:// 引用，不向 JSON 写入像素或 Base64。Pilot 导入、上传后填充真实引用；ready_for_pilot 仅表示文件已就绪，不表示上传成功。

现有抓取 Skill 的 verify:image 调用无需新增参数。图片对应此次 verify 采集，不是先前 YOLO 推理使用的帧。Web 展示还取决于部署的 Pilot 支持 Artifact 候选导入和上传成功。

部署需要重新打包更新 RobotObservation Ability；仅修改源码不会改变运行中版本。交换目录中的文件应按执行记录保留策略清理，不能在 Pilot 导入前删除。
