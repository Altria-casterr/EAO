"""通过 Pilot 调用 YOLO Ability；保存每项结果，不发送机器人运动指令。"""
import asyncio
import math
import time
from .coordinates import lookup_transform, apply_transform
from semantic_robot_skill_sdk import SkillCancelled, SkillContext, StopOutcome, StopRequest
from .controller import build_actions
from .models import YoloInput, YoloState, YoloResult


def extract_result(output, *, match=False, pose=False):
    """兼容 Pilot 输出的执行记录及 Ability 返回值包装。"""
    current = output
    for _ in range(6):
        if not isinstance(current, dict):
            break
        if current.get('status') in ('failed', 'stopped', 'interrupted') or current.get('error'):
            raise ValueError(str(current.get('error') or current.get('message') or current['status']))
        if match and 'best_match' in current and 'success' in current:
            return current
        if pose and 'best_angle' in current and 'best_iou' in current and 'success' in current:
            return current
        if 'objects' in current:
            if not isinstance(current['objects'], list) or any(not isinstance(x, dict) for x in current['objects']):
                raise ValueError('Ability objects 格式错误')
            return current
        if isinstance(current.get('result'), dict):
            current = current['result']
        elif isinstance(current.get('output'), dict):
            current = current['output']
        else:
            break
    raise ValueError('Ability 未返回包含 objects 的检测结果')


async def run(ctx: SkillContext) -> None:
    params = ctx.input(YoloInput)
    state = ctx.load_state(YoloState, default=YoloState())
    actions = build_actions(params)
    for index in range(state.next_index, len(actions)):
        ctx.check_cancelled()
        label, action = actions[index]
        started = time.perf_counter()
        output, parsed, error, conversion = {}, {}, None, None
        try:
            if label in ('match', 'pose'):
                if params.trans_flag:
                    transform = await asyncio.to_thread(lookup_transform,
                        params.target_input_frame, params.target_frame, host=params.rosbridge_host,
                        port=params.rosbridge_port, timeout=params.transform_timeout_s,
                        max_age_s=params.transform_max_age_s)
                    target = apply_transform(params.target_point, transform)
                else:
                    transform = {'position':[0.,0.,0.], 'orientation':[0.,0.,0.,1.]}
                    target = list(params.target_point)
                conversion = dict(source_frame=params.target_input_frame, target_frame=params.target_frame,
                    input_point_m=list(params.target_point), converted_point_m=target, transform=transform)
                action = action.model_copy(update={'parameters':{**action.parameters,'target_point':target}})
                ctx.check_cancelled()
            result = await ctx.execute(key=f'yolo-detect-native.{index}:{label}', action=action)
            output = result.output or {}
            state.evidence_refs.extend(result.evidence_refs)
            if result.status != 'succeeded':
                raise ValueError(result.error_message or f'Ability 状态: {result.status}')
            parsed = extract_result(output, match=label == 'match', pose=label == 'pose')
            if label == 'point_cloud':
                if not parsed.get('robot_frame') or not parsed.get('transform_camera_to_robot'):
                    raise ValueError('点云结果缺少输出坐标系或实际使用的外参')
            if label == 'match':
                if parsed.get('robot_frame') != params.target_frame:
                    raise ValueError('匹配结果坐标系与转换后的目标不一致')
                if not parsed['success'] or parsed['best_match'] is None:
                    raise ValueError('检测完成，但没有物体满足目标点匹配阈值')
            if label == 'pose':
                if parsed.get('robot_frame') != params.target_frame:
                    raise ValueError('姿态结果坐标系与目标不一致')
                if parsed.get('best_axis') not in ('roll', 'pitch', 'yaw') or not all(
                        math.isfinite(float(parsed[k])) for k in ('best_angle', 'best_iou')) or not 0 <= float(parsed['best_iou']) <= 1:
                    raise ValueError('姿态估计结果格式无效')
                if not parsed['success']:
                    raise ValueError(f"姿态计算完成，但匹配置信度未通过：IoU={parsed['best_iou']}")
            if label in ('detect', 'point_cloud') and params.require_objects and not parsed['objects']:
                raise ValueError('检测完成但未发现物体；require_objects=true')
        except SkillCancelled:
            raise
        except Exception as exc:
            error = str(exc)
        state.readings.append(dict(task=label, status='failed' if error else 'succeeded',
            elapsed_seconds=time.perf_counter()-started,
            error=error, object_count=len(parsed['objects']) if 'objects' in parsed else None,
            result=parsed, raw_output=output, target_conversion=conversion))
        state.next_index = index + 1
        ctx.checkpoint(state)
    succeeded = sum(item['status'] == 'succeeded' for item in state.readings)
    status = 'succeeded' if succeeded == len(state.readings) else 'partial' if succeeded else 'failed'
    ctx.complete(YoloResult(overall_status=status, readings=state.readings, evidence_refs=state.evidence_refs))


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    return ctx.stop_outcome(safe=True, summary='停止 YOLO 测试流程；未发送运动指令',
        physical_state='unknown', requires_intervention=False, evidence_refs=ctx.recent_evidence())
