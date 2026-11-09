"""逐项读取并汇总：一项失败不丢失其他观测。"""

from semantic_robot_skill_sdk import SkillCancelled, SkillContext, StopOutcome, StopRequest

from .controller import build_actions
from .models import ObservationInput, ObservationResult, ObservationState


def _items(output, observations):
    direct = output.get('observations')
    nested = output.get('result')
    if isinstance(direct, list):
        return direct
    if isinstance(nested, dict) and isinstance(nested.get('observations'), list):
        return nested['observations']
    return [item.model_dump(mode='json') for item in observations]


def _error_from_items(items):
    if not items:
        return 'Ability 未返回观测数据'
    for item in items:
        if not isinstance(item, dict):
            return 'Ability 返回的观测格式无效'
        value = item.get('value') or {}
        data = item.get('data', value.get('data') if isinstance(value, dict) else None)
        if data is None:
            return f"{item.get('name', '观测项')} 数据为空"
        if isinstance(data, dict) and data.get('error'):
            return f"{item.get('name', '观测项')}: {data['error']}"
    return None


async def run(ctx: SkillContext) -> None:
    params = ctx.input(ObservationInput)
    state = ctx.load_state(ObservationState, default=ObservationState())
    actions = build_actions(params)
    for index in range(state.next_index, len(actions)):
        ctx.check_cancelled()
        label, action = actions[index]
        output = {}
        items = []
        try:
            result = await ctx.execute(key=f'robot-observation-native.{index}:{label}', action=action)
            output = result.output or {}
            items = _items(output, result.observations)
            error = result.error_message if result.status != 'succeeded' else _error_from_items(items)
            if result.status != 'succeeded' and not error:
                error = f'Ability 状态: {result.status}'
            state.evidence_refs.extend(result.evidence_refs)
        except SkillCancelled:
            raise
        except Exception as exc:
            error = str(exc)
        state.readings.append({
            'item': label,
            'status': 'failed' if error else 'succeeded',
            'error': error,
            'output': output,
            'observations': items,
        })
        state.next_index = index + 1
        ctx.checkpoint(state)

    succeeded = sum(item['status'] == 'succeeded' for item in state.readings)
    overall = 'succeeded' if succeeded == len(state.readings) else 'partial' if succeeded else 'failed'
    ctx.complete(ObservationResult(overall_status=overall, readings=state.readings, evidence_refs=state.evidence_refs))


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    return ctx.stop_outcome(
        safe=True,
        summary='只读观测已停止；本 Skill 未发出运动指令',
        physical_state='unknown',
        requires_intervention=False,
        evidence_refs=ctx.recent_evidence(),
    )
