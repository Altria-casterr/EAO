"""按检查点串行执行两个阶段（躯干、手臂）；失败不继续，恢复使用相同 Action key。"""
from semantic_robot_skill_sdk import SkillContext, StopRequest, StopOutcome
from .controller import build_action, LABELS
from .models import PrepareInput, PrepareState, PrepareResult

NEXT = {'torso': 'arms', 'arms': 'done'}

def report(ctx, stage, status, summary):
    ctx.report(f'stage.{status}', stage=stage, stage_status=status, summary=summary)

async def run(ctx: SkillContext) -> None:

    ctx.check_cancelled()
    params = ctx.input(PrepareInput)  # 全部参数在第一个物理动作前完成校验。
    state = ctx.load_state(PrepareState, default=PrepareState())

    if state.failure_code:
        ctx.fail(state.failure_code, state.failure_message or '预备阶段失败', state.evidence_refs)
        return

    while state.stage != 'done':
        ctx.check_cancelled()
        stage = state.stage
        report(ctx, stage, 'running', LABELS[stage])

        if stage not in state.actions:
            state.actions[stage] = build_action(stage, params)
            
        state.physical_action_started = True
        ctx.checkpoint(state)
        if stage not in state.results:
            result = await ctx.execute(f'robot-prepare:{stage}:1', state.actions[stage])
            state.results[stage] = result
            state.evidence_refs = list(dict.fromkeys([*state.evidence_refs, *result.evidence_refs]))
            ctx.checkpoint(state)
        ctx.check_cancelled()
        result = state.results[stage]
        if result.status != 'succeeded':
            state.failure_code = 'ACTION_STATE_UNKNOWN' if result.status == 'interrupted' else 'PREPARE_ACTION_FAILED'
            state.failure_message = f'{LABELS[stage]}：{result.error_message or result.status}'
            ctx.checkpoint(state)
            report(ctx, stage, 'failed', state.failure_message)
            ctx.fail(state.failure_code, state.failure_message, state.evidence_refs)
            return
        report(ctx, stage, 'completed', f'{LABELS[stage]}完成')
        state.stage = NEXT[stage]
        ctx.checkpoint(state)
    # 从实际已保存的动作取目标，恢复时不会用新输入覆盖执行结果。
    arms = state.actions['arms'].parameters
    ctx.complete(PrepareResult(left_positions=arms['left_positions'], right_positions=arms['right_positions'],
                               evidence_refs=state.evidence_refs))

async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:

    state = ctx.load_state(PrepareState, default=PrepareState())

    uncertain = state.physical_action_started

    return ctx.stop_outcome(safe=not uncertain,
        physical_state='unknown' if uncertain else 'no_motion_started',
        requires_intervention=uncertain,
        summary='已请求停止，躯干和双臂实际停止状态待确认' if uncertain else '尚未发送物理动作',
        evidence_refs=state.evidence_refs)
