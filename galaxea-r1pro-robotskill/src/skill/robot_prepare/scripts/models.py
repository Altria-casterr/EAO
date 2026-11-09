"""关节角单位为弧度；固定归零躯干，双臂默认现场预备角。"""
from typing import Annotated, Literal
from pydantic import BaseModel, ConfigDict, Field, field_validator
from semantic_robot_skill_sdk import Action, ActionResult

Finite = Annotated[float, Field(allow_inf_nan=False)]
Positive = Annotated[float, Field(gt=0, allow_inf_nan=False)]
Seven = tuple[Finite, Finite, Finite, Finite, Finite, Finite, Finite]
SevenNames = tuple[str, str, str, str, str, str, str]
LEFT = (0.857, 0.159, 0.010, -2.074, 0.127, -0.360, -0.453)
RIGHT = (0.855, -0.129, -0.035, -2.073, -0.127, -0.354, 0.492)
Stage = Literal['torso', 'arms', 'done']

class StrictModel(BaseModel):
    model_config = ConfigDict(extra='forbid')

class TorsoSettings(StrictModel):
    positions: tuple[Literal[0], Literal[0], Literal[0], Literal[0]] = (0, 0, 0, 0)
    timeout: Positive = 30.0
    tol: Positive = 0.02
    loop_period: Positive = 0.1
    speeds: tuple[Finite, Finite, Finite, Finite] = (0.5, 0.8, 0.7, 0.7)

class ArmSettings(StrictModel):
    left_positions: Seven = LEFT
    right_positions: Seven = RIGHT
    left_joint_names: SevenNames = tuple(f'left_arm_joint{i}' for i in range(1, 8))
    right_joint_names: SevenNames = tuple(f'right_arm_joint{i}' for i in range(1, 8))
    timeout: Positive = 30.0
    tol: Positive = 0.01
    loop_freq: Positive = 0.2

    @field_validator('left_joint_names', 'right_joint_names')
    @classmethod
    def names(cls, values):
        if (len(set(values)) != 7 or any(not n.strip() for n in values)
                or any(n in {'left_arm', 'right_arm'} for n in values)):
            raise ValueError('需要七个非空且不重复的实际关节名称，与七个角度按顺序对应')
        return values

class PrepareInput(StrictModel):
    torso: TorsoSettings = Field(default_factory=TorsoSettings)
    arms: ArmSettings = Field(default_factory=ArmSettings)

class PrepareState(StrictModel):
    stage: Stage = 'torso'
    actions: dict[str, Action] = Field(default_factory=dict)
    results: dict[str, ActionResult] = Field(default_factory=dict)
    physical_action_started: bool = False
    evidence_refs: list[str] = Field(default_factory=list)
    failure_code: str | None = None
    failure_message: str | None = None

class PrepareResult(StrictModel):
    ready: bool = True
    torso_positions: tuple[int, int, int, int] = (0, 0, 0, 0)
    left_positions: Seven
    right_positions: Seven
    completed_stages: list[str] = Field(default_factory=lambda: ['torso', 'arms'])
    evidence_refs: list[str] = Field(default_factory=list)
