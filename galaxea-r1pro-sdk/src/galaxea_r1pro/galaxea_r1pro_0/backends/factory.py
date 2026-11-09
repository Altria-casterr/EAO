"""按 Robot profile 选择后端，Ability 不包含选择分支。"""

from ..utils.errors import BackendUnavailableError
from ..utils.profile import RobotProfile
from ..utils.types import BackendKind
from .base import RobotBackend
from .real import RealBackend

def create_backend(profile: RobotProfile) -> RobotBackend:

    if profile.backend is BackendKind.REAL:
        return RealBackend(profile)

    raise BackendUnavailableError(
        backend=profile.backend,
        robot_id=profile.robot_id,
        reason=BackendUnavailableError.NOT_IMPLEMENTED,
        operation="create_backend",
        hint="当前仅支持 real 后端；mujoco/isaac 后端尚未接入",
    )
