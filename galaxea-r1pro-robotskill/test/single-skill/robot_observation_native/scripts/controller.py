from semantic_robot_skill_sdk import Action
from .models import ObservationInput


def build_actions(params: ObservationInput) -> list[tuple[str, Action]]:
    specs = [('state.' + name, 'robot.get_state', {'observation': name}) for name in params.state_observations]
    specs += [('sensor.' + name, 'robot.get_sensors_state', {'observation': name}) for name in params.sensor_observations]
    if params.capture_rgbd:
        specs.append(('rgbd.' + params.sensor_id, 'robot.capture_rgbd', {'sensor_id': params.sensor_id, 'axis': 'both', 'save_images': True}))
    return [(label, Action(type=kind, parameters=data, timeout_seconds=params.timeout_seconds, label=label)) for label, kind, data in specs]
