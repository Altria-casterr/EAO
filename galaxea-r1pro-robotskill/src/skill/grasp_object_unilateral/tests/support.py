from copy import deepcopy
from semantic_robot_skill_sdk import ActionResult, MockSkillContext
from grasp_object_unilateral.scripts.models import GraspInput


def inputs(**updates):
    data = {
        "target": {"object_ref": "box-test", "position_m": [0.45, 0, 0.9],
                   "size": {"long_m": 0.6, "width_m": 0.4, "height_m": 0.34}},
        "grasp_axis": "left",
        "approach_force": {"force_threshold_n": 3, "force_coordinate": ["y"]},
        "verify_force": {"force_threshold_n": 2, "force_coordinate": ["y"]},
    }
    data.update(updates)
    return GraspInput.model_validate(data)


def pose(y, z=0.1):
    return {"header": {"frame_id": "base_link"}, "pose": {
        "position": {"x": 0.45, "y": y, "z": z},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    }}


class Responder:
    def __init__(self):
        self.poses = {"left": pose(0.42), "right": pose(-0.42)}
        self.actions = []
        self.contact_poses = None
        self.mutate = lambda key, action, result: result

    def respond(self, key, action):
        self.actions.append((key, action))
        values = action.parameters
        output = {}
        effect = "none"
        if action.type in {"robot.get_state", "robot.get_sensors_state"}:
            name = values["observation"]
            if name == "force_data":
                force = 1.0 if key.endswith("force-before") else 2.0
                data = {side + "_arm": {"force_x": force, "force_y": 0.0, "force_z": 0.0}
                        for side in ("left", "right")}
            elif name == "transformer_martix":
                data = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
            elif name == "floating_base":
                data = {"position": [0, 0, 0.8], "orientation": [0, 0, 0, 1]}
            else:
                data = deepcopy(self.poses[name.removeprefix("ee_pose_")])
            output = {"result": {"status": "completed", "observations": [{"name": name, "data": data}]}}
        elif action.type == "perception.match_target":
            output = {"result": {"status": "completed", "result": {
                "success": True, "robot_frame": values["robot_frame"],
                "best_match": {"center_robot": [0.45, 0, 0.1], "confidence": 0.9},
            }}}
        elif action.type.startswith("motion.move_end_effector_single"):
            side = values["axis"]
            target = values["_target_pose"]
            self.poses[side] = {"header": {"frame_id": values["frame_id"]}, "pose": {
                "position": {axis: target[f"target_{axis}"] for axis in "xyz"},
                "orientation": {axis: target[axis] for axis in "xyzw"},
            }}
            effect = "confirmed"
        elif action.type.startswith("motion.move_end_effector_both"):
            for side in ("left", "right"):
                target = values[f"{side}_target_pose"]
                self.poses[side] = {"header": {"frame_id": values[f"{side}_frame_id"]}, "pose": {
                    "position": {axis: target[f"target_{axis}"] for axis in "xyz"},
                    "orientation": {axis: target[axis] for axis in "xyzw"},
                }}
            effect = "confirmed"
        elif action.type == "manipulation.force_align":
            side = values["axis"]
            self.poses[side]["pose"]["position"]["y"] += -0.02 if side == "left" else 0.02
            if key.endswith("approach:other-force"):
                self.contact_poses = deepcopy(self.poses)
            effect = "confirmed"
        elif action.type == "robot.capture_rgbd":
            output = {"observations": [{"source": "sensor", "name": "rgb", "sensor_id": "head_camera",
                                        "encoding": "rgb8", "frame_id": "camera"}]}
        elif action.type == "torso.move_to_posture_all":
            effect = "confirmed"
        else:
            raise AssertionError(action.type)
        return self.mutate(key, action, ActionResult(status="succeeded", output=output, physical_effect=effect))


class Context(MockSkillContext):
    def __init__(self, params=None):
        super().__init__(params or inputs())
        self.responder = Responder()

    async def execute(self, key, action):
        if key not in self._records:
            self.queue_action(action.type, self.responder.respond(key, action))
        return await super().execute(key, action)
