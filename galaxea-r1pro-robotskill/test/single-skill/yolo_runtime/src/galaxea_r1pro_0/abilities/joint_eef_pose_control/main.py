from galaxea_r1pro_0.service.entrypoint import run_ability
from galaxea_r1pro_0.service.roles import AbilityRole


if __name__ == "__main__":
    run_ability(AbilityRole.joint_eef_pose_control, "JointEEFPoseControl.Demo")
