from galaxea_r1pro_0.service.entrypoint import run_ability
from galaxea_r1pro_0.service.roles import AbilityRole


if __name__ == "__main__":
    run_ability(AbilityRole.yolo_detect, "YoloDetect.Demo")
