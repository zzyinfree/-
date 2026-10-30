import unittest

from mission_online import MissionRejected, parse_model_reply, validate_intent


class MissionOnlineTest(unittest.TestCase):
    def test_official_example(self):
        task = "飞机起飞，然后穿过框，再去识别二维码，然后再穿回框，接着去打击红色气球，完成后就回到起点并降落；"
        commands = ["takeoff", "cross_frame", "detect_aruco", "return_cross_frame", "hit_balloon:red", "land"]
        self.assertIsNone(validate_intent(task, commands))

    def test_no_balloon(self):
        task = "起飞后穿框识别二维码，再穿回来并降落，不打气球。"
        commands = ["takeoff", "cross_frame", "detect_aruco", "return_cross_frame", "land"]
        self.assertIsNone(validate_intent(task, commands))

    def test_other_balloon_colors(self):
        for color, zh in [("blue", "蓝"), ("purple", "紫")]:
            with self.subTest(color=color):
                task = f"起飞，穿框，识别二维码，再穿回框，刺破{zh}球，回到起点并降落"
                commands = ["takeoff", "cross_frame", "detect_aruco", "return_cross_frame", f"hit_balloon:{color}", "land"]
                self.assertIsNone(validate_intent(task, commands))

    def test_wrong_color_rejected(self):
        with self.assertRaises(MissionRejected):
            validate_intent("起飞并打红色气球后降落", ["takeoff", "hit_balloon:blue", "land"])

    def test_hallucinated_action_rejected(self):
        with self.assertRaises(MissionRejected):
            validate_intent("起飞", ["takeoff", "cross_frame"])

    def test_unknown_command_rejected(self):
        with self.assertRaises(MissionRejected):
            parse_model_reply("```command\ntakeoff\nteleport\n```")

    def test_multiple_blocks_rejected(self):
        with self.assertRaises(MissionRejected):
            parse_model_reply("```command\ntakeoff\n```\n```command\nland\n```")

    def test_parse_official_format(self):
        self.assertEqual(parse_model_reply("{1. 解释\n2. 控制命令\n```command\ntakeoff\nland\n```}"), ["takeoff", "land"])


if __name__ == "__main__":
    unittest.main()
