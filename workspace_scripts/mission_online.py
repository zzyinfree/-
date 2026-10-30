"""FM9G4B-V natural-language mission entry for the online simulation.

Start this process *before* the timer. It loads the model, waits for one
operator-supplied prompt, validates model output, then publishes once. No
vision output is injected here; the existing perception nodes remain active.
"""
import argparse
import datetime as dt
import json
import os
import re
import sys
import threading
import time
import uuid

MODEL_DIR = "/home/ubuntu/model/FM9G4B-V"
PROMPT_FILE = "/home/ubuntu/model/prompts/mission_prompt.txt"
AUDIT_FILE = "/root/gpufree-data/cicc_workspace/outputs/mission_online_audit.jsonl"
VALID_SIMPLE = {"takeoff", "cross_frame", "detect_aruco", "return_cross_frame", "land"}
VALID_COLORS = {"red", "blue", "purple"}
COLOR_ZH = {"红": "red", "蓝": "blue", "紫": "purple"}


class MissionRejected(ValueError):
    pass


def parse_model_reply(reply):
    fences = re.findall(r"```\s*command\s*\n(.*?)```", reply, flags=re.I | re.S)
    if len(fences) != 1 or len(re.findall(r"```", reply)) != 2:
        raise MissionRejected("九格没有返回唯一的 command 代码块")
    commands = [s.strip() for s in fences[0].splitlines() if s.strip()]
    if not commands or len(commands) > 12:
        raise MissionRejected("命令数量为空或过多")
    for c in commands:
        if c in VALID_SIMPLE:
            continue
        if re.fullmatch(r"hit_balloon:(?:(?:A|B|C):)?(?:red|blue|purple)", c):
            continue
        raise MissionRejected(f"不支持的命令: {c}")
    return commands


def validate_intent(task, commands):
    """Reject clear omissions/additions; never synthesize a missing command."""
    lowered = task.lower()
    markers = {
        "takeoff": bool(re.search(r"起飞|升空", task)),
        "detect_aruco": bool(re.search(r"二维码|aruco", lowered)),
        "land": bool(re.search(r"降落|落地|着陆", task)),
        "return_cross_frame": bool(re.search(r"穿回|返回.*[框门]|反向穿", task)),
        "cross_frame": bool(re.search(r"穿(?:过|越)?(?:方)?框|穿门|穿越门", task)),
    }
    # "穿回框" also implies an outbound frame crossing only if stated.
    if markers["return_cross_frame"] and "cross_frame" in commands:
        markers["cross_frame"] = True
    for command, requested in markers.items():
        present = command in commands
        if requested and not present:
            raise MissionRejected(f"提示词要求 {command}，模型遗漏")
        if present and not requested:
            raise MissionRejected(f"提示词未明确要求 {command}，模型却加入")
    if len(commands) != len(set(commands)):
        raise MissionRejected("重复命令需要人工复核，拒绝自动执行")
    if "takeoff" in commands and commands[0] != "takeoff":
        raise MissionRejected("起飞不是第一步")
    if "land" in commands and commands[-1] != "land":
        raise MissionRejected("降落不是最后一步")
    if "cross_frame" in commands and "return_cross_frame" in commands:
        if commands.index("cross_frame") > commands.index("return_cross_frame"):
            raise MissionRejected("穿框顺序错误")
    target_colors = {eng for zh, eng in COLOR_ZH.items() if zh in task}
    target_colors |= {eng for eng in VALID_COLORS if eng in lowered}
    hit_commands = [c for c in commands if c.startswith("hit_balloon:")]
    requested_hit = bool(re.search(r"打击|刺破|击破|打(?:红|蓝|紫)|撞(?:红|蓝|紫)", task))
    if re.search(r"不(?:要)?打(?:击)?(?:气球)?|无须打击", task):
        requested_hit = False
    if requested_hit and not hit_commands:
        raise MissionRejected("提示词要求打击，模型遗漏")
    if hit_commands and not requested_hit:
        raise MissionRejected("提示词未明确要求打击，模型却加入")
    emitted_colors = {c.split(":")[-1] for c in hit_commands}
    if hit_commands and (not target_colors or emitted_colors != target_colors):
        raise MissionRejected("气球颜色和提示词不一致或颜色不明确")


def audit(record, path=AUDIT_FILE):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
        f.flush()
        os.fsync(f.fileno())


def load_model():
    import torch
    from transformers import AutoModel, AutoTokenizer
    print("[预热] 加载九格 FM9G4B-V；请在提示出现后粘贴官方指令，开赛时按回车。", flush=True)
    t0 = time.monotonic()
    model = AutoModel.from_pretrained(
        MODEL_DIR, trust_remote_code=True,
        attn_implementation="sdpa", torch_dtype=torch.bfloat16,
    ).eval().cuda()
    tokenizer = AutoTokenizer.from_pretrained(MODEL_DIR, trust_remote_code=True)
    knowledge = open(PROMPT_FILE, encoding="utf-8").read()
    knowledge += ("\n\n补充约束：仅生成用户明确要求的命令。用户只给出气球颜色时，"
                  "解释思路不得臆断气球属于A/B/C哪个区域；只写寻找该颜色气球，"
                  "实际区域由运行时行为树参数解析。不要把行为树完成写成裁判已计分。")
    print(f"[就绪] 九格加载 {time.monotonic()-t0:.1f}s。", flush=True)
    return model, tokenizer, knowledge


def infer(model, tokenizer, knowledge, task):
    import torch
    msgs = [
        {"role": "user", "content": [knowledge]},
        {"role": "assistant", "content": ["已了解。请给出任务。"]},
        {"role": "user", "content": [task]},
    ]
    with torch.inference_mode():
        reply = model.chat(image=None, msgs=msgs, tokenizer=tokenizer, sampling=False, stream=False)
    if not isinstance(reply, str):
        raise MissionRejected("九格返回值不是文本")
    return reply


def run_one(task, model, tokenizer, knowledge, args, ros=None):
    event = {"id": str(uuid.uuid4()), "time": dt.datetime.now(dt.timezone.utc).isoformat(),
             "task": task, "dry_run": args.dry_run}
    start = time.monotonic()
    try:
        reply = infer(model, tokenizer, knowledge, task)
        event["model_reply"] = reply
        commands = parse_model_reply(reply)
        validate_intent(task, commands)
        event["commands"] = commands
        if any(c.startswith("hit_balloon:") for c in commands):
            event["score_status"] = "unverified_no_judge_signal"
        event["inference_seconds"] = round(time.monotonic() - start, 3)
        print("[九格规划] " + " -> ".join(commands), flush=True)
        if args.dry_run:
            event["result"] = "DRY_RUN_OK"
            return True
        rospy, String, pub, result_event, result_box = ros
        if pub.get_num_connections() < 1:
            raise MissionRejected("/mission_command 没有行为树订阅者")
        # The callback is registered before publishing. Clear stale result.
        result_event.clear()
        result_box.clear()
        pub.publish(String(data="\n".join(commands)))
        event["published"] = True
        print("[执行] 已发布到 /mission_command，等待 /mission_result", flush=True)
        if not result_event.wait(args.timeout):
            event["result"] = "TIMEOUT_ABORT_REQUESTED"
            rospy.Publisher("/mission_abort", String, queue_size=1).publish(String(data="timeout"))
            raise MissionRejected(f"超过 {args.timeout}s 未收到任务结果，已请求中止")
        result = result_box.get("result", "UNKNOWN")
        event["result"] = result
        print(f"[结果] {result}", flush=True)
        if "score_status" in event:
            print("[计分] 未接入裁判反馈；此结果只表示行为树状态，不能证明气球得分。", flush=True)
        return result == "SUCCESS"
    except Exception as exc:
        event["error"] = f"{type(exc).__name__}: {exc}"
        print(f"[拒绝/失败] {event['error']}", file=sys.stderr, flush=True)
        return False
    finally:
        event["elapsed_seconds"] = round(time.monotonic() - start, 3)
        audit(event)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="九格规划和校验，但绝不发布飞行指令")
    ap.add_argument("--prompt", help="测试用；比赛时不传，先启动程序再粘贴官方提示词")
    ap.add_argument("--once", action="store_true", help="处理一次后退出")
    ap.add_argument("--timeout", type=int, default=240)
    args = ap.parse_args()
    ros = None
    if not args.dry_run:
        import rospy
        from std_msgs.msg import String
        rospy.init_node("jiuge_mission_online", anonymous=True)
        pub = rospy.Publisher("/mission_command", String, queue_size=1)
        result_event = threading.Event()
        result_box = {}

        def on_result(msg):
            result_box["result"] = msg.data
            result_event.set()

        rospy.Subscriber("/mission_result", String, on_result, queue_size=10)
        ros = (rospy, String, pub, result_event, result_box)
    model, tokenizer, knowledge = load_model()
    if args.prompt:
        ok = run_one(args.prompt, model, tokenizer, knowledge, args, ros)
        return 0 if ok else 1
    while True:
        try:
            task = input("\n[待命] 粘贴官方提示词；开赛时按回车（quit 退出）> ").strip()
        except (KeyboardInterrupt, EOFError):
            return 0
        if task.lower() in {"quit", "exit", "q"}:
            return 0
        if task:
            ok = run_one(task, model, tokenizer, knowledge, args, ros)
            if args.once:
                return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
