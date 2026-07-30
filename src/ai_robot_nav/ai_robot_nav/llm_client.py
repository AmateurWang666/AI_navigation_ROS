"""Ollama 请求封装，以及对模型语义评估的容错解析。

刻意不向模型索取速度——速度由 navigator.py 从激光几何关系算出。从这里交回上层
的只有一个风险等级和一个方向偏好，两者都只能让机器人更保守。

模型输出属于不可信输入，因此解析一律"失败即关闭"：宁可拒绝这次评估、让控制退回
纯激光，也不要把半懂不懂的内容塞进控制回路。

模块不依赖 rclpy，可以脱离 ROS 图和 Ollama 服务直接单元测试。
"""

import json
import re
from typing import Any, Dict, NamedTuple, Optional

import requests

# 白名单校验：不在集合内的取值一律视为非法，不去猜测模型想表达什么。
VALID_HAZARDS = frozenset({'NONE', 'CAUTION', 'BLOCKED'})
VALID_DIRECTIONS = frozenset({'LEFT', 'RIGHT', 'STRAIGHT', 'NONE'})

# 小模型即便被要求只输出 JSON，也常把它包在 markdown 代码块里，或在前后加一段
# 客套话。这两个正则专门用来把 JSON 从这类外壳里剥出来。
_FENCE_RE = re.compile(r'^```(?:json)?\s*|\s*```$', re.IGNORECASE)
_OBJECT_RE = re.compile(r'\{.*\}', re.DOTALL)


class DecisionError(ValueError):
    """模型响应无法转换成可用评估时抛出。"""


class Assessment(NamedTuple):
    """模型返回的语义评估。字段刻意只有这三个，见 parse_assessment 的说明。"""

    hazard: str               # NONE / CAUTION / BLOCKED
    preferred_direction: str  # LEFT / RIGHT / STRAIGHT / NONE
    description: str          # 自然语言说明，仅用于日志


def extract_json(text: str) -> Dict[str, Any]:
    """解析 ``text`` 中的第一个 JSON 对象，容忍代码块包裹与夹杂的散文。"""
    if not text or not text.strip():
        raise DecisionError('empty response')

    # 三种候选按"从保守到宽松"依次尝试：原样、剥掉代码块围栏、正则抠出最外层
    # 花括号。顺序有意义——先试原样，合法输入就不会被正则误伤。
    stripped = text.strip()
    candidates = [stripped, _FENCE_RE.sub('', stripped).strip()]
    match = _OBJECT_RE.search(stripped)
    if match:
        candidates.append(match.group(0))

    for candidate in candidates:
        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        # 必须是对象。合法 JSON 但顶层是数组或标量的（模型偶尔会回一个列表），
        # 同样无法取字段，继续尝试下一个候选。
        if isinstance(parsed, dict):
            return parsed

    raise DecisionError(f'no JSON object found in {text[:200]!r}')


def parse_assessment(text: str) -> Assessment:
    """把模型响应校验成一个风险等级和一个方向偏好。

    只读取白名单内的字段，所以模型即使额外返回 ``linear_x`` 之类的速度字段，
    也无法夹带进控制回路——多出来的键在这里被静默丢弃。
    """
    data = extract_json(text)

    # hazard 直接决定是否降速、是否判定前方不可通行，非法值只能拒绝，交给调用方
    # 重试或退回纯激光。这里绝不能"猜一个默认值"。
    hazard = str(data.get('hazard', '')).strip().upper()
    if hazard not in VALID_HAZARDS:
        raise DecisionError(f'unknown hazard {hazard!r}')

    # 方向偏好只在左右平局时才起作用，代价很小。非法值降级为 NONE 就够了，不值得
    # 为它浪费一次以秒计的推理重试。
    direction = str(data.get('preferred_direction', 'NONE')).strip().upper()
    if direction not in VALID_DIRECTIONS:
        direction = 'NONE'

    return Assessment(
        hazard=hazard,
        preferred_direction=direction,
        description=str(data.get('description', '')),
    )


class OllamaClient:
    """Ollama /api/generate 接口的轻量封装。

    只做请求与取字段，不做任何校验和重试：校验在 parse_assessment，重试策略由
    节点决定，这样本类可以脱离 ROS 单独复用。
    """

    def __init__(
        self,
        url: str,
        model: str,
        timeout: float,
        keep_alive: str = '30m',
        num_predict: int = 128,
        temperature: float = 0.1,
    ):
        self.url = url
        self.model = model
        self.timeout = timeout
        self.keep_alive = keep_alive
        self.num_predict = num_predict
        self.temperature = temperature
        # 复用 Session，让 TCP 连接在两次推理之间保持热连接，省掉每次的握手开销。
        self._session = requests.Session()

    def generate(self, prompt: str, system: str, image_b64: Optional[str] = None) -> str:
        """执行一次补全，返回模型的原始文本响应。"""
        payload = {
            'model': self.model,
            'prompt': prompt,
            'system': system,
            # 关掉流式：这里只要最终的完整 JSON，逐 token 拼接没有意义。
            'stream': False,
            # 把解码约束在合法 JSON 上，几乎消除了所有解析失败。需要 Ollama
            # >= 0.1.24；旧版本会忽略该字段，此时就靠 extract_json 兜底。
            'format': 'json',
            # 避免模型在两次推理之间被卸载又重新加载——首次加载可达数十秒，
            # 落在控制回路里就是一段没有视觉输入的空窗。
            'keep_alive': self.keep_alive,
            'options': {
                # 温度压到接近确定性：这里要的是稳定的分类结果，不是创造力。
                'temperature': self.temperature,
                # 限制输出长度：评估 JSON 很短，放开只会拖长推理时间。
                'num_predict': self.num_predict,
            },
        }
        if image_b64:
            payload['images'] = [image_b64]

        response = self._session.post(self.url, json=payload, timeout=self.timeout)
        # 让 4xx/5xx 也抛 RequestException，与连接失败在调用方走同一条处理路径。
        response.raise_for_status()
        # 字段缺失时返回空串，由 parse_assessment 统一按"空响应"拒绝。
        return response.json().get('response', '')

    def close(self):
        """释放底层连接。节点关闭时调用，避免留下悬挂的 socket。"""
        self._session.close()
