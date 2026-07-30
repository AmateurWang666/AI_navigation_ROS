"""Ollama request helper plus tolerant parsing of the model's semantic assessment.

The model is deliberately not asked for velocities; navigator.py derives those
from LiDAR geometry. What comes back here is only a hazard grade and a direction
preference, both of which can only make the robot more conservative.

Kept free of rclpy so it can be unit tested without a ROS graph or a running
Ollama server.
"""

import json
import re
from typing import Any, Dict, NamedTuple, Optional

import requests

VALID_HAZARDS = frozenset({'NONE', 'CAUTION', 'BLOCKED'})
VALID_DIRECTIONS = frozenset({'LEFT', 'RIGHT', 'STRAIGHT', 'NONE'})

_FENCE_RE = re.compile(r'^```(?:json)?\s*|\s*```$', re.IGNORECASE)
_OBJECT_RE = re.compile(r'\{.*\}', re.DOTALL)


class DecisionError(ValueError):
    """Raised when a model response cannot be turned into a usable assessment."""


class Assessment(NamedTuple):
    hazard: str
    preferred_direction: str
    description: str


def extract_json(text: str) -> Dict[str, Any]:
    """Parse the first JSON object in ``text``, tolerating fences and stray prose."""
    if not text or not text.strip():
        raise DecisionError('empty response')

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
        if isinstance(parsed, dict):
            return parsed

    raise DecisionError(f'no JSON object found in {text[:200]!r}')


def parse_assessment(text: str) -> Assessment:
    """Validate a model response into a hazard grade and a direction preference."""
    data = extract_json(text)

    hazard = str(data.get('hazard', '')).strip().upper()
    if hazard not in VALID_HAZARDS:
        raise DecisionError(f'unknown hazard {hazard!r}')

    # A bad direction only costs a tie-break, so fall back instead of retrying.
    direction = str(data.get('preferred_direction', 'NONE')).strip().upper()
    if direction not in VALID_DIRECTIONS:
        direction = 'NONE'

    return Assessment(
        hazard=hazard,
        preferred_direction=direction,
        description=str(data.get('description', '')),
    )


class OllamaClient:
    """Thin wrapper over Ollama's /api/generate endpoint."""

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
        # A session keeps the TCP connection warm between inferences.
        self._session = requests.Session()

    def generate(self, prompt: str, system: str, image_b64: Optional[str] = None) -> str:
        """Run one completion and return the raw response text."""
        payload = {
            'model': self.model,
            'prompt': prompt,
            'system': system,
            'stream': False,
            # Constrains decoding to valid JSON, which removes almost every parse failure.
            'format': 'json',
            # Prevents the model being unloaded and reloaded between inferences.
            'keep_alive': self.keep_alive,
            'options': {
                'temperature': self.temperature,
                'num_predict': self.num_predict,
            },
        }
        if image_b64:
            payload['images'] = [image_b64]

        response = self._session.post(self.url, json=payload, timeout=self.timeout)
        response.raise_for_status()
        return response.json().get('response', '')

    def close(self):
        self._session.close()
