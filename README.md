# Adaptive-Robot-Control
Adaptive-Control-Simulation


from dataclasses import dataclass
from enum import Enum
from typing import List, Optional
import math
import time


class OperatingMode(str, Enum):
    """ロボットの動作モード"""
    CRUISE = "CRUISE"
    CAUTIOUS = "CAUTIOUS"
    STOP = "STOP"


@dataclass(frozen=True)
class EnvironmentTelemetry:
    """環境センサーから取得したデータ"""
    wind_speed: float
    obstacle_density: float
    sensor_confidence: float


@dataclass(frozen=True)
class DecisionRecord:
    """判断結果の記録"""
    mode: OperatingMode
    reason: str
    processing_time_ms: float


class AdaptiveRobotController:
    """環境に応じて動作モードを切り替える制御シミュレーション"""

    def __init__(
        self,
        max_wind_speed: float = 20.0,
        max_obstacle_density: float = 1.0,
        min_sensor_confidence: float = 0.7,
    ):
        if max_wind_speed <= 0:
            raise ValueError("max_wind_speed must be positive")

        if not 0 <= max_obstacle_density <= 1:
            raise ValueError(
                "max_obstacle_density must be between 0 and 1"
            )

        if not 0 <= min_sensor_confidence <= 1:
            raise ValueError(
                "min_sensor_confidence must be between 0 and 1"
            )

        self.max_wind_speed = max_wind_speed
        self.max_obstacle_density = max_obstacle_density
        self.min_sensor_confidence = min_sensor_confidence

        self.current_mode = OperatingMode.STOP
        self.history: List[DecisionRecord] = []

    @staticmethod
    def _is_finite_number(value: float) -> bool:
        """数値が有限値かどうかを確認する"""
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )

    def validate_telemetry(
        self,
        telemetry: EnvironmentTelemetry,
    ) -> Optional[str]:
        """入力データを検証し、問題があれば理由を返す"""

        values = {
            "wind_speed": telemetry.wind_speed,
            "obstacle_density": telemetry.obstacle_density,
            "sensor_confidence": telemetry.sensor_confidence,
        }

        for name, value in values.items():
            if not self._is_finite_number(value):
                return f"{name} is not a valid finite number"

        if telemetry.wind_speed < 0:
            return "wind_speed cannot be negative"

        if not 0 <= telemetry.obstacle_density <= 1:
            return "obstacle_density must be between 0 and 1"

        if not 0 <= telemetry.sensor_confidence <= 1:
            return "sensor_confidence must be between 0 and 1"

        return None

    def select_mode(
        self,
        telemetry: EnvironmentTelemetry,
    ) -> tuple[OperatingMode, str]:
        """環境情報に基づいて動作モードを選ぶ"""

        error = self.validate_telemetry(telemetry)

        if error is not None:
            return OperatingMode.STOP, f"Invalid telemetry: {error}"

        if (
            telemetry.sensor_confidence
            < self.min_sensor_confidence
        ):
            return (
                OperatingMode.STOP,
                "Sensor confidence is below the safe threshold",
            )

        if telemetry.wind_speed > self.max_wind_speed:
            return (
                OperatingMode.CAUTIOUS,
                "Wind speed exceeds the normal operating threshold",
            )

        if (
            telemetry.obstacle_density
            > self.max_obstacle_density
        ):
            return (
                OperatingMode.CAUTIOUS,
                "Obstacle density exceeds the configured threshold",
            )

        if telemetry.obstacle_density >= 0.6:
            return (
                OperatingMode.CAUTIOUS,
                "Many obstacles detected",
            )

        return (
            OperatingMode.CRUISE,
            "Environmental conditions are within configured limits",
        )

    def update(
        self,
        telemetry: EnvironmentTelemetry,
    ) -> DecisionRecord:
        """判断し、状態を更新して結果を記録する"""

        start_time = time.perf_counter()

        new_mode, reason = self.select_mode(telemetry)
        self.current_mode = new_mode

        elapsed_ms = (
            time.perf_counter() - start_time
        ) * 1000.0

        record = DecisionRecord(
            mode=new_mode,
            reason=reason,
            processing_time_ms=elapsed_ms,
        )

        self.history.append(record)
        return record

    def print_status(self, record: DecisionRecord) -> None:
        """判断結果を表示する"""

        print(f"Mode: {record.mode.value}")
        print(f"Reason: {record.reason}")
        print(
            f"Processing time: "
            f"{record.processing_time_ms:.4f} ms"
        )
        print("-" * 50)


def run_demo() -> None:
    """サンプル環境で動作を確認する"""

    controller = AdaptiveRobotController()

    test_cases = [
        (
            "Normal environment",
            EnvironmentTelemetry(
                wind_speed=3.0,
                obstacle_density=0.1,
                sensor_confidence=0.99,
            ),
        ),
        (
            "Crowded environment",
            EnvironmentTelemetry(
                wind_speed=5.0,
                obstacle_density=0.8,
                sensor_confidence=0.95,
            ),
        ),
        (
            "Strong wind",
            EnvironmentTelemetry(
                wind_speed=25.0,
                obstacle_density=0.2,
                sensor_confidence=0.98,
            ),
        ),
        (
            "Unreliable sensor",
            EnvironmentTelemetry(
                wind_speed=2.0,
                obstacle_density=0.1,
                sensor_confidence=0.3,
            ),
        ),
    ]

    for name, telemetry in test_cases:
        print(f"\nTest: {name}")
        record = controller.update(telemetry)
        controller.print_status(record)

    print(f"Total decisions recorded: {len(controller.history)}")


if __name__ == "__main__":
    run_demo()
