"""所有任务状态的基类。"""

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from interface import PX4Interface


@dataclass
class ExecutionResult:
    """状态 execute() 的返回值。

    四种情况：
        done=True              → 状态正常完成，弹出栈
        interrupt=<State>      → 抢占当前状态，suspend + push
        done=False, 无其他      → 继续执行，下一周期再调 execute
        error="..."            → 状态异常，触发 _handle_state_error
    """

    done: bool = False
    interrupt: Optional["BaseState"] = None
    error: Optional[str] = None


class BaseState(ABC):
    """所有状态的抽象基类。"""

    def __init__(self, name: str, timeout_s: Optional[float] = None):
        self.name = name
        self.timeout_s = timeout_s  # None = 无超时限制
        self._enter_time: Optional[float] = None
        self._entered: bool = False
        self.is_completed = False
        self.error: Optional[str] = None
        self.allow_disarmed: bool = False  # 允许 disarm 的状态（如 RTL 着陆后）

    async def enter(self, interface: "PX4Interface"):
        """进入状态时调用（仅一次）。"""
        self._enter_time = time.monotonic()
        self._entered = True
        self.is_completed = False
        self.error = None

    @abstractmethod
    async def execute(self, interface: "PX4Interface") -> ExecutionResult:
        """
        每个主循环周期调用。
        返回 ExecutionResult 指示下一步动作。
        """
        ...

    async def exit(self, interface: "PX4Interface"):
        """退出状态时调用（被弹出栈时）。可重写以进行清理。"""
        pass

    async def suspend(self, interface: "PX4Interface"):
        """被新状态抢占时调用。保存断点。默认空操作。"""
        pass

    async def resume(self, interface: "PX4Interface"):
        """上层状态弹出后恢复执行。从断点继续。默认空操作。"""
        pass

    def elapsed(self) -> float:
        """自进入此状态以来经过的秒数。"""
        if self._enter_time is None:
            return 0.0
        return time.monotonic() - self._enter_time

    def is_timed_out(self) -> bool:
        """检查状态是否已超过其时间限制。"""
        if self.timeout_s is None:
            return False
        return self.elapsed() > self.timeout_s
