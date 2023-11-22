import typing
from typing import Callable, List
from dataclasses import dataclass

@dataclass
class Plan():
    name: string
    subtasks: List[skill] = field(default_factory=list)
    fsm: