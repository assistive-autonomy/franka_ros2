#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Output formatting and printing for integration test runs.

This module provides formatted headers, result lines, and summary output
for the run_all_integration_tests runner.
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional


class TestResult(Enum):
    PASSED = 'PASSED'
    FAILED = 'FAILED'
    SKIPPED = 'SKIPPED'


@dataclass
class TestOutcome:
    """Stores the outcome of a single test."""

    name: str
    result: TestResult
    duration: float
    message: Optional[str] = None


# ANSI color codes
_GREEN = '\033[92m'
_RED = '\033[91m'
_YELLOW = '\033[93m'
_RESET = '\033[0m'


def print_header(text: str, char: str = '=', width: int = 70) -> None:
    """Print a formatted header."""
    print()
    print(char * width)
    print(f' {text}')
    print(char * width)


def print_result_line(name: str, result: TestResult, duration: float) -> None:
    """Print a single result line with color coding."""
    color = _GREEN if result == TestResult.PASSED else (
        _RED if result == TestResult.FAILED else _YELLOW
    )

    status = f'{color}{result.value}{_RESET}'
    print(f'  {name:<45} {status:<20} ({duration:.1f}s)')


def print_summary(outcomes: List[TestOutcome]) -> bool:
    """
    Print test summary and return overall success.

    Args
    ----
    outcomes : List[TestOutcome]
        List of test outcomes

    Returns
    -------
    bool
        True if all tests passed, False otherwise.

    """
    print_header('TEST SUMMARY')

    passed = sum(1 for o in outcomes if o.result == TestResult.PASSED)
    failed = sum(1 for o in outcomes if o.result == TestResult.FAILED)
    skipped = sum(1 for o in outcomes if o.result == TestResult.SKIPPED)
    total = len(outcomes)
    total_duration = sum(o.duration for o in outcomes)

    # Print individual results
    for outcome in outcomes:
        print_result_line(outcome.name, outcome.result, outcome.duration)

    print()
    print(f'  Total: {total} | Passed: {passed} | Failed: {failed} | Skipped: {skipped}')
    print(f'  Total duration: {total_duration:.1f}s')

    # Print failed test details
    if failed > 0:
        print()
        print('  Failed tests:')
        for outcome in outcomes:
            if outcome.result == TestResult.FAILED:
                print(f'    - {outcome.name}')

    # Final verdict
    print()
    if failed == 0 and passed > 0:
        print(f'  {_GREEN}ALL TESTS PASSED{_RESET}')
        return True
    else:
        print(f'  {_RED}TESTS FAILED{_RESET}')
        return False
