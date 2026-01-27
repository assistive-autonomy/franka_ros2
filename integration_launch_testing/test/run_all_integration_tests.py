#!/usr/bin/env python3
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
Integration test runner that executes all controller tests sequentially.

This script runs all integration tests one after another and reports
pass/fail status for each test and an overall summary.

Usage:
    python3 run_all_integration_tests.py [--robot-ip IP] [--skip-gripper] [--use-rviz]

Exit codes:
    0 - All tests passed
    1 - One or more tests failed
"""

import argparse
import os
import subprocess
import sys
import time
from typing import List, Optional

try:
    from ament_index_python.packages import get_package_share_directory
    PACKAGE_SHARE_DIR = get_package_share_directory('integration_launch_testing')
    TEST_DIR = os.path.join(PACKAGE_SHARE_DIR, 'test')
except Exception:
    # Fallback for running from source
    TEST_DIR = os.path.dirname(os.path.abspath(__file__))

# Ensure test directory is on path for imports (e.g. test_output)
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

from test_output import (  # noqa: I101
    TestOutcome,
    TestResult,
    print_header,
    print_result_line,
    print_summary,
)


# Generic test file used for all controller tests
GENERIC_TEST_FILE = 'generic_controller_test.py'

# Define the tests to run in order
# move_to_start should run first to put robot in known position
# Each subsequent controller has needs_move_to_start=True to ensure it starts
# from a known position (the start position defined in move_to_start_example_controller)
CONTROLLER_TESTS = [
    {
        'name': 'move_to_start_example_controller',
        'description': 'Moves robot to start position',
        'needs_move_to_start': False,  # This IS the move_to_start controller
    },
    {
        'name': 'joint_position_example_controller',
        'description': 'Joint position control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'joint_velocity_example_controller',
        'description': 'Joint velocity control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'joint_impedance_example_controller',
        'description': 'Joint impedance control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'cartesian_pose_example_controller',
        'description': 'Cartesian pose control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'cartesian_velocity_example_controller',
        'description': 'Cartesian velocity control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'cartesian_orientation_example_controller',
        'description': 'Cartesian orientation control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'gravity_compensation_example_controller',
        'description': 'Gravity compensation test',
        'needs_move_to_start': True,
    },
]

GRIPPER_TESTS = [
    {
        'name': 'franka_gripper_position',
        'file': 'test_franka_gripper_position.py',  # Gripper uses dedicated test file
        'description': 'Gripper position control test',
    },
]


def run_launch_test(
    controller_name: str,
    robot_ip: str,
    timeout: int = 120,
    test_file: Optional[str] = None,
    use_rviz: bool = False,
    needs_move_to_start: bool = False,
) -> tuple:
    """
    Run a single launch_test and return (success, duration, output).

    Args
    ----
    controller_name : str
        Name of the controller to test
    robot_ip : str
        IP address of the robot
    timeout : int
        Maximum time to wait for test completion
    test_file : Optional[str]
        Optional specific test file (for non-generic tests like gripper)
    use_rviz : bool
        If True, launch RViz for visualization
    needs_move_to_start : bool
        If True, run move_to_start_example_controller before the test to ensure
        robot starts from a known position

    Returns
    -------
    tuple
        (success, duration, output)

    """
    # Use specific test file or generic test
    if test_file:
        test_path = os.path.join(TEST_DIR, test_file)
    else:
        test_path = os.path.join(TEST_DIR, GENERIC_TEST_FILE)

    if not os.path.exists(test_path):
        return False, 0.0, f'Test file not found: {test_path}'

    # Build command
    cmd = [
        'launch_test',
        test_path,
        f'robot_ip:={robot_ip}',
    ]

    # Add controller_name for generic tests
    if not test_file:
        cmd.append(f'controller_name:={controller_name}')

    # Add RViz option
    if use_rviz:
        cmd.append('use_rviz:=true')

    # Set environment variables for the test
    env = os.environ.copy()
    env['CONTROLLER_NAME'] = controller_name
    # Set flag for whether to run move_to_start before the controller test
    env['NEEDS_MOVE_TO_START'] = 'true' if needs_move_to_start else 'false'

    start_time = time.time()
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=TEST_DIR,
            env=env,
        )
        duration = time.time() - start_time

        # Check if test passed
        # launch_test returns 0 on success
        success = result.returncode == 0

        # Combine stdout and stderr for full output
        output = result.stdout + result.stderr

        return success, duration, output

    except subprocess.TimeoutExpired:
        duration = time.time() - start_time
        return False, duration, f'Test timed out after {timeout}s'
    except Exception as e:
        duration = time.time() - start_time
        return False, duration, f'Error running test: {str(e)}'


def run_all_tests(
    robot_ip: str,
    skip_gripper: bool = False,
    timeout: int = 120,
    verbose: bool = False,
    use_rviz: bool = False,
    max_retries: int = 1,
    cleanup_delay: float = 5.0,
) -> List[TestOutcome]:
    """
    Run all integration tests sequentially.

    Args
    ----
    robot_ip : str
        IP address of the robot
    skip_gripper : bool
        If True, skip gripper tests
    timeout : int
        Timeout for each test in seconds
    verbose : bool
        If True, print detailed output
    use_rviz : bool
        If True, launch RViz for visualization
    max_retries : int
        Maximum number of retries for failed tests (default: 1)
    cleanup_delay : float
        Delay between tests in seconds for cleanup (default: 5.0)

    Returns
    -------
    List[TestOutcome]
        Test outcome for each test.

    """
    outcomes: List[TestOutcome] = []

    # Combine test lists
    tests_to_run = CONTROLLER_TESTS.copy()
    if not skip_gripper:
        tests_to_run.extend(GRIPPER_TESTS)

    total_tests = len(tests_to_run)

    print(f'  Robot IP: {robot_ip}')
    print(f'  Tests to run: {total_tests}')
    print(f'  Skip gripper: {skip_gripper}')
    print(f'  Use RViz: {use_rviz}')
    print(f'  Max retries: {max_retries}')

    for i, test_info in enumerate(tests_to_run, 1):
        test_name = test_info['name']
        test_file = test_info.get('file')  # None for generic tests
        description = test_info['description']
        needs_move_to_start = test_info.get('needs_move_to_start', False)

        print_header(f'TEST {i}/{total_tests}: {test_name}', char='-')
        print(f'  Description: {description}')
        if needs_move_to_start:
            print('  Will run move_to_start first to ensure known start position')

        # Retry loop
        attempt = 0
        success = False
        duration = 0.0
        output = ''

        while attempt <= max_retries and not success:
            if attempt > 0:
                print(f'  Retry attempt {attempt}/{max_retries}...')
                # Longer delay before retry to allow full cleanup
                print(f'  Waiting {cleanup_delay * 2:.0f}s before retry...')
                time.sleep(cleanup_delay * 2)
            else:
                print('  Running...')

            success, duration, output = run_launch_test(
                test_name, robot_ip, timeout, test_file, use_rviz, needs_move_to_start
            )

            if not success:
                if attempt < max_retries:
                    print(f'  FAILED (attempt {attempt + 1}/{max_retries + 1})')
                    if verbose:
                        print()
                        print('  --- Failure Output ---')
                        for line in output.split('\n')[-30:]:  # Last 30 lines
                            print(f'  {line}')
                        print('  --- End Output ---')
                attempt += 1
            else:
                if attempt > 0:
                    print(f'  Passed on retry attempt {attempt}')

        if verbose or not success:
            print()
            print('  --- Test Output ---')
            # Indent the output
            for line in output.split('\n')[-50:]:  # Last 50 lines
                print(f'  {line}')
            print('  --- End Output ---')

        result = TestResult.PASSED if success else TestResult.FAILED
        outcome = TestOutcome(
            name=test_name,
            result=result,
            duration=duration,
            message=None if success else output[-500:] if output else 'Unknown error',
        )
        outcomes.append(outcome)

        print_result_line(test_name, result, duration)

        # Add delay between tests to allow cleanup
        if i < total_tests:
            print(f'  Waiting {cleanup_delay:.0f}s for cleanup...')
            time.sleep(cleanup_delay)

    return outcomes


def main():
    """Run the main program."""
    parser = argparse.ArgumentParser(
        description='Run all Franka ROS2 integration tests sequentially.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Run all tests with default robot IP
    python3 run_all_integration_tests.py

    # Run with specific robot IP
    python3 run_all_integration_tests.py --robot-ip 172.16.0.2

    # Skip gripper tests
    python3 run_all_integration_tests.py --skip-gripper

    # Verbose output
    python3 run_all_integration_tests.py -v

    # With RViz visualization
    python3 run_all_integration_tests.py --use-rviz

    # With retries for failed tests
    python3 run_all_integration_tests.py --retries 2
        """,
    )

    parser.add_argument(
        '--robot-ip',
        default='172.16.0.2',
        help='IP address or hostname of the robot (default: 172.16.0.2)',
    )
    parser.add_argument(
        '--skip-gripper',
        action='store_true',
        help='Skip gripper tests',
    )
    parser.add_argument(
        '--timeout',
        type=int,
        default=120,
        help='Timeout for each test in seconds (default: 120)',
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Print verbose output for all tests',
    )
    parser.add_argument(
        '--use-rviz',
        action='store_true',
        help='Launch RViz for visualization during tests',
    )
    parser.add_argument(
        '--retries',
        type=int,
        default=1,
        help='Maximum number of retries for failed tests (default: 1)',
    )
    parser.add_argument(
        '--cleanup-delay',
        type=float,
        default=5.0,
        help='Delay between tests in seconds for cleanup (default: 5.0)',
    )

    args = parser.parse_args()

    # Run all tests
    outcomes = run_all_tests(
        robot_ip=args.robot_ip,
        skip_gripper=args.skip_gripper,
        timeout=args.timeout,
        verbose=args.verbose,
        use_rviz=args.use_rviz,
        max_retries=args.retries,
        cleanup_delay=args.cleanup_delay,
    )

    # Print summary and get overall result
    all_passed = print_summary(outcomes)

    # Exit with appropriate code
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
