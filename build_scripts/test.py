#!/usr/bin/env python
# coding: utf-8

import argparse
import os
import random
import subprocess
import sys
import time
import rosgraph
import socket

UMA_REPO = 'UMA_REPO_DIR'
TEST_PROGRAM = 'catkin_make run_tests_'
TEST_STATUS = 'catkin_test_results'

# Any package listed here will have tests run on it
TEST_PACKAGES = [
    'lib',
    'uma_controls',
    'uma_navigation',
    'uma_perception',
    'uma_reasoning',
    'uma_tools'
]

TEST_PROGRAMS = [TEST_PROGRAM + package for package in TEST_PACKAGES]

def tests_failed(status):
    print("\n\nTests returned status of {}".format(status))
    failed_emojis = [u"(╯°□°）╯︵ ┻━┻", u"(╥﹏╥)", u"x⸑x", u"( º﹃º )", u"(;´༎ຶД༎ຶ`)"]
    print("TESTS FAILED {}".format(random.choice(failed_emojis).encode('utf-8').strip()))
    sys.exit(1)


def roscore_ready():
    try:
        rosgraph.Master('/rostopic').getPid()
        return True
    except socket.error:
        pass
    return False


def test(repo_path):
    print("Starting tests...\n\n")

    if not roscore_ready():
        print("Starting ROS...")
        ros_proc = subprocess.Popen('roscore')
        local_roscore_started = True
    else:
        local_roscore_started = False

    while not roscore_ready():
        time.sleep(1)

    for test_cmd in TEST_PROGRAMS:
        test_status_cmd = TEST_STATUS
        subprocess.call(test_cmd, shell=True, cwd=repo_path)
        test_status = subprocess.call(test_status_cmd, shell=True, cwd=repo_path)
    
    if local_roscore_started:
        # wait for process to actually shut down
        print("\nEnding ROS...")
        ros_proc.terminate()
        ros_proc.wait()

    if test_status != 0:
        tests_failed(test_status)

    print("\n\nTESTS SUCCEEDED!")


def parse_args():
    parser = argparse.ArgumentParser(description="Build the entire uma repo.")
    parser.add_argument(
        '-r',
        '--repo-path',
        help="Absolute path to the repo"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    repo_path = args.repo_path
    if not repo_path:
        repo_path = os.getenv(UMA_REPO)
        if not repo_path:
            raise EnvironmentError("The repo path environment variable is not set!")

    test(repo_path)


if __name__ == "__main__":
    main()


