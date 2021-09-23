#!/usr/bin/env python
# coding: utf-8

import argparse
import os
import random
import subprocess
import sys
import time

UMA_REPO = 'UMA_REPO_DIR'
LINT_PROGRAM = 'catkin_make roslint_'

# Any package listed here will have the linter run on it
LINT_PACKAGES = [
    'lib',
    'uma_controls',
    'uma_navigation',
    'uma_perception',
    'uma_reasoning',
    'uma_tools'
]

LINT_PROGRAMS = [LINT_PROGRAM + package for package in LINT_PACKAGES]

def lint_failed(status):
    print("\n\nLinter returned status of {}".format(status))
    failed_emojis = [u"(╯°□°）╯︵ ┻━┻", u"(╥﹏╥)", u"x⸑x", u"( º﹃º )", u"(;´༎ຶД༎ຶ`)"]
    print("LINT FAILED {}".format(random.choice(failed_emojis).encode('utf-8').strip()))
    sys.exit(1)


def lint(repo_path):
    print("Starting lint...\n\n")

    for lint_program in LINT_PROGRAMS:
        lint_cmd = lint_program
        lint_status = subprocess.call(lint_cmd, shell=True, cwd=repo_path)
        if lint_status != 0:
            lint_failed(lint_status)

    print("\n\nLINT SUCCEEDED!")


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

    lint(repo_path)


if __name__ == "__main__":
    main()

