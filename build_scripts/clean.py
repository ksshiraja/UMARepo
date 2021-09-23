#!/usr/bin/env python
# coding: utf-8

import argparse
import os
import random
import subprocess
import sys

# TODO(rmallya@umich.edu) Should probably make some constants file for getting environment variable names?
UMA_REPO = 'UMA_REPO_DIR'
UMA_THIRD_PARTY = 'UMA_THIRD_PARTY_DIR'
CLEAN_PROGRAM = 'catkin_make clean'

def clean_failed(status):
    print("\n\nClean return status of {}".format(status))
    failed_emojis = [u"(╯°□°）╯︵ ┻━┻", u"(╥﹏╥)", u"x⸑x", u"( º﹃º )", u"(;´༎ຶД༎ຶ`)"]
    print("CLEAN_FAILED {}".format(random.choice(failed_emojis).encode('utf-8').strip()))
    sys.exit(1)


def clean(repo_path):
    print("Starting clean for {}...\n\n".format(repo_path))
    clean_cmd = CLEAN_PROGRAM
    clean_status = subprocess.call(clean_cmd, shell=True, cwd=repo_path)
    if clean_status != 0:
        clean_failed(clean_status)

    print("\n\nCLEAN SUCCESS!")


def parse_args():
    parser = argparse.ArgumentParser(description="Clean the entire uma repo.")
    parser.add_argument(
        '-r',
        '--repo-path',
        help="Absolute path to the repo"
    )
    parser.add_argument(
        '-t',
        '--third-party-path',
        help="Absolute path to the third party repo"
    )
    parser.add_argument(
        '-a',
        '--all',
        action='store_true',
        help="Run the clean on UMARepo and UMAThirdParty"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.all:
        third_party_path = args.third_party_path
        if not third_party_path:
            third_party_path = os.getenv(UMA_THIRD_PARTY)
            if not third_party_path:
                raise EnvironmentError("The third party path environment variable is not set!")

        clean(third_party_path)

    repo_path = args.repo_path
    if not repo_path:
        repo_path = os.getenv(UMA_REPO)
        if not repo_path:
            raise EnvironmentError("The repo path environment variable is not set!")

    clean(repo_path)


if __name__ == '__main__':
    main()
