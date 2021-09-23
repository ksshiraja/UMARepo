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
BUILD_PROGRAM = 'catkin_make install'


def build_failed(status):
    print("\n\nBuild return status of {}".format(status))
    failed_emojis = [u"(╯°□°）╯︵ ┻━┻", u"(╥﹏╥)", u"x⸑x", u"( º﹃º )", u"(;´༎ຶД༎ຶ`)"]
    print("BUILD_FAILED {}".format(random.choice(failed_emojis).encode('utf-8').strip()))
    sys.exit(1)


def build(repo_path, num_cores):
    # splitting arguments into list doesn't seem to work properly
    # ex. ['catkin_make', 'install'] does not seem to invoke install when using subprocess
    print("Starting build for {}...\n\n".format(repo_path))
    if num_cores:
        build_status = subprocess.call(BUILD_PROGRAM + " -j " + str(num_cores), shell=True, cwd=repo_path)
    else:
        build_status = subprocess.call(BUILD_PROGRAM, shell=True, cwd=repo_path)
    if build_status != 0:
        build_failed(build_status)

    print("\n\nBUILD SUCCESS!")


def parse_args():
    parser = argparse.ArgumentParser(description="Build the entire uma repo.")
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
        '-j',
        '--cores',
        help="Number of cores to use when building",
        default=0,
        type=int
    )
    return parser.parse_args()


def main():
    args = parse_args()

    third_party_path = args.third_party_path
    if not third_party_path:
        third_party_path = os.getenv(UMA_THIRD_PARTY)
        if not third_party_path:
            raise EnvironmentError("The third party path environment variable is not set!")

    build(third_party_path, args.cores)

    repo_path = args.repo_path
    if not repo_path:
        repo_path = os.getenv(UMA_REPO)
        if not repo_path:
            raise EnvironmentError("The repo path environment variable is not set!")

    build(repo_path, args.cores)


if __name__ == '__main__':
    main()
