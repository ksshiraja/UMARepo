#!/usr/bin/env python
# coding: utf-8

import argparse
import os
import random
import subprocess
import sys

UMA_REPO = 'UMA_REPO_DIR'


def failure():
    print("UMA BUILD FAILED!")
    sys.exit(1)


def lint(repo_path):
    try:
        subprocess.check_call(["lint.py", "-r", repo_path])
    except subprocess.CalledProcessError:
        failure()

def build(repo_path):
    try:
        subprocess.check_call(["build.py", "-r", repo_path])
    except subprocess.CalledProcessError:
        failure()

def test(repo_path):
    try:
        subprocess.check_call(["test.py", "-r", repo_path])
    except subprocess.CalledProcessError:
        failure()

def clean(repo_path, clean_all):
    if clean_all:
        try:
            subprocess.check_call(["clean.py", "-r", repo_path, "-a"])
        except subprocess.CalledProcessError:
            failure()
    else:
        try:
            subprocess.check_call(["clean.py", "-r", repo_path])
        except subprocess.CalledProcessError:
            failure()


def build_uma(repo_path, run_lint, run_build, run_tests, run_clean, run_clean_all):
    if run_clean or run_clean_all:
        clean(repo_path, run_clean_all)
        print("\n\n")
    if run_lint:
        lint(repo_path)
        print("\n\n")
    if run_build:
        build(repo_path)
        print("\n\n")
    if run_tests:
        test(repo_path)
        print("\n\n")
            
    print("UMA BUILD SUCCEEDED!")


def parse_args():
    parser = argparse.ArgumentParser(description="Build the entire uma repo.")
    parser.add_argument(
        '--disable-lint',
        action='store_true',
        help="Do not run linting"
    )
    parser.add_argument(
        '--disable-build',
        action='store_true',
        help="Do not run build"
    )
    parser.add_argument(
        '--disable-tests',
        action='store_true',
        help="Do not run tests"
    )
    parser.add_argument(
        '-c',
        '--clean',
        action='store_true',
        help="Clean UMARepo"
    )
    parser.add_argument(
        '--clean-all',
        action='store_true',
        help="Clean UMARepo and UMAThirdParty"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    repo_path = os.getenv(UMA_REPO)
    if not repo_path:
        raise EnvironmentError("The repo path environment variable is not set!")

    build_uma(repo_path, not args.disable_lint, not args.disable_build, not args.disable_tests, args.clean, args.clean_all)


if __name__ == "__main__":
    main()

