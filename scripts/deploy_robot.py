#!/usr/bin/env python3
"""Sync Zeus code to the robot over SSH and restart robot.py."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path


ZEUS_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HOST_ENV = "ZEUS_ROBOT_HOST"
DEFAULT_PATH_ENV = "ZEUS_ROBOT_PATH"
EXCLUDES_FILE = Path(__file__).resolve().parent / "rsync_excludes.txt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Deploy Zeus robot code over SSH using rsync."
    )
    parser.add_argument(
        "--host",
        default=os.environ.get(DEFAULT_HOST_ENV),
        help=f"SSH target, e.g. robot@192.168.4.1 (env: {DEFAULT_HOST_ENV})",
    )
    parser.add_argument(
        "--remote-path",
        default=os.environ.get(DEFAULT_PATH_ENV),
        help=f"Remote project directory (env: {DEFAULT_PATH_ENV})",
    )
    parser.add_argument(
        "--sync-only",
        action="store_true",
        help="Sync files only; do not restart robot.py",
    )
    parser.add_argument(
        "--install-deps",
        action="store_true",
        help="Run pip install -r requirements.txt on the robot after syncing",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be synced without changing the robot",
    )
    return parser.parse_args()


def require_target(host: str | None, remote_path: str | None) -> tuple[str, str]:
    if not host:
        raise SystemExit(
            f"Missing SSH host. Pass --host or set {DEFAULT_HOST_ENV}."
        )
    if not remote_path:
        raise SystemExit(
            f"Missing remote path. Pass --remote-path or set {DEFAULT_PATH_ENV}."
        )
    return host, remote_path


def run_command(command: list[str], dry_run: bool = False) -> None:
    printable = " ".join(shlex.quote(part) for part in command)
    print(f"$ {printable}")
    if dry_run:
        return
    subprocess.run(command, check=True)


def sync_project(host: str, remote_path: str, dry_run: bool) -> None:
    destination = f"{host}:{remote_path.rstrip('/')}/"
    command = [
        "rsync",
        "-avz",
        "--delete",
        f"--exclude-from={EXCLUDES_FILE}",
    ]
    if dry_run:
        command.append("--dry-run")
    command.extend([f"{ZEUS_ROOT}/", destination])
    print(f"Syncing {ZEUS_ROOT} -> {destination}")
    run_command(command, dry_run=dry_run)


def run_remote(host: str, remote_path: str, remote_command: str, dry_run: bool) -> None:
    command = [
        "ssh",
        host,
        f"cd {shlex.quote(remote_path)} && {remote_command}",
    ]
    run_command(command, dry_run=dry_run)


def install_dependencies(host: str, remote_path: str, dry_run: bool) -> None:
    print("Installing remote dependencies...")
    run_remote(
        host,
        remote_path,
        ".venv/bin/pip install -r requirements.txt",
        dry_run=dry_run,
    )


def restart_robot(host: str, remote_path: str, dry_run: bool) -> None:
    print("Restarting robot.py on the robot...")
    restart_command = (
        "pkill -f 'python.*robot.py' || true; "
        "nohup .venv/bin/python robot.py > robot.log 2>&1 &"
    )
    run_remote(host, remote_path, restart_command, dry_run=dry_run)


def main() -> int:
    args = parse_args()
    host, remote_path = require_target(args.host, args.remote_path)

    sync_project(host, remote_path, dry_run=args.dry_run)

    if args.install_deps:
        install_dependencies(host, remote_path, dry_run=args.dry_run)

    if not args.sync_only:
        restart_robot(host, remote_path, dry_run=args.dry_run)

    if args.dry_run:
        print("Dry run complete. No changes were made on the robot.")
    else:
        print("Deploy complete.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
