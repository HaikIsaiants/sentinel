import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", required=True)
    options = parser.parse_args()
    help_result = subprocess.run(
        [options.runner, "--help"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if help_result.returncode != 0 or "usage:" not in help_result.stdout:
        raise SystemExit("runner help smoke test failed")
    error_result = subprocess.run(
        [options.runner],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if error_result.returncode == 0:
        raise SystemExit("runner accepted missing options")


if __name__ == "__main__":
    main()
