import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", required=True)
    options = parser.parse_args()
    result = subprocess.run(
        [options.runner],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode == 0:
        raise SystemExit("runner accepted missing options")


if __name__ == "__main__":
    main()
