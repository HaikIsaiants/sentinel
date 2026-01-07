import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--agent", required=True)
    options = parser.parse_args()
    result = subprocess.run(
        [options.agent, "--agent-id", "agent-a"],
        input=b"",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0 or result.stdout:
        raise SystemExit("agent EOF smoke test failed")


if __name__ == "__main__":
    main()
