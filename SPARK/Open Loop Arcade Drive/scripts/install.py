import os
import tomllib

with open("pyproject.toml", "rb") as f:
  toml = tomllib.load(f)
robotpy_version = toml["tool"]["robotpy"]["robotpy_version"]
os.system(f'python -m pip install --upgrade pip robotpy=={ robotpy_version } robotpy-rev==2025.0.0b4 robotpy-wpiutil==2025.0.0b3 photonlibpy==2025.0.0b8 certifi')
