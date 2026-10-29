# PlatformIO pre-script: builds the sketch folder named by custom_sketch_dir in platformio.ini
import os

Import("env")

sketch = env.GetProjectOption("custom_sketch_dir")
env.Replace(PROJECT_SRC_DIR=os.path.join(env.subst("$PROJECT_DIR"), sketch))
