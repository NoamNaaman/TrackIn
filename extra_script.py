import os
from shutil import copyfile

def bin_copy(source, target, env):
    try:
        if source[0].attributes:
            if source[0].attributes.get("BIN"):
                copyfile(source[0].get_path(), os.path.join(target[0], "firmware.bin"))
    except AttributeError:
        pass

def generate_bin_after_build(env):
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", bin_copy)

Import("env")
generate_bin_after_build(env)
