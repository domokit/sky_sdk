import subprocess
import os

gn_in = open("BUILD.input.gn", "rb")
gn_file = gn_in.read()
gn_in.close()


def get_files(path, exclude=[]):
    cmd = ["git", "ls-files", "--"]
    for ex in exclude:
        cmd.append(":!%s" % ex)
    cmd.append(path)
    git_ls = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        cwd=os.path.join(os.environ["FUCHSIA_DIR"], "third_party", "protobuf"))
    sed1 = subprocess.Popen(
        ["sed", "s/^/\"/"], stdin=git_ls.stdout, stdout=subprocess.PIPE)
    return subprocess.check_output(["sed", "s/$/\",/"], stdin=sed1.stdout)


gn_file = gn_file.replace(
    b"PROTOBUF_LITE_PUBLIC",
    get_files(
        "src/google/protobuf/*.h",
        exclude=["*/compiler/*", "*/testing/*", "*/util/*"]))
gn_file = gn_file.replace(
    b"PROTOBUF_FULL_PUBLIC",
    get_files(
        "src/google/protobuf/*.h", exclude=["*/compiler/*", "*/testing/*"]))
gn_file = gn_file.replace(
    b"PROTOC_LIB_SOURCES",
    get_files(
        "src/google/protobuf/compiler/*.cc",
        exclude=["*/main.cc", "*test*", "*mock*"]))

gn_file = subprocess.check_output(["gn", "format", "--stdin"], input=gn_file)

gn_out = open("BUILD.gn", "wb")
gn_out.write(
    b"# THIS FILE IS GENERATED FROM BUILD.input.gn BY gen.py\n# EDIT BUILD.input.gn FIRST AND THEN RUN gen.py\n#\n#\n"
)
gn_out.write(gn_file)
