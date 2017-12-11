cc_binary(
    name = "hello",
    srcs = ["hello.cpp"],
    deps = ["//godotvis:message"],
)

cc_binary(
    name = "example",
    srcs = ["main.cpp"],
    deps = ["//godotvis:godotvis",
           "@godot//:godot"],
    copts = [],
    linkopts = ["-lglfw", "-ldl", "-lpng", "-lz"]
)
