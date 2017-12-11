cc_test(
    name = "test_gtest",
    srcs = glob(["**/*.cpp"]),
    deps = ["//godotvis:message",
           "@googletest//:gtest_main",
           #"@godot//:godot"
            ],
    linkopts = ["-lglfw", "-ldl", "-lpng", "-lz"]
)
