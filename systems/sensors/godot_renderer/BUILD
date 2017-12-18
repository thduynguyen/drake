cc_library(
    name="message",
    hdrs = ["message.hpp"],
    visibility=["//visibility:public"],
)

cc_library(
    name = "godotvis",
    hdrs = glob(["*.h", "*.inl"]),
    srcs = glob(["*.cpp"]),
    linkopts = [],
    deps = ["@godot//:godot"],
    copts = [],
    visibility=["//visibility:public"],
)

