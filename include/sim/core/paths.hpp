#pragma once

#include <string>

// CMake 会把工程根目录编进来（见 CMakeLists.txt 的 add_compile_definitions）。
// 这样无论从 CLion（工作目录通常是 build 目录）还是命令行（工作目录是工程根）
// 启动，`configs/sim/xxx.yaml` 这类相对路径都能解析到同一份文件。
#ifndef SIM_PROJECT_SOURCE_DIR
#define SIM_PROJECT_SOURCE_DIR "."
#endif

namespace sim {

/// 工程根目录（编译期写入的绝对路径）。
inline const std::string& projectRoot() {
    static const std::string root{SIM_PROJECT_SOURCE_DIR};
    return root;
}

/// 相对路径拼到工程根；绝对路径原样返回。
/// 语义与 auto_aim_ 的 tools::project_path() 一致：
///   "configs/sim/a.yaml"  -> "<project>/configs/sim/a.yaml"
///   "./configs/sim/a.yaml" -> 同上（去掉开头的 ./）
///   "/abs/path/a.yaml"    -> 原样
inline std::string resolvePath(const std::string& path) {
    if (path.empty()) return path;
    if (path.front() == '/') return path;

    std::string rel = path;
    if (rel.size() >= 2 && rel[0] == '.' && rel[1] == '/') rel = rel.substr(2);
    return projectRoot() + "/" + rel;
}

}  // namespace sim
