//
// Created by xmyci on 20/02/2024.
//

#include "tbb/tbb.h"
#include "CLI11.hpp"

#include "basic/typedef.h"
#include "pipeline/run pipeline/run_pipeline.h"
#include "geogram/basic/common.h"
#include "geogram/basic/command_line_args.h"

#include <csignal>

std::vector<base_type::Triangle_Soup_Mesh> input_mesh_array;
base_type::Triangle_Soup_Mesh input_mesh_domain;

#ifdef BUILD_LIB

#include "main.h"

int dll_main(std::string path) {
#else


int main(int argc, char **argv) {

#endif


    GEO::initialize();
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");
    GEO::CmdLine::import_arg_group("co3ne");
    GEO::CmdLine::import_arg_group("pre");
    GEO::CmdLine::import_arg_group("post");
    GEO::CmdLine::import_arg_group("remesh");
//    GEO::CmdLine::set_arg("log:quiet", true);
    std::string file_path = "default";

#ifdef BUILD_LIB
    file_path = path;
#else
    CLI::App app{"App description"};
    argv = app.ensure_utf8(argv);
    app.add_option("-f,--file", file_path, "A help string");
    CLI11_PARSE(app, argc, argv);
#endif

    if (strcmp(file_path.c_str(), "default") == 0) {
        logger().warn("input config path can is null, using current dir!");
        file_path = "./default_config.json";
    }



    const size_t MB = 1024 * 1024;
    const size_t stack_size = 64 * MB;

    int num_threads = std::max(1u, std::thread::hardware_concurrency());

    logger().info("TBB threads: {}", num_threads);

    tbb::task_scheduler_init scheduler(num_threads, stack_size);


    end_2_end();


    return 0;
}