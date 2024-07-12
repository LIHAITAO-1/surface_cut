//
// Created by xmyci on 20/02/2024.
//

#include "tbb/tbb.h"
#include "CLI11.hpp"

#include "basic/typedef.h"
#include "pipeline/run pipeline/run_pipeline.h"
#include "geogram/basic/common.h"
#include "geogram/basic/command_line_args.h"
#include "basic/typedef.h"

#include <csignal>

std::vector<base_type::Triangle_Soup_Mesh> input_mesh_array;
base_type::Triangle_Soup_Mesh input_mesh_domain;

#ifdef BUILD_LIB

#include "main.h"

int dll_main(std::string path) {
#else


int main(int argc, char **argv) {

#endif


//    GEO::initialize();
//    GEO::CmdLine::import_arg_group("standard");
//    GEO::CmdLine::import_arg_group("algo");
//    GEO::CmdLine::import_arg_group("co3ne");
//    GEO::CmdLine::import_arg_group("pre");
//    GEO::CmdLine::import_arg_group("post");
//    GEO::CmdLine::import_arg_group("remesh");
////    GEO::CmdLine::set_arg("log:quiet", true);
//    std::string file_path = "default";
//
//#ifdef BUILD_LIB
//    file_path = path;
//#else
//    CLI::App app{"App description"};
//    argv = app.ensure_utf8(argv);
//    app.add_option("-f,--file", file_path, "A help string");
//    CLI11_PARSE(app, argc, argv);
//#endif
//
//    if (strcmp(file_path.c_str(), "default") == 0) {
//        logger().warn("input config path can is null, using current dir!");
//        file_path = "./default_config.json";
//    }
//
//
//
//    const size_t MB = 1024 * 1024;
//    const size_t stack_size = 64 * MB;
//
//    int num_threads = std::max(1u, std::thread::hardware_concurrency());
//
//    logger().info("TBB threads: {}", num_threads);
//
//    tbb::task_scheduler_init scheduler(num_threads, stack_size);


    std::string Path = "D:/xmy/model/";
    std::string MeshFile =  "8-2.obj"; //"8-2.obj";//"cube2.obj";
    std::string CurveFile = "curve7.obj";//"fm38.obj";//"curve6.obj";

	base_type::Triangle_Soup_Mesh meshCube;
	base_type::Triangle_Soup_Mesh meshCurve;
	base_type::Triangle_Soup_Mesh meshResult;

	meshCube.load_from_file(Path + MeshFile);
	meshCurve.load_from_file(Path + CurveFile);
    int index1 = 1;
    int index2 = 0;

    surface_cut(Path, meshCube, meshCurve, meshResult, index1, index2);

    return 0;
}