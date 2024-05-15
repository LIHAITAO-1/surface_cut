//
// Created by xmyci on 14/03/2024.
//
#include "kernel_warpper.h"
#include "kernel.hpp"
#include "utils/string/string_utils.h"


void print_with_line_num(std::string input) {
    auto r = string_split(input, "\n");
    std::cout << "-----------------------" << std::endl;
    int index = 1;
    for (auto &ele: r) {
        std::cout << index++ << " | " << ele << std::endl;
    }
    std::cout << "-----------------------" << std::endl;
}


std::string kernel_warpper::get_kernel_code(bool debug) {
    using namespace std;
    string r = kernel_code;
    //r = replace(r, " ", "\n"); // replace all spaces by new lines
    r = replace(r, "#ifdef\n", "#ifdef "); // except for the arguments after some preprocessor options that need to be in the same line
    r = replace(r, "#ifndef\n", "#ifndef ");
    r = replace(r, "#define\n", "#define "); // #define with two arguments will not work
    r = replace(r, "#if\n", "#if "); // don't leave any spaces in arguments
    r = replace(r, "#elif\n", "#elif "); // don't leave any spaces in arguments
    r = replace(r, "#pragma\n", "#pragma ");
    if (debug)
        print_with_line_num("\n" + r);

    return "\n" + r;
}