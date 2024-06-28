#pragma once

#include <cstdio>
#include <stdlib.h>
#include <cassert>
#include <string>
#include <vector>

#include "utils/file/file_path.h"

namespace Mesh_Loader {

    enum string_code {
        OBJ,
        OFF,
        VTK,
        VTU,
        MESH,
        F3GRID,
        UNKNOW
    };

    struct Cell {
        int numberOfPoints = 3; //3表示三角形
        int *pointList;
    };

    struct Edge {
        int numberOfPoints = 2; //2表示线段
        int *pointList;
    };

    template<typename T>
    struct DataArray {
        //std::string type_name;
        std::vector<T> content;
    };


    struct FileData {
        int numberOfPoints = 0;
        double *pointList = nullptr;

        int numberOfCell = 0;
        Cell *cellList = nullptr;

        int numberOfEdge = 0;
        Edge *edgeList = nullptr;

        std::map<std::string, DataArray<std::string>> cellDataString;
        std::map<std::string, DataArray<double>> cellDataDouble;
        std::map<std::string, DataArray<float>> cellDataFloat;
        std::map<std::string, DataArray<int>> cellDataInt;
        std::map<std::string, DataArray<unsigned long long>> cellDataUInt64;
        std::map<std::string, DataArray<unsigned int>> cellDataUInt;
        std::map<std::string, DataArray<bool>> cellDataBool;

        std::map<std::string, DataArray<std::string>> pointDataString;
        std::map<std::string, DataArray<double>> pointDataDouble;
        std::map<std::string, DataArray<float>> pointDataFloat;
        std::map<std::string, DataArray<int>> pointDataInt;
        std::map<std::string, DataArray<unsigned int>> pointDataUInt;
        std::map<std::string, DataArray<unsigned long long>> pointDataUInt64;
        std::map<std::string, DataArray<bool>> pointDataBool;

        static double *malloc_point(int number_point) {
            return (double *) malloc(sizeof(double) * number_point * 3);
        }

        static Mesh_Loader::Cell *malloc_cell(int number_cell) {
            return (Mesh_Loader::Cell *) malloc(sizeof(Mesh_Loader::Cell) * number_cell);
        }

        static Mesh_Loader::Edge *malloc_edge(int number_edge) {
            return (Mesh_Loader::Edge *) malloc(sizeof(Mesh_Loader::Edge) * number_edge);
        }

        FileData() {
            pointList = nullptr;
            cellList = nullptr;
            edgeList = nullptr;
        }

        void set_point_number(int number_point) {
            if (pointList != nullptr)
                assert(false);
            numberOfPoints = number_point;
            pointList = malloc_point(number_point);
        }

        void set_cell_number(int number_cell) {
            if (cellList != nullptr)
                assert(false);
            numberOfCell = number_cell;
            cellList = malloc_cell(number_cell);
        }

        void set_edge_number(int number_edge) {
            if (edgeList != nullptr)
                assert(false);
            numberOfEdge = number_edge;
            edgeList = malloc_edge(number_edge);
        }

        ~FileData() {
            if (numberOfPoints > 0 && pointList != nullptr) {
                free(pointList);
                pointList = nullptr;
            }
            if (numberOfCell > 0 && cellList != nullptr) {

                for (int i = 0; i < numberOfCell; i++) {
                    auto &cell = cellList[i];
                    delete (cell.pointList);
                }

                free(cellList);
                cellList = nullptr;
            }
            if (numberOfEdge > 0 && edgeList != nullptr) {

                for (int i = 0; i < numberOfEdge; i++) {
                    auto &edge = edgeList[i];
                    delete (edge.pointList);
                }

                free(edgeList);
                edgeList = nullptr;
            }
        }

    };

    bool load_by_extension(const char *in_file_path, FileData &data);

    bool load_f3grid(const char *in_file_path, FileData &data);

    //bool load_vtu(const char *in_file_path, FileData &data);

    bool load_mesh(const char *in_file_path, FileData &data);

    bool load_obj(const char *in_file_path, FileData &data);

    //bool load_vtk(const char *in_file_path, FileData &data);

 /*   bool save_vtu(const char *out_file_path, const FileData &data);*/

    bool save_obj(const char *out_file_path, const FileData &data);
}






