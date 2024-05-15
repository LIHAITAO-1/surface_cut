
#include <string.h>
#include <vector>
#include <set>
#include <map>
#include <bitset>
#include <array>

#include <vtkCellData.h>
#include <vtkCellTypes.h>
#include <vtkDataSet.h>
#include <vtkFieldData.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkRectilinearGrid.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLCompositeDataReader.h>
#include <vtkXMLImageDataReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLReader.h>
#include <vtkXMLRectilinearGridReader.h>
#include <vtkXMLStructuredGridReader.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtksys/SystemTools.hxx>
#include <vtkTetra.h>
#include <vtkTriangle.h>
#include <vtkStringArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedIntArray.h>
#include "vtkUnsignedLongLongArray.h"

#include "utils/file/file_path.h"
#include "mesh_loader.h"
#include "utils/log/logger.h"
#include "utils/string/string_utils.h"
#include "basic/macro.h"

namespace Mesh_Loader {

    template<class TReader>
    vtkDataSet *ReadAnXMLFile(const char *fileName) {
        vtkSmartPointer<TReader> reader = vtkSmartPointer<TReader>::New();
        reader->SetFileName(fileName);
        reader->Update();
        reader->GetOutput()->Register(reader);
        return vtkDataSet::SafeDownCast(reader->GetOutput());
    }

    char *read_line(char *string, FILE *infile, int *linenumber, bool skip_sapce_and_tab = true) {
        char *result;

        // Search for a non-empty line.
        do {
            result = fgets(string, 2048, infile);
            if (linenumber) (*linenumber)++;
            if (result == (char *) NULL) {
                return (char *) NULL;
            }
            // Skip white spaces.
            if (skip_sapce_and_tab)
                while ((*result == ' ') || (*result == '\t')) result++;
            // If it's end of line, read another line and try again.
        } while ((*result == '\0') || (*result == '\r') || (*result == '\n'));
        return result;
    }

    bool load_vtu(const char *in_file_path, FileData &data) {
        //vtkDataSet *dataSet = ReadAnXMLFile<vtkXMLUnstructuredGridReader>(in_file_path);
        vtkSmartPointer<vtkXMLUnstructuredGridReader> reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
        reader->SetFileName(in_file_path);
        reader->Update();
        reader->GetOutput()->Register(reader);
        vtkUnstructuredGrid *g = reader->GetOutput();

        int numberofcell = g->GetNumberOfCells();
        int numberofpoint = g->GetNumberOfPoints();

        vtkCellTypes *p_vtkCellTypes = vtkCellTypes::New();
        g->GetCellTypes(p_vtkCellTypes);
        int numberofcelltypes = p_vtkCellTypes->GetNumberOfTypes();
        auto p_CellTypesArray = p_vtkCellTypes->GetCellTypesArray();

        //Point
        data.set_point_number(numberofpoint);


        data.pointList = new double[numberofpoint * 3];
        for (int i = 0; i < numberofpoint; i++) {
            g->GetPoint(i, &data.pointList[i * 3]);
        }

        //Cell
        data.set_cell_number(numberofcell);

        for (int i = 0; i < numberofcell; i++) {
            auto &cell = data.cellList[i];
            if (g->GetCellType(i) == 10) {
                cell.numberOfPoints = 4;
                cell.pointList = new int[4];
                vtkIdType npts;
                vtkIdType const *pts;
                g->GetCellPoints(i, npts, pts);
                assert(npts == 4);
                cell.pointList[0] = pts[0];
                cell.pointList[1] = pts[1];
                cell.pointList[2] = pts[2];
                cell.pointList[3] = pts[3];
            } else if (g->GetCellType(i) == 5) {
                //triangle
                //vtkCellTypes::GetClassNameFromTypeId
                cell.numberOfPoints = 3;
                cell.pointList = new int[3];
                vtkIdType npts;
                vtkIdType const *pts;
                g->GetCellPoints(i, npts, pts);
                assert(npts == 3);
                cell.pointList[0] = pts[0];
                cell.pointList[1] = pts[1];
                cell.pointList[2] = pts[2];
            } else {
                ASSERT_MSG(false, "ERROR: unsupport vtu cell type, currently only support tetrahedra and triangle!");
            }
        }

        //Cell Data
        vtkCellData *g_celldata = g->GetCellData();
        int array_number = g_celldata->GetNumberOfArrays();
        //data.cellDataString.resize(array_number);
        for (int i = 0; i < array_number; i++) {
            vtkAbstractArray *array = g_celldata->GetAbstractArray(i);
            //g_celldata->GetAbstractArray(0)->GetDataType () -> 13
            //g_celldata->GetAbstractArray(0)->GetNumberOfValues() ->1558755
            switch (array->GetDataType()) {
                case VTK_DOUBLE: {
                    data.cellDataDouble[array->GetName()];
                    auto &celldata_array = data.cellDataDouble[array->GetName()].content;
                    celldata_array.resize(array->GetNumberOfValues());
                    for (int j = 0; j < array->GetNumberOfValues(); j++) {
                        auto *p_double = (double *) array->GetVoidPointer(j);
                        celldata_array[j] = *p_double;
                    }
                    break;
                }
                case VTK_FLOAT: {
                    data.cellDataFloat[array->GetName()];
                    auto &celldata_array = data.cellDataFloat[array->GetName()].content;
                    celldata_array.resize(array->GetNumberOfValues());
                    for (int j = 0; j < array->GetNumberOfValues(); j++) {
                        auto *p_double = (float *) array->GetVoidPointer(j);
                        celldata_array[j] = *p_double;
                    }
                    break;
                }
                case VTK_INT: {
                    data.cellDataInt[array->GetName()];
                    auto &celldata_array = data.cellDataInt[array->GetName()].content;
                    celldata_array.resize(array->GetNumberOfValues());
                    for (int j = 0; j < array->GetNumberOfValues(); j++) {
                        auto *p_double = (int *) array->GetVoidPointer(j);
                        celldata_array[j] = *p_double;
                    }
                    break;
                }
                case VTK_UNSIGNED_INT: {
                    data.cellDataUInt[array->GetName()];
                    auto &celldata_array = data.cellDataUInt[array->GetName()].content;
                    celldata_array.resize(array->GetNumberOfValues());
                    for (int j = 0; j < array->GetNumberOfValues(); j++) {
                        auto *p_double = (unsigned int *) array->GetVoidPointer(j);
                        celldata_array[j] = *p_double;
                    }
                    break;
                }
                case VTK_STRING: {
                    data.cellDataString[array->GetName()];
                    std::vector<std::string> &string_array = data.cellDataString[array->GetName()].content;
                    string_array.resize(array->GetNumberOfValues());
                    for (int j = 0; j < array->GetNumberOfValues(); j++) {
                        vtkStdString *vtkstring = (vtkStdString *) array->GetVoidPointer(j);
                        string_array[j] = *vtkstring;
                    }
                    break;
                }
                default:
                    ASSERT_MSG(false, "RROR: unsupport vtu celldata type!")
            }


        }
        return true;
    }

    bool save_vtu(const char *out_file_path, const FileData &data) {
        vtkNew<vtkPoints> points;
        vtkNew<vtkTetra> tetra;
        vtkNew<vtkTriangle> triangle;
        vtkNew<vtkCellArray> cellArray;
        vtkNew<vtkUnsignedCharArray> celltypes;

        celltypes->SetNumberOfComponents(1);
        celltypes->SetNumberOfValues(data.numberOfCell);

        for (int i = 0; i < data.numberOfPoints; i++) {
            points->InsertNextPoint(data.pointList[i * 3], data.pointList[i * 3 + 1], data.pointList[i * 3 + 2]);
        }

        for (int i = 0; i < data.numberOfCell; i++) {
            Cell &cell = data.cellList[i];
            if (cell.numberOfPoints == 4) {
                tetra->GetPointIds()->SetId(0, cell.pointList[0]);
                tetra->GetPointIds()->SetId(1, cell.pointList[1]);
                tetra->GetPointIds()->SetId(2, cell.pointList[2]);
                tetra->GetPointIds()->SetId(3, cell.pointList[3]);
                cellArray->InsertNextCell(tetra);
                celltypes->SetValue(i, VTK_TETRA);
            } else if (cell.numberOfPoints == 3) {
                triangle->GetPointIds()->SetId(0, cell.pointList[0]);
                triangle->GetPointIds()->SetId(1, cell.pointList[1]);
                triangle->GetPointIds()->SetId(2, cell.pointList[2]);
                cellArray->InsertNextCell(triangle);
                celltypes->SetValue(i, VTK_TRIANGLE);
            } else {
                ASSERT_MSG(false, "ERROR: unsupport input");
            }
        }

        vtkNew<vtkUnstructuredGrid> unstructuredGrid;
        unstructuredGrid->SetPoints(points);
        unstructuredGrid->SetCells(celltypes, cellArray);

        {
            //set celldata
            vtkCellData *g_celldata = unstructuredGrid->GetCellData();

            for (auto iter = data.cellDataString.begin(); iter != data.cellDataString.end(); iter++) {
                vtkStringArray *array = vtkStringArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j].c_str());
                }
                g_celldata->AddArray(array);
            }

            for (auto iter = data.cellDataDouble.begin(); iter != data.cellDataDouble.end(); iter++) {
                vtkDoubleArray *array = vtkDoubleArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_celldata->AddArray(array);
            }

            for (auto iter = data.cellDataFloat.begin(); iter != data.cellDataFloat.end(); iter++) {
                vtkFloatArray *array = vtkFloatArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_celldata->AddArray(array);
            }

            for (auto iter = data.cellDataInt.begin(); iter != data.cellDataInt.end(); iter++) {
                vtkIntArray *array = vtkIntArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_celldata->AddArray(array);
            }

            for (auto iter = data.cellDataUInt.begin(); iter != data.cellDataUInt.end(); iter++) {
                vtkUnsignedIntArray *array = vtkUnsignedIntArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_celldata->AddArray(array);
            }

            for (auto iter = data.cellDataUInt64.begin(); iter != data.cellDataUInt64.end(); iter++) {
                vtkUnsignedLongLongArray *array = vtkUnsignedLongLongArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_celldata->AddArray(array);
            }

            for (auto iter = data.cellDataBool.begin(); iter != data.cellDataBool.end(); iter++) {
                vtkIntArray *array = vtkIntArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_celldata->AddArray(array);
            }

        }

        {
            //set pointdata
            vtkPointData *g_pointdata = unstructuredGrid->GetPointData();

            for (auto iter = data.pointDataString.begin(); iter != data.pointDataString.end(); iter++) {
                vtkStringArray *array = vtkStringArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j].c_str());
                }
                g_pointdata->AddArray(array);
            }

            for (auto iter = data.pointDataDouble.begin(); iter != data.pointDataDouble.end(); iter++) {
                vtkDoubleArray *array = vtkDoubleArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_pointdata->AddArray(array);
            }

            for (auto iter = data.pointDataFloat.begin(); iter != data.pointDataFloat.end(); iter++) {
                vtkFloatArray *array = vtkFloatArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_pointdata->AddArray(array);
            }

            for (auto iter = data.pointDataInt.begin(); iter != data.pointDataInt.end(); iter++) {
                vtkIntArray *array = vtkIntArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_pointdata->AddArray(array);
            }

            for (auto iter = data.pointDataUInt.begin(); iter != data.pointDataUInt.end(); iter++) {
                vtkUnsignedIntArray *array = vtkUnsignedIntArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_pointdata->AddArray(array);
            }

            for (auto iter = data.pointDataUInt64.begin(); iter != data.pointDataUInt64.end(); iter++) {
                vtkUnsignedLongLongArray *array = vtkUnsignedLongLongArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_pointdata->AddArray(array);
            }


            for (auto iter = data.pointDataBool.begin(); iter != data.pointDataBool.end(); iter++) {
                vtkIntArray *array = vtkIntArray::New();

                array->SetName(iter->first.c_str());
                array->SetNumberOfValues(iter->second.content.size());
                for (int j = 0; j < iter->second.content.size(); j++) {
                    array->SetValue(j, iter->second.content[j]);
                }
                g_pointdata->AddArray(array);
            }
        }

        // Write file.
        vtkNew<vtkXMLUnstructuredGridWriter> writer;
        writer->SetFileName(out_file_path);
        writer->SetInputData(unstructuredGrid);
        writer->SetDataModeToAscii();
        writer->Write();
        return true;
    }

    bool load_by_extension(const char *in_file_path, FileData &data) {
        auto extension = get_file_extension(in_file_path);

        auto hash_hit = [](std::string inString) {
            if (inString == "obj") return OBJ;
            if (inString == "vtk") return VTK;
            if (inString == "mesh") return MESH;
            if (inString == "vtu") return VTU;
            return UNKNOW;
        };

        switch (hash_hit(extension)) {
            case OBJ:
                return load_obj(in_file_path, data);

            case VTK:
                return load_vtk(in_file_path, data);

            case VTU:
                return load_vtu(in_file_path, data);

            default:
                assert(false);

        }

        return false;
    }

    bool save_mesh(const char *out_file_path, const FileData &data) {

        FILE *fout = fopen(out_file_path, "w");

        if (fout == (FILE *) NULL) {
            return false;
        }

        fprintf(fout, "MeshVersionFormatted 1\n");
        fprintf(fout, "Dimension 3\n");
        fprintf(fout, "Vertices\n");
        fprintf(fout, "%d\n", data.numberOfPoints);
        for (int i = 0; i < data.numberOfPoints; i++) {
            fprintf(fout, "%.16g %.16g %.16g %d\n", data.pointList[i * 3], data.pointList[i * 3 + 1], data.pointList[i * 3 + 2], 0);
        }

        fprintf(fout, "Tetrahedra\n");
        fprintf(fout, "%d\n", data.numberOfCell);

        for (int i = 0; i < data.numberOfCell; i++) {
            auto &face = data.cellList[i];
            assert(face.numberOfPoints == 4);
            fprintf(fout, "%d %d %d %d\n", face.pointList[0] + 1, face.pointList[1] + 1, face.pointList[2] + 1, face.pointList[3] + 1);
        }
        fclose(fout);
        return true;
    }

    bool load_mesh(const char *in_file_path, FileData &data) {
        FILE *fp = fopen(in_file_path, "r");
        if (fp == (FILE *) NULL) {
            //printf("File I/O Error:  Cannot create file %s.\n", vtk_file_path);
            return false;
        }

        char buffer[2048];
        char *bufferp;
        int line_count = 0;

        int nverts = 0, iverts = 0;
        int ntetrahedras = 0, itetrahedras = 0, itetrahedrasattr = 0;
        bool read_vtx_begin = false;
        bool read_tet_begin = false;

        while ((bufferp = read_line(buffer, fp, &line_count)) != NULL) {
            if (read_vtx_begin && nverts > iverts) {
                data.pointList[iverts * 3] = (double) strtod(bufferp, &bufferp);
                data.pointList[iverts * 3 + 1] = (double) strtod(bufferp, &bufferp);
                data.pointList[iverts * 3 + 2] = (double) strtod(bufferp, &bufferp);
                iverts++;
                continue;
            } else if (read_tet_begin && ntetrahedras > itetrahedras) {
                int p0, p1, p2, p3;
                int type;
                sscanf(bufferp, "%d %d %d %d %d",
                       &p0,
                       &p1,
                       &p2,
                       &p3,
                       &type
                );

                data.cellList[itetrahedras].pointList = new int[4];
                data.cellList[itetrahedras].numberOfPoints = 4;

                auto Cell_point = &data.cellList[itetrahedras].pointList[0];
                Cell_point[0] = p0 - 1;
                Cell_point[1] = p1 - 1;
                Cell_point[2] = p2 - 1;
                Cell_point[3] = p3 - 1;

                //data.cellList[itetrahedras].cellattr = new double[1];
                //data.cellList[itetrahedras].cellattr[0] = type;
                itetrahedras++;
                continue;
            }

            char s[500];
            sscanf(bufferp, "%s", &s);
            if (strcmp(s, "Vertices") == 0) {
                read_vtx_begin = true;
                read_line(buffer, fp, &line_count);
                sscanf(bufferp, "%d", &nverts);

                data.set_point_number(nverts);
                //data.cellattrname.push_back("type");
            } else if (strcmp(s, "Tetrahedra") == 0) {
                read_tet_begin = true;
                read_line(buffer, fp, &line_count);
                sscanf(bufferp, "%d", &ntetrahedras);
                data.set_cell_number(ntetrahedras);
            }
        }
        fclose(fp);
        return true;
    }

    bool load_obj(const char *in_file_path, FileData &data) {
        FILE *fp = fopen(in_file_path, "r");
        if (fp == (FILE *) NULL) {
            //printf("File I/O Error:  Cannot create file %s.\n", vtk_file_path);
            return false;
        }

        char buffer[2048];
        char *bufferp;
        int line_count = 0;

        int nverts = 0, iverts = 0;
        int ncells = 0, icells = 0;


        while ((bufferp = read_line(buffer, fp, &line_count)) != NULL) {
            char string[150];
            sscanf(bufferp, "%s", &string);
            if (strcmp(string, "v") == 0) {
                nverts++;
            } else if (strcmp(string, "f") == 0) {
                ncells++;
            }
        }

        line_count = 0;
        fclose(fp);
        fp = fopen(in_file_path, "r");

        data.set_point_number(nverts);
        data.set_cell_number(ncells);

        while ((bufferp = read_line(buffer, fp, &line_count)) != NULL) {
            char string[150];
            char x[150], y[150], z[150];
            char *ptr;
            sscanf(bufferp, "%s", &string);
            if (strcmp(string, "v") == 0) {
                sscanf(bufferp, "%*s %s %s %s", &x, &y, &z);
                double d1 = (double) strtod(x, &ptr);
                double d2 = (double) strtod(y, &ptr);
                double d3 = (double) strtod(z, &ptr);
                data.pointList[iverts * 3] = (double) strtod(x, &ptr);
                data.pointList[iverts * 3 + 1] = (double) strtod(y, &ptr);;
                data.pointList[iverts * 3 + 2] = (double) strtod(z, &ptr);;
                iverts++;
            } else if (strcmp(string, "f") == 0) {
                char x[35], y[35], z[35];
                int p0, p1, p2, p3;
                sscanf(bufferp, "%*s %s %s %s",
                       &x,
                       &y,
                       &z
                );

                auto string_split = [](std::string s, std::string delimiter) {
                    std::vector<std::string> res;
                    int index = 0;

                    while (true) {
                        int find_index = s.find(delimiter, index);
                        if (find_index == -1) {
                            res.push_back(s.substr(index, s.size() - index));
                            break;
                        }
                        res.push_back(s.substr(index, find_index - index));
                        index = find_index + 1;
                    }
                    return res;
                };

                data.cellList[icells].pointList = new int[3];
                data.cellList[icells].numberOfPoints = 3;

                auto Cell_point = &data.cellList[icells].pointList[0];
                Cell_point[0] = (int) strtod(string_split(x, "/")[0].c_str(), &ptr) - 1;
                Cell_point[1] = (int) strtod(string_split(y, "/")[0].c_str(), &ptr) - 1;
                Cell_point[2] = (int) strtod(string_split(z, "/")[0].c_str(), &ptr) - 1;

                icells++;
            }
        }

        fclose(fp);
        return true;
    }

    bool load_vtk(const char *in_file_path, FileData &data) {
        return false;
    }

    bool save_obj(const char *out_file_path, const FileData &data) {
        FILE *fout = fopen(out_file_path, "w");

        if (fout == (FILE *) NULL) {
            //printf("File I/O Error:  Cannot create file %s.\n", vtk_file_path);
            return false;
        }

        for (int i = 0; i < data.numberOfPoints; i++) {
            fprintf(fout, "%c %.16g %.16g %.16g\n", 'v', data.pointList[i * 3], data.pointList[i * 3 + 1],
                    data.pointList[i * 3 + 2]);
        }
        for (int i = 0; i < data.numberOfCell; i++) {
            auto &face = data.cellList[i];
            assert(face.numberOfPoints == 3);

            fprintf(fout, "%c %d %d %d\n", 'f', face.pointList[0] + 1, face.pointList[1] + 1, face.pointList[2] + 1);
        }
        fclose(fout);
        return true;
    }

}