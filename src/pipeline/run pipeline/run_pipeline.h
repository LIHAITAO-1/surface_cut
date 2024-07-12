#ifndef TETGEO_SURFACE_CUT_H
#define TETGEO_SURFACE_CUT_H

#include "basic/typedef.h"

void surface_cut(std::string Path, base_type::Triangle_Soup_Mesh& meshCube, base_type::Triangle_Soup_Mesh& meshCurve, base_type::Triangle_Soup_Mesh& meshResult, int index1, int index2);

void surface_cut(base_type::Triangle_Soup_Mesh& meshCube, base_type::Triangle_Soup_Mesh& meshCurve, base_type::Triangle_Soup_Mesh& meshResult, int index1, int index2);


#endif //TETGEO_END_2_END_H
