/*
 * Copyright 2013 Computer Graphics Group, RWTH Aachen University
 * Author: Hans-Christian Ebke <ebke@cs.rwth-aachen.de>
 *
 * This file is part of QEx.
 *
 * QEx is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * QEx is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with QEx.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <qex.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#define USE_POSIX
#include <libgen.h>
#include <cstring>
#endif

using namespace QEx;

void printUsage(const std::string &cmd) {
#ifdef USE_POSIX
    char *cmdCpy = new char[cmd.size() + 1];
    strcpy(cmdCpy, cmd.data());
    std::string basename = ::basename(cmdCpy);
    delete[] cmdCpy;
#endif
    std::cout << "Usage: "
            << basename
            << " <infile> <outfile>" << std::endl
            << std::endl
            << "Reads the mesh with face-based UVs from <infile> which must be an" << std::endl
            << "OBJ file extracts the quad mesh from the UVs and stores the resulting" << std::endl
            << "mesh into <outfile> in OBJ format." << std::endl;
}

bool infileGood(const std::string &filePath) {
    std::ifstream is(filePath.c_str());
    return is.good();
}

bool outfileGood(const std::string &filePath) {
    std::ofstream is(filePath.c_str());
    return is.good();
}

int main(int argc, const char *argv[]) {
    if (argc != 3) {
        printUsage(argv[0]);
        return 1;
    }

    if (!infileGood(argv[1])) {
        std::cout << "Can't read input file." << std::endl;
        return 2;
    }

    if (!outfileGood(argv[2])) {
        std::cout << "Can't write output file." << std::endl;
        return 3;
    }

    /*
     * Read input mesh.
     */
    TriMesh inputMesh;
    inputMesh.request_halfedge_texcoords2D();
    OpenMesh::IO::Options readOpts(OpenMesh::IO::Options::FaceTexCoord);
    OpenMesh::IO::read_mesh(inputMesh, argv[1], readOpts);

    /*
     * Convert texture coordinates into separate uv vector.
     */
    std::vector<OpenMesh::Vec2d> uvVector;
    uvVector.reserve(inputMesh.n_halfedges());
    for (TriMesh::HalfedgeIter he_it = inputMesh.halfedges_begin(), he_end = inputMesh.halfedges_end();
            he_it != he_end; ++he_it) {
        const OpenMesh::Vec2f &uv_f = inputMesh.texcoord2D(*he_it);
        OpenMesh::Vec2d uv(uv_f[0], uv_f[1]);
        uvVector.push_back(uv);
    }

    QuadMesh out;
    extractQuadMeshOMT(&inputMesh, &uvVector, 0, &out);

    /*
     * Write output mesh.
     */
    OpenMesh::IO::write_mesh(out, argv[2], OpenMesh::IO::Options::Default, 12);

    return 0;
}
