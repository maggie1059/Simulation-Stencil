#include "simulation.h"

#include <iostream>

#include "graphics/MeshLoader.h"

using namespace Eigen;

Simulation::Simulation()
{
}

void Simulation::init()
{
    // STUDENTS: This code loads up the tetrahedral mesh in 'example-meshes/single-tet.mesh'
    //    (note: your working directory must be set to the root directory of the starter code
    //    repo for this file to load correctly). You'll probably want to instead have this code
    //    load up a tet mesh based on e.g. a file path specified with a command line argument.
    std::vector<Vector3f> vertices;
    std::vector<Vector4i> tets;
    if(MeshLoader::loadTetMesh("example-meshes/single-tet.mesh", vertices, tets)) {
        // STUDENTS: This code computes the surface mesh of the loaded tet mesh, i.e. the faces
        //    of tetrahedra which are on the exterior surface of the object. Right now, this is
        //    hard-coded for the single-tet mesh. You'll need to implement surface mesh extraction
        //    for arbitrary tet meshes. Think about how you can identify which tetrahedron faces
        //    are surface faces...
//        for (auto i : tets){
//            std::sort(i.data(), i.data()+i.size(), std::less<int>());
//            std::string result;
//            result = std::to_string(i[0]) + std::to_string(i[1]) + std::to_string(i[2]) + std::to_string(i[3]);
//            if (m_surface.find(result) == m_surface.end()){
//                m_surface[result] = 1;
//            } else {
//                m_surface[result]++;
//            }
//        }
        std::unordered_map<std::string, int> fourths;
        for (auto i : tets){
            Vector3i face1(i[1], i[0], i[2]);
            Vector3i face2(i[2], i[0], i[3]);
            Vector3i face3(i[3], i[1], i[2]);
            Vector3i face4(i[3], i[0], i[1]);
            std::sort(face1.data(), face1.data()+face1.size(), std::less<int>());
            std::sort(face2.data(), face2.data()+face2.size(), std::less<int>());
            std::sort(face3.data(), face3.data()+face3.size(), std::less<int>());
            std::sort(face4.data(), face4.data()+face4.size(), std::less<int>());

            std::string result1;
            std::string result2;
            std::string result3;
            std::string result4;
            result1 = std::to_string(face1[0]) + std::to_string(face1[1]) + std::to_string(face1[2]);
            result2 = std::to_string(face2[0]) + std::to_string(face2[1]) + std::to_string(face2[2]);
            result3 = std::to_string(face3[0]) + std::to_string(face3[1]) + std::to_string(face3[2]);
            result4 = std::to_string(face4[0]) + std::to_string(face4[1]) + std::to_string(face4[2]);
            if (m_surface.find(result1) == m_surface.end()){
                m_surface[result1] = 1;
                fourths[result1] = i[3];
            } else {
                m_surface[result1]++;
            }
            if (m_surface.find(result2) == m_surface.end()){
                m_surface[result2] = 1;
                fourths[result1] = i[1];
            } else {
                m_surface[result2]++;
            }
            if (m_surface.find(result3) == m_surface.end()){
                m_surface[result3] = 1;
                fourths[result1] = i[0];
            } else {
                m_surface[result3]++;
            }
            if (m_surface.find(result4) == m_surface.end()){
                m_surface[result4] = 1;
                fourths[result1] = i[2];
            } else {
                m_surface[result4]++;
            }
        }

        std::vector<Vector3i> faces;
        for (auto it = m_surface.begin(); it != m_surface.end(); ++it){
            std::string tet = it->first;
            if (m_surface[it->second] > 1){
                continue;
            }
            int zero = tet[0].toInt();
            int one = tet[1].toInt();
            int two = tet[2].toInt();
            int three = tet[3].toInt();
            Vector3i face1(one, zero, two);
            Vector3i face2(two, zero, three);
            Vector3i face3(three, one, two);
            Vector3i face4(three, zero, one);
            turnClockwise(face1);
            turnClockwise(face2);
            turnClockwise(face3);
            turnClockwise(face4);
            faces.emplace_back(face1);
            faces.emplace_back(face2);
            faces.emplace_back(face3);
            faces.emplace_back(face4);
        }
//        faces.emplace_back(1, 0, 2);
//        faces.emplace_back(2, 0, 3);
//        faces.emplace_back(3, 1, 2);
//        faces.emplace_back(3, 0, 1);
        m_shape.init(vertices, faces, tets);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 2, 0)));

    initGround();
}

void Simulation::update(float seconds)
{
    // STUDENTS: This method should contain all the time-stepping logic for your simulation.
    //   Specifically, the code you write here should compute new, updated vertex positions for your
    //   simulation mesh, and it should then call m_shape.setVertices to update the display with those
    //   newly-updated vertices.

    // STUDENTS: As currently written, the program will just continually compute simulation timesteps as long
    //    as the program is running (see View::tick in view.cpp) . You might want to e.g. add a hotkey for pausing
    //    the simulation, and perhaps start the simulation out in a paused state.

    //element-> stress/strain
    //node-> forces (accumulated from adjacent faces)
}

void Simulation::turnClockwise(Vector3i &face){

}

void Simulation::draw(Shader *shader)
{
    m_shape.draw(shader);
    m_ground.draw(shader);
}

void Simulation::toggleWire()
{
    m_shape.toggleWireframe();
}

void Simulation::initGround()
{
    std::vector<Vector3f> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, 0, -5);
    groundVerts.emplace_back(-5, 0, 5);
    groundVerts.emplace_back(5, 0, 5);
    groundVerts.emplace_back(5, 0, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    m_ground.init(groundVerts, groundFaces);
}
