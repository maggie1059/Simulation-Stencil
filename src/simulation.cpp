#include "simulation.h"
#include <iostream>
#include "graphics/MeshLoader.h"
#define GRAVITY -1.f
#define GRAVITY_ON true

using namespace Eigen;

Simulation::Simulation()
{
    m_nodes.clear();
    m_elements.clear();
}

void Simulation::init()
{
    // STUDENTS: This code loads up the tetrahedral mesh in 'example-meshes/single-tet.mesh'
    //    (note: your working directory must be set to the root directory of the starter code
    //    repo for this file to load correctly). You'll probably want to instead have this code
    //    load up a tet mesh based on e.g. a file path specified with a command line argument.
    std::vector<Vector3f> vertices;
    std::vector<Vector4i> tets;
    if(MeshLoader::loadTetMesh("example-meshes/sphere.mesh", vertices, tets)) {
        // STUDENTS: This code computes the surface mesh of the loaded tet mesh, i.e. the faces
        //    of tetrahedra which are on the exterior surface of the object. Right now, this is
        //    hard-coded for the single-tet mesh. You'll need to implement surface mesh extraction
        //    for arbitrary tet meshes. Think about how you can identify which tetrahedron faces
        //    are surface faces...

        for (auto v : vertices){
            shared_ptr<Node> n(new Node(v));
            m_nodes.push_back(n);
        }

//        std::unordered_map<int, int> nodeappears;

        for (auto t : tets){
//            if (nodeappears.find(t[0]) == nodeappears.end()){
//                nodeappears[t[0]] = 1;
//            } else {
//                nodeappears[t[0]]++;
//            }
//            if (nodeappears.find(t[1]) == nodeappears.end()){
//                nodeappears[t[1]] = 1;
//            } else {
//                nodeappears[t[1]]++;
//            }
//            if (nodeappears.find(t[2]) == nodeappears.end()){
//                nodeappears[t[2]] = 1;
//            } else {
//                nodeappears[t[2]]++;
//            }
//            if (nodeappears.find(t[3]) == nodeappears.end()){
//                nodeappears[t[3]] = 1;
//            } else {
//                nodeappears[t[3]]++;
//            }
            shared_ptr<Element> e(new Element(m_nodes[t[0]], m_nodes[t[1]], m_nodes[t[2]], m_nodes[t[3]]));
//            e->setMasses();
            m_elements.push_back(e);
        }

//        for (int i = 0; i < vertices.size(); i++){
//            shared_ptr<Node> v = m_nodes[i];
//            float mass = v->m_mass/nodeappears[i];
//            std::cout << mass << std::endl;
//        }

        for (auto i : tets){
            Vector3i face1(i[1], i[0], i[2]);
            Vector3i face2(i[2], i[0], i[3]);
            Vector3i face3(i[3], i[1], i[2]);
            Vector3i face4(i[3], i[0], i[1]);
            std::sort(face1.data(), face1.data()+face1.size(), std::less<int>());
            std::sort(face2.data(), face2.data()+face2.size(), std::less<int>());
            std::sort(face3.data(), face3.data()+face3.size(), std::less<int>());
            std::sort(face4.data(), face4.data()+face4.size(), std::less<int>());

            auto result1 = std::make_tuple(face1[0], face1[1], face1[2]);
            auto result2 = std::make_tuple(face2[0], face2[1], face2[2]);
            auto result3 = std::make_tuple(face3[0], face3[1], face3[2]);
            auto result4 = std::make_tuple(face4[0], face4[1], face4[2]);

            if (m_surface2.find(result1) == m_surface2.end()){
                m_surface2[result1] = 1;
                fourths[result1] = i[3];
            } else {
                m_surface2[result1]++;
            }
            if (m_surface2.find(result2) == m_surface2.end()){
                m_surface2[result2] = 1;
                fourths[result2] = i[1];
            } else {
                m_surface2[result2]++;
            }
            if (m_surface2.find(result3) == m_surface2.end()){
                m_surface2[result3] = 1;
                fourths[result3] = i[0];
            } else {
                m_surface2[result3]++;
            }
            if (m_surface2.find(result4) == m_surface2.end()){
                m_surface2[result4] = 1;
                fourths[result4] = i[2];
            } else {
                m_surface2[result4]++;
            }
        }

        std::vector<Vector3i> faces;
        for (auto it = m_surface2.begin(); it != m_surface2.end(); ++it){
            auto f = it->first;
            if (it->second > 1){
                continue;
            }
            int zero = std::get<0>(f);
            int one = std::get<1>(f);
            int two = std::get<2>(f);
//            std::cout<< zero << " " << one << " " << two << std::endl;
            Vector3i face(zero, one, two);
            Vector3i newface = turnClockwise(face, fourths[f], vertices);
            faces.emplace_back(newface);
//            std::cout<< "new: " << newface[0] << " " << newface[1] << " " << newface[2] << std::endl;
        }
        m_shape.init(vertices, faces, tets);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 2, 0)));

    initGround();

//    std::cout << m_nodes[1]->m_position << std::endl;
//    m_nodes[0]->m_position += Vector3f(0,0.1f,0);
//    for (shared_ptr<Node> i : m_nodes){
//        i->m_position -= Vector3f(0,1.f,0);
//    }
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

    updateForces();

    std::vector<Vector3f> vertices;

    std::vector<Vector3f> oldPosV;
    std::vector<Vector3f> oldVelV;

//    for (shared_ptr<Node> i : m_nodes){
    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
//        sphereInfo check = checkSphereCollision(i->m_position);
//        if (check.intersect){
//            std::cout << "hi" <<std::endl;
//            i->m_force -= (i->m_force.norm()/3.f)*(1.f+(0.2f*check.depth))*check.normal;
//        }
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
//            i->m_force[1] -= i->m_force[1]*(1.f+(-2.f - i->m_position[1]));
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal; //instead of m_force.norm()
//            i->m_force = reflect(i->m_force, normal);
//            i->m_force[1] *= -1;
//            i->m_force[1] = 0.f;
        }
        Vector3f a = i->m_force/i->m_mass;
        oldPosV.push_back(i->m_position);
        oldVelV.push_back(i->m_velocity);

//        i->m_position = i->m_position + 0.5*seconds*i->m_velocity;
//        if (GRAVITY_ON && i->m_position[1] <= -2.f){
//            i->m_velocity *= -1;

//        } else {
            i->m_velocity = i->m_velocity + 0.5*seconds*a;
//        }

//        i->m_position = oldPos + 0.5*seconds*oldVel;
//        i->m_velocity = oldVel + 0.5*seconds*a;
    }

    updateForces();
    Vector3f prevA(0,0,0);

//    for (shared_ptr<Node> i : m_nodes){
    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
//            i->m_force[1] -= i->m_force[1]*(1.f+(-2.f - i->m_position[1]));
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
//            i->m_force = reflect(i->m_force, normal);
//            i->m_force[1] *= -1;
//            i->m_force[1] = 0.f;
        }
        Vector3f a = i->m_force/i->m_mass;
        Vector3f oldPos = oldPosV[k];//i->m_position;
        Vector3f oldVel = oldVelV[k];//i->m_velocity;

        i->m_position = oldPos + 0.5*seconds*i->m_velocity;
//        i->m_velocity = oldVel + 0.5*seconds*a;

//        i->m_position = oldPos + 0.5*seconds*oldVel;

        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            i->m_velocity *= -1;

        } else {
            i->m_velocity = oldVel + 0.5*seconds*a;
        }

//        if (abs((oldPos - i->m_position).norm()) > 1.f){
//            std::cout << "oldPos: " << oldPos <<std::endl;
//            std::cout << "oldVel: " << oldVel <<std::endl;
//            std::cout << "force: " << i->m_force <<std::endl;
//            std::cout << "newPos: " << i->m_position <<std::endl;
//            std::cout << "newVel: " << i->m_velocity <<std::endl;
//        }
//        if (k==0 && i->m_force[1] != -116){
//        if (a != prevA && abs(a[1] - prevA[1]) > 0.02f && prevA[1] != 0.f){
//            std::cout << "prev: " << prevA << std::endl;
//            std::cout << "accel: " << a << std::endl;
//        }

        vertices.push_back(i->m_position);
//        prevA = a;
    }

    m_shape.setVertices(vertices); //should prob set normals too
}

Vector3f Simulation::reflect(Vector3f in, Vector3f n){
    Vector3f out = in - (2*in.dot(n)*n);
    return out;
}

void Simulation::updateForces(){
    //clear forces
    for (shared_ptr<Node> n : m_nodes){
        n->m_force = Vector3f(0, 0, 0);
    }

    //get forces (stress/strain) for whole element
    //distribute/accumulate across 4 nodes
    for (shared_ptr<Element> e : m_elements){
        e->updateForces();

        if (GRAVITY_ON){
            e->m_n1->m_force += Vector3f(0, GRAVITY, 0);
            e->m_n2->m_force += Vector3f(0, GRAVITY, 0);
            e->m_n3->m_force += Vector3f(0, GRAVITY, 0);
            e->m_n4->m_force += Vector3f(0, GRAVITY, 0);
        }
    }
}

Vector3i Simulation::turnClockwise(Vector3i &face, int f, std::vector<Vector3f> const &vertices){
    Vector3f A = vertices[face[0]];
    Vector3f B = vertices[face[1]];
    Vector3f C = vertices[face[2]];
    Vector3f fourth = vertices[f];
    Vector3f AB = B-A;
    Vector3f AC = C-A;
    Vector3f normal = AB.cross(AC);
    normal.normalize();
    Vector3f centroid = (A+B+C)/3.f;
    Vector3f oppose = fourth - centroid;

    if (normal.dot(oppose) > 0.f){
        return Vector3i(face[2], face[1], face[0]);
    } else {
        return face;
    }
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
    groundVerts.emplace_back(Vector3f(-5, 0, -5));
    groundVerts.emplace_back(Vector3f(-5, 0, 5));
    groundVerts.emplace_back(Vector3f(5, 0, 5));
    groundVerts.emplace_back(Vector3f(5, 0, -5));
    groundFaces.emplace_back(Vector3i(0, 1, 2));
    groundFaces.emplace_back(Vector3i(0, 2, 3));
    m_ground.init(groundVerts, groundFaces);
}

//if distance between point and center of sphere < sphere's radius -> intersection
//adjust force by that distance
//normal = <2x, 2y, 2z> (just use node's position and hope for the best!)
sphereInfo Simulation::checkSphereCollision(Vector3f position){
    bool intersect = false;
    Vector3f normal(0,0,0);

    Vector3f sphereCenter(0.f, -2.f, 0.f);
    float radius = 1.f;
    float depth = abs((position-sphereCenter).norm());
    if (depth <= radius){
        intersect = true;
        normal = 2*position;
        normal.normalize();
    }
    return sphereInfo{intersect, normal, depth};
}
