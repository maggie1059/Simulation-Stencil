#include "simulation.h"
#include <iostream>
#include "graphics/MeshLoader.h"
#define GRAVITY -2.f
#define GRAVITY_ON true
#define SPHERE_ON true

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

        for (auto t : tets){
            shared_ptr<Element> e(new Element(m_nodes[t[0]], m_nodes[t[1]], m_nodes[t[2]], m_nodes[t[3]]));
            m_elements.push_back(e);
        }

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
            Vector3i face(zero, one, two);
            Vector3i newface = turnClockwise(face, fourths[f], vertices);
            faces.emplace_back(newface);
        }
        m_shape.init(vertices, faces, tets);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 2, 0)));

    initGround();

    if (SPHERE_ON){
        initSphere();
    }

//        m_nodes[0]->m_position += Vector3f(0,0.1f,0);
//    for (shared_ptr<Node> i : m_nodes){
//        i->m_position += Vector3f(0,1.f,0);
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

    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
        sphereInfo check{false, Vector3f(0,0,0), 0};

        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }

        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal; //instead of m_force.norm()
        }
        Vector3f a = i->m_force/i->m_mass;
        oldPosV.push_back(i->m_position);
        oldVelV.push_back(i->m_velocity);

        i->m_position = i->m_position + 0.5*seconds*i->m_velocity;
        i->m_velocity = i->m_velocity + 0.5*seconds*a;
    }

    updateForces();

    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];

        sphereInfo check{false, Vector3f(0,0,0), 0};

        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
//            i->m_force[1] -= i->m_force[1]*(1.f+(-2.f - i->m_position[1]));
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        Vector3f a = i->m_force/i->m_mass;
        Vector3f oldPos = oldPosV[k];
        Vector3f oldVel = oldVelV[k];

        i->m_position = oldPos + 0.5*seconds*i->m_velocity;

        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            i->m_velocity *= -1;
        } else if (SPHERE_ON && check.intersect){
            i->m_velocity *= -1;
        } else {
            i->m_velocity = oldVel + 0.5*seconds*a;
        }
        vertices.push_back(i->m_position);
    }
    m_shape.setVertices(vertices);
}

void Simulation::updateRK4(float seconds) {
    updateForces();

    std::vector<Vector3f> vertices;

    std::vector<Vector3f> oldPosV;
    std::vector<Vector3f> vel0;
    std::vector<Vector3f> vel1;
    std::vector<Vector3f> vel2;

    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
        sphereInfo check{false, Vector3f(0,0,0), 0};
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }
        Vector3f a = i->m_force/i->m_mass;
        oldPosV.push_back(i->m_position);
        vel0.push_back(i->m_velocity);

        i->m_position = i->m_position + 0.5*seconds*i->m_velocity;
        i->m_velocity = i->m_velocity + 0.5*seconds*a;
    }

    updateForces();

    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
        sphereInfo check{false, Vector3f(0,0,0), 0};
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }
        Vector3f a = i->m_force/i->m_mass;
        Vector3f oldPos = oldPosV[k];
        Vector3f oldVel = vel0[k];
        vel1.push_back(i->m_velocity);

        i->m_position = oldPos + 0.5*seconds*i->m_velocity;
        i->m_velocity = oldVel + 0.5*seconds*a;
    }

    updateForces();

    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
        sphereInfo check{false, Vector3f(0,0,0), 0};
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }
        Vector3f a = i->m_force/i->m_mass;
        Vector3f oldPos = oldPosV[k];
        Vector3f oldVel = vel0[k];
        vel2.push_back(i->m_velocity);

        i->m_position = oldPos + seconds*i->m_velocity;
        i->m_velocity = oldVel + 0.5*seconds*a;
    }

    updateForces();

    for (int k = 0; k < m_nodes.size(); k++){
        shared_ptr<Node> i = m_nodes[k];
        sphereInfo check{false, Vector3f(0,0,0), 0};
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }
        Vector3f a = i->m_force/i->m_mass;
        Vector3f oldPos = oldPosV[k];
        Vector3f oldVel = vel0[k];

        i->m_position = oldPos + seconds*((vel0[k] + (2*vel1[k]) + (2*vel2[k]) + i->m_velocity)/6.f);
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            i->m_velocity *= -1;
        } else if (SPHERE_ON && check.intersect){
            i->m_velocity *= -1;
        } else {
            i->m_velocity = oldVel + seconds*a;
        }

        vertices.push_back(i->m_position);
    }
    m_shape.setVertices(vertices);
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

Vector3f Simulation::reflect(Vector3f in, Vector3f n){
    Vector3f out = in - (2*in.dot(n)*n);
    return out;
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
    if (SPHERE_ON){
        m_sphere.draw(shader);
    }
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

void Simulation::initSphere(){
    std::vector<Vector3f> vertices;
    std::vector<Vector4i> tets;
    std::vector<shared_ptr<Node>> nodes;
    std::vector<shared_ptr<Element>> elements;
    std::unordered_map<std::tuple<int, int, int>, int, hash_tuple> surface;
    std::unordered_map<std::tuple<int, int, int>, int, hash_tuple> fourths2;
    if(MeshLoader::loadTetMesh("example-meshes/sphere.mesh", vertices, tets)) {
        for (auto v : vertices){
            shared_ptr<Node> n(new Node(v));
            nodes.push_back(n);
        }

        for (auto t : tets){
            shared_ptr<Element> e(new Element(nodes[t[0]], nodes[t[1]], nodes[t[2]], nodes[t[3]]));
            elements.push_back(e);
        }

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

            if (surface.find(result1) == surface.end()){
                surface[result1] = 1;
                fourths2[result1] = i[3];
            } else {
                surface[result1]++;
            }
            if (surface.find(result2) == surface.end()){
                surface[result2] = 1;
                fourths2[result2] = i[1];
            } else {
                surface[result2]++;
            }
            if (surface.find(result3) == surface.end()){
                surface[result3] = 1;
                fourths2[result3] = i[0];
            } else {
                surface[result3]++;
            }
            if (surface.find(result4) == surface.end()){
                surface[result4] = 1;
                fourths2[result4] = i[2];
            } else {
                surface[result4]++;
            }
        }

        std::vector<Vector3i> faces;
        for (auto it = surface.begin(); it != surface.end(); ++it){
            auto f = it->first;
            if (it->second > 1){
                continue;
            }
            int zero = std::get<0>(f);
            int one = std::get<1>(f);
            int two = std::get<2>(f);
            Vector3i face(zero, one, two);
            Vector3i newface = turnClockwise(face, fourths2[f], vertices);
            faces.emplace_back(newface);
        }
        m_sphere.init(vertices, faces, tets);
    }
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
