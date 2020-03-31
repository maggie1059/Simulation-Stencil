#include "simulation.h"
#include <iostream>
#include "graphics/MeshLoader.h"
#define GRAVITY_ON true
#define SPHERE_ON true
#define DRAG_ON false
#define RECALC_AREA false
#define GRAVITY -0.2f //acceleration due to gravity
#define C_d 0.47f
#define AIR_DENSITY 1.5f

using namespace Eigen;

Simulation::Simulation()
{
    m_nodes.clear();
    m_elements.clear();
    m_surface_faces.clear();
    drag_surface = 0.f;
}

void Simulation::init(const std::string &filePath)
{
    // STUDENTS: This code loads up the tetrahedral mesh in 'example-meshes/single-tet.mesh'
    //    (note: your working directory must be set to the root directory of the starter code
    //    repo for this file to load correctly). You'll probably want to instead have this code
    //    load up a tet mesh based on e.g. a file path specified with a command line argument.
    std::vector<Vector3f> vertices;
    std::vector<Vector4i> tets;
    if(MeshLoader::loadTetMesh(filePath, vertices, tets)) {
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

        //hash all faces to isolate surface faces (only appear once)
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

        //store surface faces and correct orientation
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
            Vector3i newface = turnCounterclockwise(face, fourths[f], vertices);
            faces.emplace_back(newface);
            m_surface_faces.emplace_back(newface);
        }
        m_shape.init(vertices, faces, tets);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 2, 0)));

    initGround();

    //initialize sphere obstacle mesh
    if (SPHERE_ON){
        initSphere();
    }
//see the effects of moving one vertex from rest position (used for video)
//    m_nodes[0]->m_position += Vector3f(0,0.1f,0);

//see the effects of shifting entire shape up (used for video)-- comment out to see default!
    for (shared_ptr<Node> i : m_nodes){
        i->m_position += Vector3f(0,1.f,0);
    }
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

        //penalty force if collision with sphere
        if (SPHERE_ON){
            check = checkSphereCollision(i->m_position);
            if (check.intersect){
                i->m_force -= (i->m_force.norm())*(1.f+(0.2f*check.depth))*check.normal;
            }
        }

        //penalty force if collision with ground plane
        if (GRAVITY_ON && i->m_position[1] <= -2.f){
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        //calculate acceleration at old position
        Vector3f a = i->m_force/i->m_mass;
        oldPosV.push_back(i->m_position);
        oldVelV.push_back(i->m_velocity);

        //update midway position and velocity according to midpoint method
        i->m_position = i->m_position + 0.5*seconds*i->m_velocity;
        i->m_velocity = i->m_velocity + 0.5*seconds*a;
    }

    //update forces at midpoint
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
            Vector3f normal(0, 1.f, 0);
            i->m_force += abs(i->m_force[1])*(1.f+(0.2f*(-2.f - i->m_position[1])))*normal;
        }
        //get acceleration at new midway point
        Vector3f a = i->m_force/i->m_mass;
        Vector3f oldPos = oldPosV[k];
        Vector3f oldVel = oldVelV[k];

        //update position using old position and new velocity
        i->m_position = oldPos + seconds*i->m_velocity;

        //negate velocity if collision so that obstacles remain as rigid-body obstacles
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

        //update position/velocity to midpoint
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

        //update to second midpoint using previous midpoint velocity
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

        //update one full timestep using second midpoint velocity
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

        //set position using weighted average of velocities from previous steps
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
    }

    Vector3f avgVel;
    if (DRAG_ON){
        avgVel = getAvgVelocity();
        if (RECALC_AREA){
            getDragArea(avgVel);
        }
    }

    //add gravitational and drag force to all nodes (accumulated over shared nodes but divided by accumulated mass in update())
    for (shared_ptr<Node> n : m_nodes){
        if (GRAVITY_ON){
            Vector3f gravity(0, GRAVITY, 0);
            gravity *= n->m_mass;
            if (DRAG_ON){
                Vector3f drag;
                drag = C_d*AIR_DENSITY*0.5*drag_surface*avgVel.array().square();
                avgVel.normalize();
                //drag acts in direction opposite to velocity of entire object
                drag = -avgVel * drag.norm();
                gravity += drag;
            }
            n->m_force += gravity;
        }
    }
}

//gets average velocity of entire object to use for drag equation
Vector3f Simulation::getAvgVelocity(){
    Vector3f avgVel(0,0,0);
    for (shared_ptr<Node> n : m_nodes){
        avgVel += n->m_velocity;
    }
    avgVel = avgVel/m_nodes.size();
    return avgVel;
}

//turns order of nodes in a face counterclockwise for rendering
Vector3i Simulation::turnCounterclockwise(Vector3i &face, int f, std::vector<Vector3f> const &vertices){
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
        normal = -normal;
        face = Vector3i(face[2], face[1], face[0]);
    }

    if (DRAG_ON){
        Vector3f down(0, -1.f, 0);

        //surface area for drag accumulated for faces facing down
        if (normal.dot(down) > 0.f){
            float ab = (B-A).norm();
            float bc = (C-B).norm();
            float ca = (A-C).norm();
            float s = (ab+bc+ca)/2.f;

            float base = sqrt(s*(s-ab)*(s-bc)*(s-ca));
            drag_surface += base;
        }
    }

    return face;
}

//get surface area for drag calculations
void Simulation::getDragArea(Vector3f avgVel){
    drag_surface = 0;
    for (auto face : m_surface_faces){
        Vector3f A = m_nodes[face[0]]->m_position;
        Vector3f B = m_nodes[face[1]]->m_position;
        Vector3f C = m_nodes[face[2]]->m_position;
        Vector3f AB = B-A;
        Vector3f AC = C-A;
        Vector3f normal = AB.cross(AC);
        normal.normalize();

        //surface area for drag accumulated for faces facing direction of avg velocity
        if (normal.dot(avgVel) > 0.f){
            float ab = (B-A).norm();
            float bc = (C-B).norm();
            float ca = (A-C).norm();
            float s = (ab+bc+ca)/2.f;

            float base = sqrt(s*(s-ab)*(s-bc)*(s-ca));
            drag_surface += base;
        }
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
    groundVerts.emplace_back(Vector3f(-25, 0, -25));
    groundVerts.emplace_back(Vector3f(-25, 0, 25));
    groundVerts.emplace_back(Vector3f(25, 0, 25));
    groundVerts.emplace_back(Vector3f(25, 0, -25));
    groundFaces.emplace_back(Vector3i(0, 1, 2));
    groundFaces.emplace_back(Vector3i(0, 2, 3));
    m_ground.init(groundVerts, groundFaces);
}

//adds sphere mesh for sphere obstacle
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
            Vector3i newface = turnCounterclockwise(face, fourths2[f], vertices);
            faces.emplace_back(newface);
        }
        m_sphere.init(vertices, faces, tets);
    }
}

sphereInfo Simulation::checkSphereCollision(Vector3f position){
    bool intersect = false;
    Vector3f normal(0,0,0);

    //define location of sphere
    Vector3f sphereCenter(0.f, -2.f, 0.f);
    float radius = 1.f;

    //if distance between point and center of sphere < sphere's radius -> intersection
    //adjust force by that distance
    float depth = abs((position-sphereCenter).norm());
    if (depth <= radius){
        intersect = true;
        normal = 2*position;
        normal.normalize();
    }
    return sphereInfo{intersect, normal, depth};
}
