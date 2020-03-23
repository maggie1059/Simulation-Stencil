#include "Element.h"
#include <iostream>
#include <memory>
#define DENSITY 1200.f
#define PHI 5.f //incompressibility for viscosity
#define PSI 5.f //rigidity for viscosity
#define LAMBDA 1e3 //incompressibility
#define MU 1e3 //rigidity

using namespace Eigen;


Element::Element(shared_ptr<Node> n1, shared_ptr<Node> n2, shared_ptr<Node> n3, shared_ptr<Node> n4)
    : m_n1(n1), m_n2(n2), m_n3(n3), m_n4(n4)
{
    setBeta();
}

Element::~Element(){

}

void Element::setMasses(){
    Vector3f A = m_n1->m_position;
    Vector3f B = m_n2->m_position;
    Vector3f C = m_n3->m_position;
    Vector3f D = m_n4->m_position;

    float ab = (B-A).norm();
    float bc = (C-B).norm();
    float ca = (A-C).norm();
    float s = (ab+bc+ca)/2.f;

    float base = sqrt(s*(s-ab)*(s-bc)*(s-ca));

    Vector3f AB = B-A;
    Vector3f AC = C-A;
    Vector3f normal = AB.cross(AC);
    normal.normalize();
    Vector3f centroid = (A+B+C)/3.f;
    Vector3f oppose = D - centroid;

    if (normal.dot(oppose) < 0.f){
        normal = -normal;
    }

    float height = fabs(normal.dot(D-A));

    float volume = (1.f/3.f) * base * height;
    float mass = DENSITY * volume/4.f;
    m_n1->setMass(mass);
    m_n2->setMass(mass);
    m_n3->setMass(mass);
    m_n4->setMass(mass);
}

void Element::setBeta(){
    Vector3f p1 = m_n1->m_position-m_n4->m_position;
    Vector3f p2 = m_n2->m_position-m_n4->m_position;
    Vector3f p3 = m_n3->m_position-m_n4->m_position;

//    std::cout << p1 << std::endl;

//    m_beta << p1[0], p1[1], p1[2],
//            p2[0], p2[1], p2[2],
//            p3[0], p3[1], p3[2];

    m_beta << p1[0], p2[0], p3[0],
            p1[1], p2[1], p3[1],
            p1[2], p2[2], p3[2];

    m_beta = m_beta.inverse().eval();
}

Vector3f Element::addForces(Matrix3f dx_du, Matrix3f stress, shared_ptr<Node> n1, std::vector<shared_ptr<Node>> nodes){
    Vector3f A = nodes[0]->m_position;
    Vector3f B = nodes[1]->m_position;
    Vector3f C = nodes[2]->m_position;
//    std::cout << A <<std::endl;
//    std::cout << B <<std::endl;
//    std::cout << C <<std::endl;

    float ab = (B-A).norm();
    float bc = (C-B).norm();
    float ca = (A-C).norm();
    float s = (ab+bc+ca)/2.f;

    float area = sqrt(s*(s-ab)*(s-bc)*(s-ca));

    Vector3f AB = B-A;
    Vector3f AC = C-A;
    Vector3f normal = AB.cross(AC);
    normal.normalize();
    Vector3f centroid = (A+B+C)/3.f;
    Vector3f oppose = centroid - n1->m_position;
    oppose.normalize();

    if (normal.dot(oppose) < 0.f){
        normal = -normal;
    }

//    std::cout << dx_du << std::endl;
//    std::cout << stress <<std::endl;
//    std::cout << normal <<std::endl;

    Vector3f f = dx_du*stress*area*normal;

//    std::cout << f <<std::endl;
    return f;
}

void Element::updateForces(){
    Vector3f p1 = m_n1->m_position-m_n4->m_position;
    Vector3f p2 = m_n2->m_position-m_n4->m_position;
    Vector3f p3 = m_n3->m_position-m_n4->m_position;

    Vector3f v1 = m_n1->m_velocity-m_n4->m_velocity;
    Vector3f v2 = m_n2->m_velocity-m_n4->m_velocity;
    Vector3f v3 = m_n3->m_velocity-m_n4->m_velocity;

    Matrix3f P;

    P << p1[0], p2[0], p3[0],
            p1[1], p2[1], p3[1],
            p1[2], p2[2], p3[2];

    Matrix3f V;

    V << v1[0], v2[0], v3[0],
            v1[1], v2[1], v3[1],
            v1[2], v2[2], v3[2];

    Matrix3f dx_du = P*m_beta;
    Matrix3f dx_dot_du = V*m_beta;
    Matrix3f strain = (dx_du.transpose() * dx_du) - Matrix3f::Identity();
    Matrix3f stress1 = (LAMBDA*Matrix3f::Identity()*strain.trace()) + (2*MU*strain);
    Matrix3f strain2 = (dx_du.transpose()*dx_dot_du) + (dx_dot_du.transpose()*dx_du);
    Matrix3f stress2 = (PHI*Matrix3f::Identity()*strain2.trace()) + (2*PSI*strain2);
    Matrix3f stress = stress1 + stress2;

//    std::cout << "stress1: " << stress1 <<std::endl;
//    std::cout << "stress2: " <<stress2 << std::endl;
//    std::cout << "P: " << P <<std::endl;
//    std::cout << "node 1: " << m_n1->m_position <<std::endl;
//    std::cout << "node 2: " << m_n2->m_position <<std::endl;
//    std::cout << "node 3: " << m_n3->m_position <<std::endl;
//    std::cout << "node 4: " << m_n4->m_position <<std::endl;

//    std::cout << "V: " << V <<std::endl;

    m_n1->m_force += addForces(dx_du, stress, m_n1, std::vector<shared_ptr<Node>>{m_n2, m_n3, m_n4});
    m_n2->m_force += addForces(dx_du, stress, m_n2, std::vector<shared_ptr<Node>>{m_n1, m_n3, m_n4});
    m_n3->m_force += addForces(dx_du, stress, m_n3, std::vector<shared_ptr<Node>>{m_n2, m_n1, m_n4});
    m_n4->m_force += addForces(dx_du, stress, m_n4, std::vector<shared_ptr<Node>>{m_n2, m_n3, m_n1});
}
