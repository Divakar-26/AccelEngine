#include "AccelEngine/pfgen.h"

using namespace AccelEngine;

void ParticleForceRegistry::add(Particle * p, ParticleForceGenerator * fg){
    ParticleForceRegistration newFgReg;
    newFgReg.p = p;
    newFgReg.fg = fg;

    registration.push_back(newFgReg);
}

void ParticleForceRegistry::remove(Particle * p, ParticleForceGenerator * fg){

}

void ParticleForceRegistry::clear(){
    registration.clear();
}

void ParticleForceRegistry::updateForces(real duration){
    Registry::iterator i = registration.begin();
    for(; i != registration.end(); i++){
        i->fg->updateForce(i->p, duration);
    }
}

void ParticleGravity::updateForce(Particle * p, real duration){
    if(!p->hasFiniteMass()){
        return;
    }

    p->addForce(gravity * (1.0f / p->inverseMass));
}

void ParticleDrag::updateForce(Particle * p, real duration){
    Vector2 force = p->getVelocity();

    real dragCoeff = force.magnitude();
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

    force.normalize();
    force *= -dragCoeff;
    p->addForce(force);
}

void ParticleSpring::updateForce(Particle * p, real duration){
    Vector2 force = p->getPosition();
    force -= other->getPosition();

    real magnitude = force.magnitude();
    magnitude = magnitude - restLenght;
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

void ParticleAnchoredSpring::updateForce(Particle * p, real duration){
    Vector2 force = p->getPosition();
    force -= *anchor;

    real magnitude = force.magnitude();
    magnitude = magnitude - restLenght;
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}

void ParticleBungee::updateForce(Particle * p, real duration){
    Vector2 force = p->getPosition();
    force -= other->getPosition();

    real magnitude = force.magnitude();
    if(magnitude <= restLenght){
        return;
    }
    magnitude = springConstant * (magnitude - restLenght);

    force.normalize();
    force *= -magnitude;
    p->addForce(force);
}