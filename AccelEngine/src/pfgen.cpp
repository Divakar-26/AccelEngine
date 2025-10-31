#include "pfgen.h"

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

    p->addForce(gravity * p->getMass());
}

