#include <AccelEngine/ParticleWorld.h>

using namespace AccelEngine;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
    : resolver(iterations),
      maxContacts(maxContacts),
      firstParticle(nullptr),
      firstContactGen(nullptr)
{
    contacts = new ParticleContact[maxContacts];
}

void ParticleWorld::startFrame()
{
    ParticleRegistration *reg = firstParticle;
    while (reg)
    {
        reg->particle->clearAccumulator();
        reg = reg->next;
    }
}

unsigned ParticleWorld::generateContacts()
{
    unsigned limit = maxContacts;
    ParticleContact *nextContact = contacts;

    ContactGenRegistration *reg = firstContactGen;
    while (reg)
    {
        unsigned used = reg->gen->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        if (limit <= 0)
            break;

        reg = reg->next;
    }

    return maxContacts - limit;
}

void ParticleWorld::integrate(real duration)
{
    ParticleRegistration *reg = firstParticle;
    while (reg)
    {
        reg->particle->integrate(duration);
        reg = reg->next;
    }
}

void ParticleWorld::runPhysics(real duration)
{
    registry.updateForces(duration);

    integrate(duration);

    unsigned usedContacts = generateContacts();

    if (usedContacts > 0)
        resolver.resolveContact(contacts, usedContacts, duration);
}

void ParticleWorld::addParticle(Particle *p)
{
    ParticleRegistration *reg = new ParticleRegistration();
    reg->particle = p;
    reg->next = firstParticle;
    firstParticle = reg;
}

void ParticleWorld::addContactGenerator(ParticleContactGenerator *gen)
{
    ContactGenRegistration *reg = new ContactGenRegistration();
    reg->gen = gen;
    reg->next = firstContactGen;
    firstContactGen = reg;
}
