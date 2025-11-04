#include "AccelEngine/ParticleContact.h"
#include "AccelEngine/pfgen.h"
#include <SDL3/SDL_events.h>
#include <compare>

using namespace AccelEngine;

void ParticleContact::resolve(real duration)
{
  resolveVelocity(duration);
  resolveInterpenetration(duration);
}

real ParticleContact::calculateSeperatingVelocity() const
{
  Vector2 relativeVelocity = particle[0]->getVelocity();
  if (particle[1])
    relativeVelocity -= particle[1]->getVelocity();
  return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real duration)
{
  real seperatingVelocity = calculateSeperatingVelocity();

  if (seperatingVelocity > 0)
  {
    return;
  }

  real newSeperatingVelocity = -seperatingVelocity * restitution;

  Vector2 accCausedVelocity = particle[0]->acceleration;
  if (particle[1])
    accCausedVelocity -= particle[1]->acceleration;
  real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;
  if (accCausedSepVelocity < 0)
  {
    newSeperatingVelocity += restitution * accCausedSepVelocity;

    if (newSeperatingVelocity < 0)
      newSeperatingVelocity = 0;
  }

  real deltaVelocity = newSeperatingVelocity - seperatingVelocity;

  real totalInverseMass = particle[0]->inverseMass;
  if (particle[1])
    totalInverseMass += particle[1]->inverseMass;

  if (totalInverseMass <= 0)
    return;

  real impulse = deltaVelocity / totalInverseMass;
  Vector2 impulsePerIMass = contactNormal * impulse;

  particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->inverseMass);
  if (particle[1])
  {
    particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerIMass * -particle[1]->inverseMass);
  }
}

void ParticleContact::resolveInterpenetration(real duration)
{
  if (penetration <= 0)
    return;

  real totalInverseMass = particle[0]->inverseMass;
  if (particle[1])
    totalInverseMass += particle[1]->inverseMass;

  if (totalInverseMass <= 0)
    return;

  Vector2 movePerIMass = contactNormal * (penetration / totalInverseMass);

  particle[0]->setPosition(particle[0]->getPosition() + movePerIMass * particle[0]->inverseMass);
  if (particle[1])
  {
    particle[1]->setPosition(particle[1]->getPosition() - movePerIMass * particle[1]->inverseMass);
  }
}

void ParticleContactResolver::resolveContact(ParticleContact *contactArray, unsigned numContact, real duration)
{
  itertaionsUsed = 0;
  while (itertaionsUsed < iterations)
  {
    real max = 0;
    unsigned maxIndex = numContact;

    for (unsigned i = 0; i < numContact; i++)
    {
      real sepVel = contactArray[i].calculateSeperatingVelocity();
      if (sepVel < max)
      {
        max = sepVel;
        maxIndex = i;
      }
    }

    contactArray[maxIndex].resolve(duration);
    itertaionsUsed++;
  }
}