#pragma once
#include <AccelEngine/body.h>
#include <AccelEngine/core.h>
#include <AccelEngine/narrow_collision.h>
#include <vector>

namespace AccelEngine
{
    class ContactResolver
    {
    public:
        static void ResolveContacts(std::vector<Contact> &contacts, float dt);
    };
}
