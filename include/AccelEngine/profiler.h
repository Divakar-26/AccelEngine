#pragma once

#include <unordered_map>
#include <string>
#include <chrono>

#define PROFILE_SCOPE(name) ScopedProfile timer##__LINE__(name)

struct ProfileData{
    double totalTime = 0;
    int callCount = 0;
};

inline std::unordered_map<std::string, ProfileData> aProfileData;

class ScopedProfile{
    public:
        ScopedProfile(const char * name) : name(name), start(std::chrono::high_resolution_clock::now()) {}
        ~ScopedProfile(){
            auto end = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(end - start).count();

            auto & entry  = aProfileData[name];
            entry.totalTime += ms;
            entry.callCount++;
        }

    private: 
        const char * name;
        std::chrono::high_resolution_clock::time_point start;
};