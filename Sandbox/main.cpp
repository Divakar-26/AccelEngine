#include <SDL3/SDL.h>
#include <iostream>
#include <AccelEngine/core.h>

int main(){
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window * window = SDL_CreateWindow("Hello", 800, 800, SDL_WINDOW_RESIZABLE);

    AccelEngine::Vector3 a(1,2,3), b(4,5,6);
    a = a % b;
    std::cout << a.x <<" "<< a.y <<" " << a.z <<" " << std::endl;
    std::cout<<"The magnitude of vector is : "<<a.magnitude()<<std::endl;


    return 0;
}