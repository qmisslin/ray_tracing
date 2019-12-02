// main.cpp : Basic raytracing algorithm.
//

#include <stdio.h> 
#include <string.h>

#include <vector> 

#include "../include/scene.h"

const int RX=800;
const int RY=800;

scene<float> sc(RX,RY);


//  PROGRAMM LOADS A SCENE and WAITS for EVENTS
int main(int argc, char **argv)
{

  // import the scene: .obj format
 sc.importOBJ("scene");

  // add light source
  sc.pushLight(vec4<float>(0.0f, 10.0f, 9.5f, 1.0f), luminance(1.0f));
  
  // set lookAt
  sc.lookAt(vec3<float>(10.0, 10.0, 5.0), vec3<float>(0.0));
  
  // compute raytracing
  sc.raytracing("mypicture.ppm");
  return 0;            
}


