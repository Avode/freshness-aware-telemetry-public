#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <iostream>
int main(int argc, char** argv) {
  if(argc!=3) return 2;
  Assimp::Importer loader;
  const auto* scene=loader.ReadFile(argv[1], aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|aiProcess_GenSmoothNormals);
  if(!scene){std::cerr<<loader.GetErrorString();return 1;}
  Assimp::Exporter exporter;
  if(exporter.Export(scene,"glb2",argv[2])!=AI_SUCCESS){std::cerr<<exporter.GetErrorString();return 1;}
  return 0;
}
