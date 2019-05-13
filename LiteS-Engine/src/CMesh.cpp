#include "CMesh.h"
#include "stb_image.h"
using namespace std;

GLuint CMesh::boundIndex[36] = {0, 1, 2, 0, 2, 3, 0, 5, 1, 0, 6, 5,
                                0, 3, 7, 0, 7, 6, 4, 5, 6, 4, 6, 7,
                                4, 2, 3, 4, 3, 7, 4, 5, 1, 4, 1, 2};

CMesh::CMesh() : isRenderNormal(false) {}

void CMesh::saveMesh(const CMesh* v_mesh, string v_outName) {
  aiScene* scene = new aiScene;
  scene->mRootNode = new aiNode;

  scene->mMeshes = new aiMesh*[1];
  scene->mMeshes[0] = new aiMesh;
  scene->mNumMeshes = 1;

  // scene->mMaterials = new aiMaterial*[1];
  // scene->mMaterials[0] = nullptr;
  // scene->mNumMaterials = 1;

  // scene->mMaterials[0] = new aiMaterial();

  // scene->mMeshes[0]->mMaterialIndex = 0;

  scene->mRootNode->mMeshes = new unsigned int[1];
  scene->mRootNode->mMeshes[0] = 0;
  scene->mRootNode->mNumMeshes = 1;

  auto pMesh = scene->mMeshes[0];

  size_t numValidPoints = v_mesh->vertices.size();

  pMesh->mVertices = new aiVector3D[numValidPoints];
  pMesh->mNormals = new aiVector3D[numValidPoints];
  pMesh->mNumVertices = (unsigned int)numValidPoints;

  int i = 0;
  for (auto& p : v_mesh->vertices) {
    pMesh->mVertices[i] = aiVector3D(v_mesh->vertices[i].Position.x,
                                     v_mesh->vertices[i].Position.y,
                                     v_mesh->vertices[i].Position.z);
    pMesh->mNormals[i] =
        aiVector3D(v_mesh->vertices[i].Normal.x, v_mesh->vertices[i].Normal.y,
                   v_mesh->vertices[i].Normal.z);
    ++i;
  }

  Assimp::Exporter* mAiExporter = new Assimp::Exporter;
  Assimp::ExportProperties* properties = new Assimp::ExportProperties;
  properties->SetPropertyBool(AI_CONFIG_EXPORT_POINT_CLOUDS, true);
  mAiExporter->Export(scene, "ply", v_outName, 0, properties);

  cout << mAiExporter->GetErrorString() << endl;
  // delete properties;
  return;
}

unsigned int TextureFromFile(const char* path, const string& directory) {
  string filename = string(path);
  filename = directory + '/' + filename;

  unsigned int textureID;
  glGenTextures(1, &textureID);

  int width, height, nrComponents;
  unsigned char* data =
      stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
  if (data) {
    GLenum format;
    if (nrComponents == 1)
      format = GL_RED;
    else if (nrComponents == 3)
      format = GL_RGB;
    else if (nrComponents == 4)
      format = GL_RGBA;

    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format,
                 GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
  } else {
    std::cout << "Texture failed to load at path: " << path << std::endl;
    stbi_image_free(data);
  }

  return textureID;
}

void CMesh::loadMaterialTextures(aiMaterial* mat, aiTextureType type,
                                 std::string typeName,
                                 std::vector<Texture>& textures) {
  for (unsigned int i = 0; i < mat->GetTextureCount(type); i++) {
    aiString str;
    mat->GetTexture(type, i, &str);
    // check if texture was loaded before and if so, continue to next iteration:
    // skip loading a new texture
    bool skip = false;
    for (unsigned int j = 0; j < textures_loaded.size(); j++) {
      if (std::strcmp(textures_loaded[j].path.data, str.C_Str()) == 0) {
        textures.push_back(textures_loaded[j]);
        skip = true;  // a texture with the same filepath has already been
                      // loaded, continue to next one. (optimization)
        break;
      }
    }
    if (!skip) {  // if texture hasn't been loaded already, load it
      Texture texture;
      texture.id = TextureFromFile(str.C_Str(), this->directory);
      texture.type = typeName;
      texture.path = str.C_Str();
      textures.push_back(texture);
      textures_loaded.push_back(texture);
    }
  }
}

void CMesh::processNode(aiNode* node, const aiScene* scene) {
  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    std::vector<Vertex>& vertices = this->vertices;
    std::vector<unsigned int>& indices = this->indices;

    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
      Vertex vertex;
      glm::vec3 vector;
      vector.x = mesh->mVertices[i].x;
      vector.y = mesh->mVertices[i].y;
      vector.z = mesh->mVertices[i].z;
      vertex.Position = vector;
      // normals
      if (mesh->mNormals) {
        vector.x = mesh->mNormals[i].x;
        vector.y = mesh->mNormals[i].y;
        vector.z = mesh->mNormals[i].z;
        vertex.Normal = vector;
      }

      //if (mesh->mColors[0]) {
      //  vector.x = mesh->mColors[0][i][0];
      //  vector.y = mesh->mColors[0][i][1];
      //  vector.z = mesh->mColors[0][i][2];
      //  vertex.Color = vector;
      //} else
       vertex.Color = glm::vec3(1.f,1.f,1.f);

      vertices.push_back(vertex);
    }
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
      aiFace face = mesh->mFaces[i];
      for (unsigned int j = 0; j < face.mNumIndices; j++)
        indices.push_back(face.mIndices[j]);
    }

    // With Material
    int count = 0;
    aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
    MeshMaterial meshMaterial;
    aiColor3D color(1.f, 1.f, 1.f);
    int tempInt;
    float tempFLoat;
    aiString testAiString;
    if (AI_SUCCESS == material->Get(AI_MATKEY_NAME, testAiString)) {
      meshMaterial.name = testAiString.data;
      ++count;
    }
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_DIFFUSE, color)) {
      meshMaterial.diffuse.x = color.r;
      meshMaterial.diffuse.y = color.g;
      meshMaterial.diffuse.z = color.b;
      ++count;
    }
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR, color)) {
      meshMaterial.specular.x = color.r;
      meshMaterial.specular.y = color.g;
      meshMaterial.specular.z = color.b;
      ++count;
    }

    if (AI_SUCCESS == material->Get(AI_MATKEY_SHADING_MODEL, tempInt)) {
      meshMaterial.shadingModel = tempInt;
      ++count;
    }

    if (AI_SUCCESS == material->Get(AI_MATKEY_OPACITY, tempFLoat)) {
      meshMaterial.opacity = tempFLoat;
      ++count;
    }
    if (AI_SUCCESS == material->Get(AI_MATKEY_SHININESS, tempFLoat)) {
      meshMaterial.shininess = tempFLoat;
      ++count;
    }
    if (count != material->mNumProperties)
      cout << "Have unresolved properties" << endl;

    material = scene->mMaterials[mesh->mMaterialIndex];
    // 1. diffuse maps
    vector<Texture> diffuseMaps;
    loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse",
                         diffuseMaps);
    textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
    // 2. specular maps
    vector<Texture> specularMaps;
    loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular",
                         specularMaps);
    textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    // 3. normal maps
    std::vector<Texture> normalMaps;
    loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal",
                         normalMaps);
    textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
    // 4. height maps
    std::vector<Texture> heightMaps;
    loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height",
                         heightMaps);
    textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
  }
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    processNode(node->mChildren[i], scene);
  }
}

void CMesh::loadMeshFromFile(const std::string& vPath, bool vIsRender) {
  Assimp::Importer importer;
  const aiScene* scene;

  scene = importer.ReadFile(vPath, aiProcess_Triangulate);
  // check for errors
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE ||
      !scene->mRootNode) {
    std::cout << "Read Model Error" << importer.GetErrorString() << std::endl;
    throw std::string("ERROR::ASSIMP:: ") + importer.GetErrorString();
  }

  processNode(scene->mRootNode, scene);
  std::cout << "Read Model Done" << std::endl;
}
