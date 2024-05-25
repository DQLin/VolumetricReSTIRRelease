Custom Build of Assimp
=======
* https://github.com/assimp/assimp
* Branch: `v5.0.0`
* Visual Studio 2017 version 15.9.3
* Compiler: MSVC 19.16.27024.1
* Build Config: Release Win64

The following code has been added to `OptimizeMeshesProcess::CanJoin()` in `code/OptimizeMeshes.cpp`:

```
    // If either material is emissive, do not join the meshes together!
    aiMaterial *matlA = mScene->mMaterials[ma->mMaterialIndex], *matlB = mScene->mMaterials[mb->mMaterialIndex];
    aiColor3D colA, colB;
    matlA->Get(AI_MATKEY_COLOR_EMISSIVE, colA);
    matlB->Get(AI_MATKEY_COLOR_EMISSIVE, colB);
    if (!colA.IsBlack() || !colB.IsBlack() || matlA->GetTextureCount(aiTextureType_EMISSIVE) || matlB->GetTextureCount(aiTextureType_EMISSIVE))
        return false;
```